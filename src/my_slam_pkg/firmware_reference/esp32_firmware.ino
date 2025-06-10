#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <sensor_msgs/msg/joint_state.h>
#include <ESP32Encoder.h>

// === HARDWARE CONFIGURATION ===
// Motor pins - Front Left (FL)
#define MOTOR_FL_EN_PIN 25
#define MOTOR_FL_DIR1_PIN 26
#define MOTOR_FL_DIR2_PIN 27
#define MOTOR_FL_ENC_A_PIN 34
#define MOTOR_FL_ENC_B_PIN 35

// Motor pins - Front Right (FR)
#define MOTOR_FR_EN_PIN 14
#define MOTOR_FR_DIR1_PIN 12
#define MOTOR_FR_DIR2_PIN 13
#define MOTOR_FR_ENC_A_PIN 36
#define MOTOR_FR_ENC_B_PIN 39

// Motor pins - Rear Left (RL)
#define MOTOR_RL_EN_PIN 33
#define MOTOR_RL_DIR1_PIN 32
#define MOTOR_RL_DIR2_PIN 18
#define MOTOR_RL_ENC_A_PIN 23
#define MOTOR_RL_ENC_B_PIN 19

// Motor pins - Rear Right (RR)
#define MOTOR_RR_EN_PIN 15
#define MOTOR_RR_DIR1_PIN 4
#define MOTOR_RR_DIR2_PIN 16
#define MOTOR_RR_ENC_A_PIN 21
#define MOTOR_RR_ENC_B_PIN 22

#define STATUS_LED_PIN 2

// PWM Configuration
#define PWM_FREQ 1000
#define PWM_RESOLUTION 8
#define MAX_PWM_VALUE 255

// Robot physical constants
#define WHEEL_BASE 0.20  // Distance between front and rear wheels (meters)
#define TRACK_WIDTH 0.15 // Distance between left and right wheels (meters)
#define WHEEL_RADIUS 0.033 // Wheel radius (meters)

// Encoder configuration
#define ENCODER_PPR 360  // Pulses per revolution of encoder
#define COUNTS_PER_REVOLUTION (ENCODER_PPR * 4) // x4 for quadrature, no gear ratio

// Encoder direction correction flags
#define FL_ENCODER_REVERSE true
#define FR_ENCODER_REVERSE false
#define RL_ENCODER_REVERSE true
#define RR_ENCODER_REVERSE false

// Motor direction correction flags
#define FL_MOTOR_REVERSE false
#define FR_MOTOR_REVERSE false
#define RL_MOTOR_REVERSE false
#define RR_MOTOR_REVERSE false

// === ENCODER AVERAGING CONFIGURATION ===
#define AVERAGING_SAMPLES 5    // Number of samples to average (reduced for better responsiveness)
#define SMOOTHING_ALPHA 0.15   // Exponential smoothing factor (0.0-1.0, higher = less smoothing)

// Micro-ROS objects
rcl_subscription_t subscriber;
rcl_publisher_t encoder_publisher;
rcl_publisher_t wheel_speed_publisher;
geometry_msgs__msg__Twist cmd_vel_msg;
std_msgs__msg__Int32MultiArray encoder_msg;
sensor_msgs__msg__JointState joint_state_msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t status_timer;
rcl_timer_t encoder_timer;

// ESP32Encoder objects
ESP32Encoder encoder_FL;
ESP32Encoder encoder_FR;
ESP32Encoder encoder_RL;
ESP32Encoder encoder_RR;

// Motor control variables
struct Motor {
  int en_pin;
  int dir1_pin;
  int dir2_pin;
  int pwm_value;
  bool direction; // true = forward, false = backward
  bool reverse_flag; // Motor direction correction
};

Motor motors[4] = {
  {MOTOR_FL_EN_PIN, MOTOR_FL_DIR1_PIN, MOTOR_FL_DIR2_PIN, 0, true, FL_MOTOR_REVERSE}, // FL
  {MOTOR_FR_EN_PIN, MOTOR_FR_DIR1_PIN, MOTOR_FR_DIR2_PIN, 0, true, FR_MOTOR_REVERSE}, // FR
  {MOTOR_RL_EN_PIN, MOTOR_RL_DIR1_PIN, MOTOR_RL_DIR2_PIN, 0, true, RL_MOTOR_REVERSE}, // RL
  {MOTOR_RR_EN_PIN, MOTOR_RR_DIR1_PIN, MOTOR_RR_DIR2_PIN, 0, true, RR_MOTOR_REVERSE}  // RR
};

// Encoder correction array
bool encoder_reverse[4] = {FL_ENCODER_REVERSE, FR_ENCODER_REVERSE, RL_ENCODER_REVERSE, RR_ENCODER_REVERSE};

// === ENCODER AVERAGING VARIABLES ===
int32_t encoder_history[4][AVERAGING_SAMPLES];  // History buffer for each encoder
int history_index = 0;
bool history_filled = false;
float encoder_smoothing[4] = {0, 0, 0, 0};  // Exponential smoothing values
bool smoothing_initialized = false;

// Current velocity commands
float linear_x = 0.0;
float angular_z = 0.0;

// Function prototypes
void setupMotors();
void setupEncoders();
void setupMicroRosMessages();
void setMotorSpeed(int motor_index, int speed);
void stopAllMotors();
void calculateWheelSpeeds(float linear_x, float angular_z, float* wheel_speeds);
void cmdVelCallback(const void* msgin);
void statusLedCallback(rcl_timer_t* timer, int64_t last_call_time);
void encoderTimerCallback(rcl_timer_t* timer, int64_t last_call_time);
void publishEncoderData();
void publishJointStates();
void printEncoderValues();
int32_t getCorrectedEncoderCount(int encoder_index);

// Averaging function prototypes
void initializeAveraging();
void updateEncoderAverages();
int32_t getRawEncoderCount(int encoder_index);
int32_t getAveragedEncoderCount(int encoder_index);
int32_t getSmoothedEncoderCount(int encoder_index);
void handleSerialCommands();

// Error handling macro
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop() {
  while(1) {
    digitalWrite(STATUS_LED_PIN, HIGH);
    delay(100);
    digitalWrite(STATUS_LED_PIN, LOW);
    delay(100);
  }
}

void setup() {
  Serial.begin(115200);
  
  // Initialize hardware
  setupMotors();
  setupEncoders();
  setupMicroRosMessages();
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LOW);
  
  // Initialize micro-ROS
  set_microros_transports();
  
  delay(2000);
  
  allocator = rcl_get_default_allocator();
  
  // Create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  
  // Create node
  RCCHECK(rclc_node_init_default(&node, "esp32_robot_controller", "", &support));
  
  // Create subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"));
  
  // Create encoder publisher
  RCCHECK(rclc_publisher_init_default(
    &encoder_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
    "wheel_encoders"));
  
  // Create joint state publisher
  RCCHECK(rclc_publisher_init_default(
    &wheel_speed_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
    "joint_states"));
  
  // Create timer for status LED
  RCCHECK(rclc_timer_init_default(
    &status_timer,
    &support,
    RCL_MS_TO_NS(500),
    statusLedCallback));
  
  // Create timer for encoder publishing (200Hz for maximum accuracy)
  RCCHECK(rclc_timer_init_default(
    &encoder_timer,
    &support,
    RCL_MS_TO_NS(5), // 200Hz for higher accuracy
    encoderTimerCallback));
  
  // Create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &cmd_vel_msg, &cmdVelCallback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &status_timer));
  RCCHECK(rclc_executor_add_timer(&executor, &encoder_timer));
  
  Serial.println("ESP32 Robot Controller Ready with Encoder Averaging!");
  Serial.printf("Encoder CPR: %.0f\n", COUNTS_PER_REVOLUTION);
  Serial.printf("Averaging Samples: %d\n", AVERAGING_SAMPLES);
  Serial.printf("Smoothing Alpha: %.2f\n", SMOOTHING_ALPHA);
  Serial.println("Commands: 'cal' = calibrate, 'reset' = reset encoders, 'raw' = show raw values");
  digitalWrite(STATUS_LED_PIN, HIGH);
}

void loop() {
  delay(1); // Reduced delay for better responsiveness
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1)));
  
  // Handle serial commands for calibration and debugging
  handleSerialCommands();
}

void setupMotors() {
  // Initialize all motor pins and PWM channels
  for (int i = 0; i < 4; i++) {
    // Direction pins
    pinMode(motors[i].dir1_pin, OUTPUT);
    pinMode(motors[i].dir2_pin, OUTPUT);
    
    // Enable pins with PWM using ledcAttach
    ledcAttach(motors[i].en_pin, PWM_FREQ, PWM_RESOLUTION);
    
    // Set initial motor state
    digitalWrite(motors[i].dir1_pin, LOW);
    digitalWrite(motors[i].dir2_pin, LOW);
    ledcWrite(motors[i].en_pin, 0);
  }
  
  Serial.println("Motors initialized");
}

void setupEncoders() {
  // Enable the weak pull up resistors for encoder pins
  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  
  // Attach encoders to pins using full quadrature
  encoder_FL.attachFullQuad(MOTOR_FL_ENC_A_PIN, MOTOR_FL_ENC_B_PIN);
  encoder_FR.attachFullQuad(MOTOR_FR_ENC_A_PIN, MOTOR_FR_ENC_B_PIN);
  encoder_RL.attachFullQuad(MOTOR_RL_ENC_A_PIN, MOTOR_RL_ENC_B_PIN);
  encoder_RR.attachFullQuad(MOTOR_RR_ENC_A_PIN, MOTOR_RR_ENC_B_PIN);
  
  // Clear encoder counts
  encoder_FL.clearCount();
  encoder_FR.clearCount();
  encoder_RL.clearCount();
  encoder_RR.clearCount();
  
  // Set encoder counts to zero
  encoder_FL.setCount(0);
  encoder_FR.setCount(0);
  encoder_RL.setCount(0);
  encoder_RR.setCount(0);
  
  // Initialize averaging
  initializeAveraging();
  
  Serial.println("Encoders initialized with averaging system");
}

void setupMicroRosMessages() {
  // Initialize encoder message
  encoder_msg.data.capacity = 4;
  encoder_msg.data.size = 4;
  encoder_msg.data.data = (int32_t*)malloc(4 * sizeof(int32_t));
  encoder_msg.layout.dim.capacity = 1;
  encoder_msg.layout.dim.size = 1;
  encoder_msg.layout.dim.data = (std_msgs__msg__MultiArrayDimension*)malloc(1 * sizeof(std_msgs__msg__MultiArrayDimension));
  encoder_msg.layout.dim.data[0].label.capacity = 20;
  encoder_msg.layout.dim.data[0].label.size = 8;
  encoder_msg.layout.dim.data[0].label.data = (char*)malloc(20 * sizeof(char));
  strcpy(encoder_msg.layout.dim.data[0].label.data, "encoders");
  encoder_msg.layout.dim.data[0].size = 4;
  encoder_msg.layout.dim.data[0].stride = 4;
  encoder_msg.layout.data_offset = 0;
  
  // Initialize joint state message
  joint_state_msg.name.capacity = 4;
  joint_state_msg.name.size = 4;
  joint_state_msg.name.data = (rosidl_runtime_c__String*)malloc(4 * sizeof(rosidl_runtime_c__String));
  
  joint_state_msg.position.capacity = 4;
  joint_state_msg.position.size = 4;
  joint_state_msg.position.data = (double*)malloc(4 * sizeof(double));
  
  joint_state_msg.velocity.capacity = 4;
  joint_state_msg.velocity.size = 4;
  joint_state_msg.velocity.data = (double*)malloc(4 * sizeof(double));
  
  joint_state_msg.effort.capacity = 4;
  joint_state_msg.effort.size = 4;
  joint_state_msg.effort.data = (double*)malloc(4 * sizeof(double));
  
  // Initialize joint names
  const char* joint_names[4] = {"front_left_wheel", "front_right_wheel", "rear_left_wheel", "rear_right_wheel"};
  for (int i = 0; i < 4; i++) {
    joint_state_msg.name.data[i].capacity = 20;
    joint_state_msg.name.data[i].size = strlen(joint_names[i]);
    joint_state_msg.name.data[i].data = (char*)malloc(20 * sizeof(char));
    strcpy(joint_state_msg.name.data[i].data, joint_names[i]);
  }
  
  Serial.println("Messages initialized");
}

// === ENCODER AVERAGING FUNCTIONS ===

void initializeAveraging() {
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < AVERAGING_SAMPLES; j++) {
      encoder_history[i][j] = 0;
    }
    encoder_smoothing[i] = 0;
  }
  history_index = 0;
  history_filled = false;
  smoothing_initialized = false;
}

int32_t getRawEncoderCount(int encoder_index) {
  int32_t count = 0;
  
  switch(encoder_index) {
    case 0: count = encoder_FL.getCount(); break;
    case 1: count = encoder_FR.getCount(); break;
    case 2: count = encoder_RL.getCount(); break;
    case 3: count = encoder_RR.getCount(); break;
  }
  
  // Apply encoder direction correction
  if (encoder_reverse[encoder_index]) {
    count = -count;
  }
  
  return count;
}

void updateEncoderAverages() {
  // Store current readings in history buffer
  for (int i = 0; i < 4; i++) {
    encoder_history[i][history_index] = getRawEncoderCount(i);
  }
  
  // Update index
  history_index = (history_index + 1) % AVERAGING_SAMPLES;
  
  // Mark as filled once we've gone through all samples
  if (history_index == 0 && !history_filled) {
    history_filled = true;
  }
}

int32_t getAveragedEncoderCount(int encoder_index) {
  if (!history_filled) {
    // If history not filled yet, return current value
    return getRawEncoderCount(encoder_index);
  }
  
  // Calculate average
  int64_t sum = 0;
  for (int i = 0; i < AVERAGING_SAMPLES; i++) {
    sum += encoder_history[encoder_index][i];
  }
  
  return (int32_t)(sum / AVERAGING_SAMPLES);
}

int32_t getSmoothedEncoderCount(int encoder_index) {
  int32_t raw_count = getRawEncoderCount(encoder_index);
  
  if (!smoothing_initialized) {
    encoder_smoothing[encoder_index] = raw_count;
    smoothing_initialized = true;
  } else {
    encoder_smoothing[encoder_index] = SMOOTHING_ALPHA * raw_count + (1.0 - SMOOTHING_ALPHA) * encoder_smoothing[encoder_index];
  }
  
  return (int32_t)encoder_smoothing[encoder_index];
}

// Main function - uses smoothed values for best balance of stability and responsiveness
int32_t getCorrectedEncoderCount(int encoder_index) {
  return getSmoothedEncoderCount(encoder_index);
}

void setMotorSpeed(int motor_index, int speed) {
  if (motor_index < 0 || motor_index > 3) return;
  
  // Constrain speed to valid range
  speed = constrain(speed, -MAX_PWM_VALUE, MAX_PWM_VALUE);
  
  // Apply motor direction correction
  if (motors[motor_index].reverse_flag) {
    speed = -speed;
  }
  
  if (speed == 0) {
    // Stop motor
    digitalWrite(motors[motor_index].dir1_pin, LOW);
    digitalWrite(motors[motor_index].dir2_pin, LOW);
    ledcWrite(motors[motor_index].en_pin, 0);
  } else if (speed > 0) {
    // Forward direction
    digitalWrite(motors[motor_index].dir1_pin, HIGH);
    digitalWrite(motors[motor_index].dir2_pin, LOW);
    ledcWrite(motors[motor_index].en_pin, speed);
    motors[motor_index].direction = true;
  } else {
    // Backward direction
    digitalWrite(motors[motor_index].dir1_pin, LOW);
    digitalWrite(motors[motor_index].dir2_pin, HIGH);
    ledcWrite(motors[motor_index].en_pin, -speed);
    motors[motor_index].direction = false;
  }
  
  motors[motor_index].pwm_value = speed;
}

void stopAllMotors() {
  for (int i = 0; i < 4; i++) {
    setMotorSpeed(i, 0);
  }
}

void calculateWheelSpeeds(float linear_x, float angular_z, float* wheel_speeds) {
  // Differential drive kinematics for 4WD robot
  float v_left = linear_x - angular_z * TRACK_WIDTH / 2.0;
  float v_right = linear_x + angular_z * TRACK_WIDTH / 2.0;
  
  // Convert from m/s to PWM values (simple proportional control)
  float max_speed = 1.0; // m/s
  
  wheel_speeds[0] = (v_left / max_speed) * MAX_PWM_VALUE;  // FL
  wheel_speeds[1] = (v_right / max_speed) * MAX_PWM_VALUE; // FR
  wheel_speeds[2] = (v_left / max_speed) * MAX_PWM_VALUE;  // RL
  wheel_speeds[3] = (v_right / max_speed) * MAX_PWM_VALUE; // RR
}

void cmdVelCallback(const void* msgin) {
  const geometry_msgs__msg__Twist* msg = (const geometry_msgs__msg__Twist*)msgin;
  
  // Extract velocity commands
  linear_x = msg->linear.x;
  angular_z = msg->angular.z;
  
  // Calculate individual wheel speeds
  float wheel_speeds[4];
  calculateWheelSpeeds(linear_x, angular_z, wheel_speeds);
  
  // Apply speeds to motors
  for (int i = 0; i < 4; i++) {
    setMotorSpeed(i, (int)wheel_speeds[i]);
  }
  
  // Debug output
  Serial.printf("Cmd: linear_x=%.2f, angular_z=%.2f\n", linear_x, angular_z);
  Serial.printf("Speeds: FL=%d, FR=%d, RL=%d, RR=%d\n", 
                (int)wheel_speeds[0], (int)wheel_speeds[1], 
                (int)wheel_speeds[2], (int)wheel_speeds[3]);
}

void encoderTimerCallback(rcl_timer_t* timer, int64_t last_call_time) {
  (void) last_call_time;
  (void) timer;
  
  // Update averaging buffers
  updateEncoderAverages();
  
  // Publish encoder data
  publishEncoderData();
  publishJointStates();
}

void publishEncoderData() {
  // Get current encoder counts with correction and averaging
  encoder_msg.data.data[0] = getCorrectedEncoderCount(0); // FL
  encoder_msg.data.data[1] = getCorrectedEncoderCount(1); // FR
  encoder_msg.data.data[2] = getCorrectedEncoderCount(2); // RL
  encoder_msg.data.data[3] = getCorrectedEncoderCount(3); // RR
  
  // Publish encoder data
  RCSOFTCHECK(rcl_publish(&encoder_publisher, &encoder_msg, NULL));
}

void publishJointStates() {
  static int64_t last_time = 0;
  static int32_t last_counts[4] = {0, 0, 0, 0};
  static bool first_run = true;
  
  int64_t current_time = rmw_uros_epoch_millis();
  int32_t current_counts[4] = {
    getCorrectedEncoderCount(0),
    getCorrectedEncoderCount(1),
    getCorrectedEncoderCount(2),
    getCorrectedEncoderCount(3)
  };
  
  // Calculate time difference
  double dt = (current_time - last_time) / 1000.0; // Convert to seconds
  
  if (!first_run && dt > 0.001) { // Skip first run and ensure minimum dt
    // More accurate calculation with higher precision
    const double counts_to_radians = (2.0 * M_PI) / COUNTS_PER_REVOLUTION;
    
    for (int i = 0; i < 4; i++) {
      // Position in radians - Higher precision calculation
      joint_state_msg.position.data[i] = current_counts[i] * counts_to_radians;
      
      // Velocity in rad/s - Smoothed calculation with bounds checking
      int32_t count_diff = current_counts[i] - last_counts[i];
      double raw_velocity = count_diff * counts_to_radians / dt;
      
      // Apply velocity smoothing filter (simple low-pass)
      static double prev_velocity[4] = {0, 0, 0, 0};
      double alpha = 0.7; // Smoothing factor (0-1, higher = less smoothing)
      joint_state_msg.velocity.data[i] = alpha * raw_velocity + (1.0 - alpha) * prev_velocity[i];
      prev_velocity[i] = joint_state_msg.velocity.data[i];
      
      // Effort (PWM value normalized to -1.0 to 1.0)
      joint_state_msg.effort.data[i] = motors[i].pwm_value / 255.0;
      
      last_counts[i] = current_counts[i];
    }
    
    // Set high-precision timestamp
    joint_state_msg.header.stamp.sec = current_time / 1000;
    joint_state_msg.header.stamp.nanosec = (current_time % 1000) * 1000000UL;
    
    // Publish joint states
    RCSOFTCHECK(rcl_publish(&wheel_speed_publisher, &joint_state_msg, NULL));
  }
  
  if (first_run) {
    first_run = false;
    // Initialize last_counts on first run
    for (int i = 0; i < 4; i++) {
      last_counts[i] = current_counts[i];
    }
  }
  
  last_time = current_time;
}

void statusLedCallback(rcl_timer_t* timer, int64_t last_call_time) {
  (void) last_call_time;
  (void) timer;
  
  static bool led_state = false;
  led_state = !led_state;
  digitalWrite(STATUS_LED_PIN, led_state);
  
  // Print encoder values periodically for debugging
  printEncoderValues();
}

void printEncoderValues() {
  static unsigned long last_print = 0;
  static bool show_raw = false;
  unsigned long now = millis();
  
  if (now - last_print > 1000) { // Print every 1 second
    if (show_raw) {
      Serial.printf("Raw Encoders - FL:%ld, FR:%ld, RL:%ld, RR:%ld\n",
                    getRawEncoderCount(0), getRawEncoderCount(1),
                    getRawEncoderCount(2), getRawEncoderCount(3));
      
      Serial.printf("Averaged Encoders - FL:%ld, FR:%ld, RL:%ld, RR:%ld\n",
                    getAveragedEncoderCount(0), getAveragedEncoderCount(1),
                    getAveragedEncoderCount(2), getAveragedEncoderCount(3));
    }
    
    Serial.printf("Smoothed Encoders - FL:%ld, FR:%ld, RL:%ld, RR:%ld\n",
                  getCorrectedEncoderCount(0), getCorrectedEncoderCount(1),
                  getCorrectedEncoderCount(2), getCorrectedEncoderCount(3));
    
    // Show positions in degrees with averaging status
    const double counts_to_radians = (2.0 * M_PI) / COUNTS_PER_REVOLUTION;
    Serial.printf("Positions (deg) [SMOOTHED] - FL:%.2f, FR:%.2f, RL:%.2f, RR:%.2f\n",
                  getCorrectedEncoderCount(0) * counts_to_radians * 180.0 / M_PI,
                  getCorrectedEncoderCount(1) * counts_to_radians * 180.0 / M_PI,
                  getCorrectedEncoderCount(2) * counts_to_radians * 180.0 / M_PI,
                  getCorrectedEncoderCount(3) * counts_to_radians * 180.0 / M_PI);
    
    // Show variance (how much they differ)
    float positions[4];
    float avg_position = 0;
    for (int i = 0; i < 4; i++) {
      positions[i] = getCorrectedEncoderCount(i) * counts_to_radians * 180.0 / M_PI;
      avg_position += positions[i];
    }
    avg_position /= 4.0;
    
    float max_diff = 0;
    for (int i = 0; i < 4; i++) {
      float diff = abs(positions[i] - avg_position);
      if (diff > max_diff) max_diff = diff;
    }
    Serial.printf("Average: %.2f°, Max Diff: %.2f°, Status: %s\n", 
                  avg_position, max_diff,
                  history_filled ? "AVERAGED" : "INITIALIZING");
    
    last_print = now;
  }
}

void handleSerialCommands() {
  if (Serial.available()) {
    String command = Serial.readString();
    command.trim();
    command.toLowerCase();
    
    if (command == "cal" || command == "calibrate") {
      Serial.println("Calibrating encoders...");
      // Reset smoothing
      smoothing_initialized = false;
      initializeAveraging();
      
      // Clear encoders
      encoder_FL.setCount(0);
      encoder_FR.setCount(0);
      encoder_RL.setCount(0);
      encoder_RR.setCount(0);
      
      Serial.println("Encoder calibration complete!");
      
    } else if (command == "reset") {
      // Reset all encoders to zero
      encoder_FL.setCount(0);
      encoder_FR.setCount(0);
      encoder_RL.setCount(0);
      encoder_RR.setCount(0);
      
      // Reset averaging
      initializeAveraging();
      
      Serial.println("Encoders reset to zero");
      
    } else if (command == "raw") {
      Serial.println("Showing raw encoder values in next cycle...");
      // This will be handled in printEncoderValues
      
    } else if (command == "help") {
      Serial.println("Available commands:");
      Serial.println("  cal/calibrate - Reset encoders to zero");
      Serial.println("  reset - Reset encoders and averaging");
      Serial.println("  raw - Show raw encoder values");
      Serial.println("  help - Show this help");
    }
  }
}

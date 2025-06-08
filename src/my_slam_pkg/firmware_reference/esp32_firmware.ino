#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <sensor_msgs/msg/joint_state.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/string.h>

// ESP32 specific includes
#include <WiFi.h>

// === TRANSPORT CONFIGURATION ===
#define USE_SERIAL_TRANSPORT    // Use USB Serial connection
// #define USE_WIFI_TRANSPORT   // Use WiFi connection

#ifdef USE_WIFI_TRANSPORT
const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";
const char* agent_ip = "192.168.1.100";
const int agent_port = 8888;
#endif

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

// === ROBOT PARAMETERS ===
#define JOINT_COUNT 4
#define WHEEL_RADIUS 0.05           // meters
#define WHEEL_BASE_WIDTH 0.34       // meters
#define ENCODER_PPR 11              // Physical pulses per revolution
#define GEAR_RATIO 1.0              // Gearbox reduction ratio

// PWM Configuration
#define PWM_FREQUENCY 20000         // 20 kHz
#define PWM_RESOLUTION 8            // 8-bit resolution
#define PWM_MAX_DUTY 255

// Control parameters
#define CONTROL_FREQUENCY_HZ 50     // Control loop frequency
#define JOINT_STATE_FREQUENCY_HZ 20 // Joint state publish frequency
#define DIAGNOSTIC_FREQUENCY_HZ 1   // Diagnostic publish frequency
#define WATCHDOG_TIMEOUT_MS 1000    // Command timeout

// PID Constants
#define KP 2.0f
#define KI 0.5f  
#define KD 0.1f
#define MAX_PID_OUTPUT 255.0f
#define MIN_PID_OUTPUT -255.0f

// === GLOBAL VARIABLES ===
// Encoder data (volatile for ISR safety)
volatile long encoder_counts[JOINT_COUNT] = {0, 0, 0, 0};
long encoder_prev[JOINT_COUNT] = {0, 0, 0, 0};

// Joint state variables
double joint_positions[JOINT_COUNT] = {0.0, 0.0, 0.0, 0.0};
double joint_velocities[JOINT_COUNT] = {0.0, 0.0, 0.0, 0.0};
double joint_efforts[JOINT_COUNT] = {0.0, 0.0, 0.0, 0.0};

// Control variables
double velocity_commands[JOINT_COUNT] = {0.0, 0.0, 0.0, 0.0};

// PID control variables
float pid_errors[JOINT_COUNT] = {0.0, 0.0, 0.0, 0.0};
float pid_prev_errors[JOINT_COUNT] = {0.0, 0.0, 0.0, 0.0};
float pid_error_sums[JOINT_COUNT] = {0.0, 0.0, 0.0, 0.0};

// Timing variables
unsigned long last_control_time = 0;
unsigned long last_joint_state_time = 0;
unsigned long last_cmd_time = 0;
unsigned long last_diagnostic_time = 0;

// Connection management
bool ros_connected = false;
int connection_attempts = 0;

// micro-ROS entities
rcl_subscription_t twist_subscriber;
rcl_publisher_t joint_state_publisher;
rcl_publisher_t diagnostic_publisher;

// Messages
geometry_msgs__msg__Twist twist_msg;
sensor_msgs__msg__JointState joint_state_msg;
std_msgs__msg__String diagnostic_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// Joint names for ros2_control compatibility
const char* joint_names[JOINT_COUNT] = {
  "front_left_wheel_joint",
  "front_right_wheel_joint", 
  "rear_left_wheel_joint",
  "rear_right_wheel_joint"
};

char diagnostic_buffer[200];
bool joint_state_memory_initialized = false;

// === FUNCTION PROTOTYPES ===
void IRAM_ATTR encoderISR_FL();
void IRAM_ATTR encoderISR_FR(); 
void IRAM_ATTR encoderISR_RL();
void IRAM_ATTR encoderISR_RR();

void setupHardware();
void setupMotors();
void setupEncoders();
void setupMicroROS();
bool initMicroROS();
void cleanupMicroROS();

void controlLoop();
void updateJointStates(float dt);
float calculatePID(int joint_id, float setpoint, float process_value, float dt);
void setMotorSpeed(int motor_id, float speed_normalized);
void stopAllMotors();

void twistCallback(const void* msgin);
void publishJointStates();
void publishDiagnostics();

void blinkLED(int times, int duration_ms);

//*****************************************************************************
// ENCODER ISRs
//*****************************************************************************
void IRAM_ATTR encoderISR_FL() {
  if (digitalRead(MOTOR_FL_ENC_A_PIN) == digitalRead(MOTOR_FL_ENC_B_PIN)) {
    encoder_counts[0]--;
  } else {
    encoder_counts[0]++;
  }
}

void IRAM_ATTR encoderISR_FR() {
  if (digitalRead(MOTOR_FR_ENC_A_PIN) == digitalRead(MOTOR_FR_ENC_B_PIN)) {
    encoder_counts[1]++;
  } else {
    encoder_counts[1]--;
  }
}

void IRAM_ATTR encoderISR_RL() {
  if (digitalRead(MOTOR_RL_ENC_A_PIN) == digitalRead(MOTOR_RL_ENC_B_PIN)) {
    encoder_counts[2]--;
  } else {
    encoder_counts[2]++;
  }
}

void IRAM_ATTR encoderISR_RR() {
  if (digitalRead(MOTOR_RR_ENC_A_PIN) == digitalRead(MOTOR_RR_ENC_B_PIN)) {
    encoder_counts[3]++;
  } else {
    encoder_counts[3]--;
  }
}

//*****************************************************************************
// SETUP FUNCTION
//*****************************************************************************
void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("=== ESP32 4-Motor Test ===");
  Serial.println("Initializing hardware...");

  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LOW);

  setupHardware(); // This initializes pins and PWM
  // setupMicroROS(); // Keep this commented for now

  blinkLED(3, 200);
  Serial.println("Hardware setup complete. Testing motors...");

  // --- TEMPORARY MOTOR TEST CODE ---
  // Test Front Left motor
  Serial.println("Testing Motor FL (0) - Forward");
  setMotorSpeed(0, 0.5); // Half speed forward
  delay(3000);
  Serial.println("Testing Motor FL (0) - Backward");
  setMotorSpeed(0, -0.5); // Half speed backward
  delay(3000);
  Serial.println("Stopping Motor FL (0)");
  setMotorSpeed(0, 0.0);
  delay(1000);

  // Repeat for other motors (uncomment one by one or all)
  Serial.println("Testing Motor FR (1) - Forward");
  setMotorSpeed(1, 0.5);
  delay(3000);
  setMotorSpeed(1, 0.0);
  delay(1000);

  // Test Rear Left motor
  Serial.println("Testing Motor RL (2) - Forward");
  setMotorSpeed(2, 0.5);
  delay(3000);
  setMotorSpeed(2, 0.0);
  delay(1000);

  // Test Rear Right motor
  Serial.println("Testing Motor RR (3) - Forward");
  setMotorSpeed(3, 0.5);
  delay(3000);
  setMotorSpeed(3, 0.0);
  delay(1000);
  // --- END TEMPORARY MOTOR TEST CODE ---

  Serial.println("Motor test complete. Now proceed with ROS setup (if enabled).");
  // last_control_time = millis();
  // last_joint_state_time = millis();
  // last_cmd_time = millis();
  // last_diagnostic_time = millis();
}

//*****************************************************************************
// MAIN LOOP
//*****************************************************************************
void loop() {
  // Keep empty or just a small delay for this test phase
  delay(10);
  
  // Comment out all the ROS logic for now
  /*
  unsigned long current_time = millis();
  
  // Connection management
  if (!ros_connected) {
    if (current_time - last_diagnostic_time > 3000) {
      last_diagnostic_time = current_time;
      Serial.printf("Attempting ROS connection (attempt %d)...\n", ++connection_attempts);
      
      if (connection_attempts > 1) {
        cleanupMicroROS();
      }
      
      ros_connected = initMicroROS();
      if (ros_connected) {
        Serial.println("ros2_control interface connected!");
        digitalWrite(STATUS_LED_PIN, HIGH);
        connection_attempts = 0;
      } else {
        Serial.println("Connection failed, retrying...");
        blinkLED(2, 100);
      }
    }
  }

  if (ros_connected) {
    // Process ROS callbacks
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
    
    // Safety watchdog
    if (current_time - last_cmd_time > WATCHDOG_TIMEOUT_MS) {
      stopAllMotors();
    }
    
    // Publish joint states
    if (current_time - last_joint_state_time >= (1000 / JOINT_STATE_FREQUENCY_HZ)) {
      publishJointStates();
      last_joint_state_time = current_time;
    }
    
    // Publish diagnostics
    if (current_time - last_diagnostic_time >= (1000 / DIAGNOSTIC_FREQUENCY_HZ)) {
      publishDiagnostics();
      last_diagnostic_time = current_time;
    }
  } else {
    stopAllMotors();
  }

  // Always run control loop
  if (current_time - last_control_time >= (1000 / CONTROL_FREQUENCY_HZ)) {
    controlLoop();
    last_control_time = current_time;
  }
  */
}

//*****************************************************************************
// HARDWARE SETUP
//*****************************************************************************
void setupHardware() {
  setupMotors();
  setupEncoders();
  Serial.println("Hardware initialization complete.");
}

void setupMotors() {
  // Configure direction pins
  pinMode(MOTOR_FL_DIR1_PIN, OUTPUT); pinMode(MOTOR_FL_DIR2_PIN, OUTPUT);
  pinMode(MOTOR_FR_DIR1_PIN, OUTPUT); pinMode(MOTOR_FR_DIR2_PIN, OUTPUT);
  pinMode(MOTOR_RL_DIR1_PIN, OUTPUT); pinMode(MOTOR_RL_DIR2_PIN, OUTPUT);
  pinMode(MOTOR_RR_DIR1_PIN, OUTPUT); pinMode(MOTOR_RR_DIR2_PIN, OUTPUT);

  // Setup PWM - try new API first, fallback to old API
  #if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
    // New ESP32 Arduino Core (3.x)
    ledcAttach(MOTOR_FL_EN_PIN, PWM_FREQUENCY, PWM_RESOLUTION);
    ledcAttach(MOTOR_FR_EN_PIN, PWM_FREQUENCY, PWM_RESOLUTION);
    ledcAttach(MOTOR_RL_EN_PIN, PWM_FREQUENCY, PWM_RESOLUTION);
    ledcAttach(MOTOR_RR_EN_PIN, PWM_FREQUENCY, PWM_RESOLUTION);
  #else
    // Older ESP32 Arduino Core (2.x and earlier)
    ledcSetup(0, PWM_FREQUENCY, PWM_RESOLUTION);
    ledcSetup(1, PWM_FREQUENCY, PWM_RESOLUTION);
    ledcSetup(2, PWM_FREQUENCY, PWM_RESOLUTION);
    ledcSetup(3, PWM_FREQUENCY, PWM_RESOLUTION);
    
    ledcAttachPin(MOTOR_FL_EN_PIN, 0);
    ledcAttachPin(MOTOR_FR_EN_PIN, 1);
    ledcAttachPin(MOTOR_RL_EN_PIN, 2);
    ledcAttachPin(MOTOR_RR_EN_PIN, 3);
  #endif

  stopAllMotors();
  Serial.printf("Motors initialized with %d Hz PWM\n", PWM_FREQUENCY);
}

void setupEncoders() {
  // Configure encoder pins with pullups
  pinMode(MOTOR_FL_ENC_A_PIN, INPUT_PULLUP); pinMode(MOTOR_FL_ENC_B_PIN, INPUT_PULLUP);
  pinMode(MOTOR_FR_ENC_A_PIN, INPUT_PULLUP); pinMode(MOTOR_FR_ENC_B_PIN, INPUT_PULLUP);
  pinMode(MOTOR_RL_ENC_A_PIN, INPUT_PULLUP); pinMode(MOTOR_RL_ENC_B_PIN, INPUT_PULLUP);
  pinMode(MOTOR_RR_ENC_A_PIN, INPUT_PULLUP); pinMode(MOTOR_RR_ENC_B_PIN, INPUT_PULLUP);

  // Attach interrupts
  attachInterrupt(digitalPinToInterrupt(MOTOR_FL_ENC_A_PIN), encoderISR_FL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTOR_FR_ENC_A_PIN), encoderISR_FR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTOR_RL_ENC_A_PIN), encoderISR_RL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTOR_RR_ENC_A_PIN), encoderISR_RR, CHANGE);

  Serial.printf("Encoders initialized (%d PPR, %.1fx gear ratio)\n", ENCODER_PPR, GEAR_RATIO);
}

//*****************************************************************************
// MICRO-ROS SETUP
//*****************************************************************************
void setupMicroROS() {
#ifdef USE_SERIAL_TRANSPORT
  set_microros_transports();
  Serial.println("micro-ROS serial transport configured");
#endif

#ifdef USE_WIFI_TRANSPORT
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.printf("WiFi connected! IP: %s\n", WiFi.localIP().toString().c_str());
  
  set_microros_wifi_transports(ssid, password, agent_ip, agent_port);
  Serial.println("micro-ROS WiFi transport configured");
#endif
}

bool initMicroROS() {
  allocator = rcl_get_default_allocator();

  // Initialize support
  if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) {
    Serial.println("Failed to initialize RCL support");
    return false;
  }

  // Initialize node
  if (rclc_node_init_default(&node, "esp32_hardware_interface", "", &support) != RCL_RET_OK) {
    Serial.println("Failed to initialize RCL node");
    return false;
  }

  // Initialize joint state message (only once)
  if (!joint_state_memory_initialized) {
    // Header
    joint_state_msg.header.frame_id.data = (char*)"base_link";
    joint_state_msg.header.frame_id.size = strlen("base_link");
    joint_state_msg.header.frame_id.capacity = strlen("base_link") + 1;

    // Allocate joint names
    joint_state_msg.name.capacity = JOINT_COUNT;
    joint_state_msg.name.size = JOINT_COUNT;
    joint_state_msg.name.data = (rosidl_runtime_c__String*)malloc(JOINT_COUNT * sizeof(rosidl_runtime_c__String));
    
    if (!joint_state_msg.name.data) {
      Serial.println("Failed to allocate joint names memory");
      return false;
    }

    for (int i = 0; i < JOINT_COUNT; i++) {
      size_t len = strlen(joint_names[i]);
      joint_state_msg.name.data[i].data = (char*)malloc((len + 1) * sizeof(char));
      if (!joint_state_msg.name.data[i].data) {
        Serial.printf("Failed to allocate memory for joint name %d\n", i);
        return false;
      }
      joint_state_msg.name.data[i].size = len;
      joint_state_msg.name.data[i].capacity = len + 1;
      strcpy(joint_state_msg.name.data[i].data, joint_names[i]);
    }

    // Assign data arrays
    joint_state_msg.position.data = joint_positions;
    joint_state_msg.position.size = JOINT_COUNT;
    joint_state_msg.position.capacity = JOINT_COUNT;
    
    joint_state_msg.velocity.data = joint_velocities;
    joint_state_msg.velocity.size = JOINT_COUNT;
    joint_state_msg.velocity.capacity = JOINT_COUNT;
    
    joint_state_msg.effort.data = joint_efforts;
    joint_state_msg.effort.size = JOINT_COUNT;
    joint_state_msg.effort.capacity = JOINT_COUNT;

    joint_state_memory_initialized = true;
  }

  // Initialize diagnostic message
  diagnostic_msg.data.capacity = sizeof(diagnostic_buffer);
  diagnostic_msg.data.size = 0;
  diagnostic_msg.data.data = diagnostic_buffer;

  // Create subscription for cmd_vel
  if (rclc_subscription_init_default(&twist_subscriber, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel") != RCL_RET_OK) {
    Serial.println("Failed to create twist subscriber");
    return false;
  }

  // Create publishers
  if (rclc_publisher_init_default(&joint_state_publisher, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState), "joint_states") != RCL_RET_OK) {
    Serial.println("Failed to create joint state publisher");
    return false;
  }

  if (rclc_publisher_init_default(&diagnostic_publisher, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "esp32/diagnostics") != RCL_RET_OK) {
    Serial.println("Failed to create diagnostic publisher");
    return false;
  }

  // Create executor
  if (rclc_executor_init(&executor, &support.context, 1, &allocator) != RCL_RET_OK) {
    Serial.println("Failed to create executor");
    return false;
  }

  // Add subscription to executor
  rclc_executor_add_subscription(&executor, &twist_subscriber, &twist_msg, &twistCallback, ON_NEW_DATA);

  Serial.println("ros2_control interface initialized successfully");
  return true;
}

void cleanupMicroROS() {
  if (ros_connected) {
    // Note: Cleanup order is important to avoid crashes
    rclc_executor_fini(&executor);
    rcl_publisher_fini(&diagnostic_publisher, &node);
    rcl_publisher_fini(&joint_state_publisher, &node);
    rcl_subscription_fini(&twist_subscriber, &node);
    rcl_node_fini(&node);
    rclc_support_fini(&support);
    ros_connected = false;
  }
}

//*****************************************************************************
// CONTROL FUNCTIONS
//*****************************************************************************
void controlLoop() {
  unsigned long now = millis();
  float dt = (now - last_control_time) / 1000.0f;
  
  if (dt <= 0 || dt > 0.1f) {
    dt = 1.0f / CONTROL_FREQUENCY_HZ;
  }

  updateJointStates(dt);

  // PID velocity control for each joint
  for (int i = 0; i < JOINT_COUNT; i++) {
    float pid_output = calculatePID(i, velocity_commands[i], joint_velocities[i], dt);
    float motor_speed = constrain(pid_output / MAX_PID_OUTPUT, -1.0f, 1.0f);
    setMotorSpeed(i, motor_speed);
    
    // Estimate effort from PID output
    joint_efforts[i] = pid_output / MAX_PID_OUTPUT;
  }
}

void updateJointStates(float dt) {
  for (int i = 0; i < JOINT_COUNT; i++) {
    long current_counts;
    
    // Atomic read of encoder counts
    noInterrupts();
    current_counts = encoder_counts[i];
    interrupts();
    
    long count_diff = current_counts - encoder_prev[i];
    encoder_prev[i] = current_counts;
    
    // Convert encoder counts to radians
    double position_increment = (double)count_diff * 2.0 * PI / (ENCODER_PPR * GEAR_RATIO);
    joint_positions[i] += position_increment;
    
    // Calculate velocity in rad/s
    if (dt > 0) {
      joint_velocities[i] = position_increment / dt;
    }
  }
}

float calculatePID(int joint_id, float setpoint, float process_value, float dt) {
  if (dt <= 0) return 0.0f;
  
  float& error = pid_errors[joint_id];
  float& prev_error = pid_prev_errors[joint_id];
  float& error_sum = pid_error_sums[joint_id];
  
  error = setpoint - process_value;
  error_sum += error * dt;
  
  // Anti-windup
  error_sum = constrain(error_sum, -100.0f, 100.0f);
  
  float derivative = (error - prev_error) / dt;
  prev_error = error;
  
  float output = (KP * error) + (KI * error_sum) + (KD * derivative);
  return constrain(output, MIN_PID_OUTPUT, MAX_PID_OUTPUT);
}

void setMotorSpeed(int motor_id, float speed_normalized) {
  speed_normalized = constrain(speed_normalized, -1.0f, 1.0f);
  
  int pwm_value = (int)(fabs(speed_normalized) * PWM_MAX_DUTY);
  bool forward = (speed_normalized >= 0.0f);
  
  int dir1_pin, dir2_pin, pwm_pin;
  
  #if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
    // For new ESP32 core, pwm_pin is the actual pin number
    switch (motor_id) {
      case 0: dir1_pin = MOTOR_FL_DIR1_PIN; dir2_pin = MOTOR_FL_DIR2_PIN; pwm_pin = MOTOR_FL_EN_PIN; break;
      case 1: dir1_pin = MOTOR_FR_DIR1_PIN; dir2_pin = MOTOR_FR_DIR2_PIN; pwm_pin = MOTOR_FR_EN_PIN; break;
      case 2: dir1_pin = MOTOR_RL_DIR1_PIN; dir2_pin = MOTOR_RL_DIR2_PIN; pwm_pin = MOTOR_RL_EN_PIN; break;
      case 3: dir1_pin = MOTOR_RR_DIR1_PIN; dir2_pin = MOTOR_RR_DIR2_PIN; pwm_pin = MOTOR_RR_EN_PIN; break;
      default: return;
    }
  #else
    // For old ESP32 core, pwm_pin is the channel number
    switch (motor_id) {
      case 0: dir1_pin = MOTOR_FL_DIR1_PIN; dir2_pin = MOTOR_FL_DIR2_PIN; pwm_pin = 0; break;
      case 1: dir1_pin = MOTOR_FR_DIR1_PIN; dir2_pin = MOTOR_FR_DIR2_PIN; pwm_pin = 1; break;
      case 2: dir1_pin = MOTOR_RL_DIR1_PIN; dir2_pin = MOTOR_RL_DIR2_PIN; pwm_pin = 2; break;
      case 3: dir1_pin = MOTOR_RR_DIR1_PIN; dir2_pin = MOTOR_RR_DIR2_PIN; pwm_pin = 3; break;
      default: return;
    }
  #endif
  
  // Set direction
  digitalWrite(dir1_pin, forward ? HIGH : LOW);
  digitalWrite(dir2_pin, forward ? LOW : HIGH);
  
  // Set PWM speed
  ledcWrite(pwm_pin, pwm_value);
}

void stopAllMotors() {
  for (int i = 0; i < JOINT_COUNT; i++) {
    setMotorSpeed(i, 0.0f);
    velocity_commands[i] = 0.0;
    pid_error_sums[i] = 0.0; // Reset integral terms
  }
}

//*****************************************************************************
// ROS CALLBACKS AND PUBLISHERS
//*****************************************************************************
void twistCallback(const void* msgin) {
  const geometry_msgs__msg__Twist* msg = (const geometry_msgs__msg__Twist*)msgin;
  
  float linear_x = msg->linear.x;
  float angular_z = msg->angular.z;
  
  // Convert twist to wheel velocities (differential drive kinematics)
  float left_vel = (linear_x - angular_z * WHEEL_BASE_WIDTH / 2.0) / WHEEL_RADIUS;
  float right_vel = (linear_x + angular_z * WHEEL_BASE_WIDTH / 2.0) / WHEEL_RADIUS;
  
  // Assign to wheels
  velocity_commands[0] = left_vel;   // Front left
  velocity_commands[1] = right_vel;  // Front right
  velocity_commands[2] = left_vel;   // Rear left
  velocity_commands[3] = right_vel;  // Rear right
  
  last_cmd_time = millis();
}

void publishJointStates() {
  if (!ros_connected) return;
  
  // Update timestamp
  int64_t time_ns = rmw_uros_epoch_nanos();
  joint_state_msg.header.stamp.sec = time_ns / 1000000000LL;
  joint_state_msg.header.stamp.nanosec = time_ns % 1000000000LL;
  
  // Publish joint states
  rcl_ret_t ret = rcl_publish(&joint_state_publisher, &joint_state_msg, NULL);
  if (ret != RCL_RET_OK) {
    Serial.println("Failed to publish joint states");
  }
}

void publishDiagnostics() {
  if (!ros_connected) return;
  
  snprintf(diagnostic_buffer, sizeof(diagnostic_buffer),
    "Uptime: %lu s, Encoders: [%ld,%ld,%ld,%ld], Vel: [%.2f,%.2f,%.2f,%.2f]",
    millis() / 1000,
    encoder_counts[0], encoder_counts[1], encoder_counts[2], encoder_counts[3],
    joint_velocities[0], joint_velocities[1], joint_velocities[2], joint_velocities[3]
  );
  
  diagnostic_msg.data.size = strlen(diagnostic_buffer);
  rcl_publish(&diagnostic_publisher, &diagnostic_msg, NULL);
}

//*****************************************************************************
// UTILITY FUNCTIONS
//*****************************************************************************
void blinkLED(int times, int duration_ms) {
  for (int i = 0; i < times; i++) {
    digitalWrite(STATUS_LED_PIN, HIGH);
    delay(duration_ms);
    digitalWrite(STATUS_LED_PIN, LOW);
    if (i < times - 1) delay(duration_ms);
  }
}
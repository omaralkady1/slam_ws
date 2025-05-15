Okay, I've reviewed and modified your code to address the critical issues and incorporate best practices, especially concerning pin usage, PWM control, and memory management with micro-ROS.

Key Changes Made:

MOTOR_FL_EN_PIN Changed: Changed from 1 (TXD0) to 25. Please verify this pin is free on your specific ESP32 board. If not, choose another suitable GPIO.

ESP32 LEDC PWM: Implemented the ESP32's ledc API for PWM motor control. This allows you to use your defined PWM_FREQUENCY and PWM_RESOLUTION.

setupMotors() now uses ledcSetup() and ledcAttachPin().

setMotorSpeed() now uses ledcWrite().

joint_state_msg Memory Leak Fixed: Added a flag (joint_msg_memory_initialized) to ensure that the dynamic memory for joint_state_msg.name.data and its string elements is allocated only once, preventing a memory leak if initMicroROS() is called multiple times (e.g., during reconnection attempts). Added NULL checks for these malloc calls.

Basic micro-ROS Error Handling: Added rcl_ret_t checks for key micro-ROS initialization functions (rclc_support_init, rclc_node_init_default, rclc_publisher_init_default for one example, rclc_executor_init). A simple error_loop_handler function is called on failure.

Cleaned up Pin Definitions: Removed octal notation (e.g., 01 became 1, then 25 for the conflicting pin).

Comments: Added comments to highlight changes and important considerations.

Important Notes Before Use:

Verify MOTOR_FL_EN_PIN (Pin 25): Ensure GPIO25 is actually available and suitable on your ESP32 board. If it's used by another peripheral (like on-board SPI flash, camera, etc.), you must choose a different free GPIO.

Encoder PPR and Gear Ratio: The values ENCODER_PPR = 11 and GEAR_RATIO = 1.0 are critical. Double-check these against your motor and encoder specifications. Low PPR will lead to poor velocity estimation and control. ENCODER_PPR should be pulses per motor revolution before the gearbox.

PID Tuning: The PID constants (KP, KI, KD) are generic. You will almost certainly need to tune them for your specific robot's mechanics and motors.

Robot Dimensions: Ensure WHEEL_RADIUS, WHEEL_BASE_WIDTH, and WHEEL_BASE_LENGTH accurately reflect your robot.

WiFi Credentials: Replace "alkady1", "omarhohoo@1", and "192.168.1.8" with your actual WiFi SSID, password, and micro-ROS agent IP address.

Here is the modified code:

#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>
#include <sensor_msgs/msg/joint_state.h>
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <std_msgs/msg/string.h>

// Constants for motor pins
// ***** MODIFIED PIN ***** Changed MOTOR_FL_EN_PIN from 1 (TX0) to 25. VERIFY PIN 25 IS FREE ON YOUR BOARD!
#define MOTOR_FL_EN_PIN 25    // Front Left Motor Enable Pin (PWM)
#define MOTOR_FL_DIR1_PIN 26  // Front Left Motor Direction Pin 1
#define MOTOR_FL_DIR2_PIN 27  // Front Left Motor Direction Pin 2
#define MOTOR_FL_ENC_A_PIN 34 // Front Left Motor Encoder A
#define MOTOR_FL_ENC_B_PIN 35 // Front Left Motor Encoder B

#define MOTOR_FR_EN_PIN 14    // Front Right Motor Enable Pin (PWM)
#define MOTOR_FR_DIR1_PIN 12  // Front Right Motor Direction Pin 1
#define MOTOR_FR_DIR2_PIN 13  // Front Right Motor Direction Pin 2
#define MOTOR_FR_ENC_A_PIN 36 // Front Right Motor Encoder A
#define MOTOR_FR_ENC_B_PIN 22 // Front Right Motor Encoder B

#define MOTOR_RL_EN_PIN 33    // Rear Left Motor Enable Pin (PWM)
#define MOTOR_RL_DIR1_PIN 32  // Rear Left Motor Direction Pin 1
#define MOTOR_RL_DIR2_PIN 18  // Rear Left Motor Direction Pin 2
#define MOTOR_RL_ENC_A_PIN 23 // Rear Left Motor Encoder A
#define MOTOR_RL_ENC_B_PIN 21 // Rear Left Motor Encoder B

#define MOTOR_RR_EN_PIN 15    // Rear Right Motor Enable Pin (PWM)
#define MOTOR_RR_DIR1_PIN 4   // Rear Right Motor Direction Pin 1
#define MOTOR_RR_DIR2_PIN 16  // Rear Right Motor Direction Pin 2
#define MOTOR_RR_ENC_A_PIN 19 // Rear Right Motor Encoder A
#define MOTOR_RR_ENC_B_PIN 5  // Rear Right Motor Encoder B

// LED pin for status indication
#define STATUS_LED_PIN 2      // Built-in LED on most ESP32 boards (Strapping pin, usually OK as output)

// PWM properties for ESP32 LEDC
#define PWM_FREQUENCY 20000   // 20 kHz
#define PWM_RESOLUTION 8      // 8-bit resolution (0-255)
#define PWM_MAX_DUTY 255      // Maximum duty cycle for 8-bit resolution

// Motor channels for LEDC PWM and tracking (0-3 used, ESP32 has 16 channels)
#define MOTOR_FL_CHANNEL 0
#define MOTOR_FR_CHANNEL 1
#define MOTOR_RL_CHANNEL 2
#define MOTOR_RR_CHANNEL 3

// Robot dimensions - ADJUST TO MATCH YOUR ROBOT
#define WHEEL_RADIUS 0.05       // meters
#define WHEEL_BASE_WIDTH 0.34   // meters (distance between left and right wheels)
#define WHEEL_BASE_LENGTH 0.30  // meters (distance between front and rear wheels) - Used for some odometry models, not directly in this kinematic one

// Encoder resolution (pulses per revolution) - ADJUST CAREFULLY
#define ENCODER_PPR 11          // Pulses per revolution of the MOTOR SHAFT (before gearbox)
#define GEAR_RATIO 1.0          // Gearbox reduction ratio (e.g., 50.0 for 50:1 gearbox)

// PID Constants - THESE REQUIRE TUNING for your specific motors and robot
#define KP 0.5
#define KI 0.1
#define KD 0.01
#define MAX_PID_OUTPUT 255.0f // Corresponds to max PWM duty
#define MIN_PID_OUTPUT -255.0f

// Control loop frequency
#define CONTROL_FREQUENCY_HZ 50  // 50 Hz control loop
#define ODOMETRY_FREQUENCY_HZ 20 // 20 Hz odometry publishing

// Watchdog timeout (ms) - stop motors if no command received
#define WATCHDOG_TIMEOUT 500

// Interrupt handlers must be global
volatile long encoder_fl_count = 0;
volatile long encoder_fr_count = 0;
volatile long encoder_rl_count = 0;
volatile long encoder_rr_count = 0;
volatile long encoder_fl_prev = 0;
volatile long encoder_fr_prev = 0;
volatile long encoder_rl_prev = 0;
volatile long encoder_rr_prev = 0;

// Target velocities (rad/s for each wheel)
float target_vel_fl = 0.0;
float target_vel_fr = 0.0;
float target_vel_rl = 0.0;
float target_vel_rr = 0.0;

// Current velocities (rad/s for each wheel)
float current_vel_fl = 0.0;
float current_vel_fr = 0.0;
float current_vel_rl = 0.0;
float current_vel_rr = 0.0;

// PID variables
float error_fl = 0.0, error_prev_fl = 0.0, error_sum_fl = 0.0;
float error_fr = 0.0, error_prev_fr = 0.0, error_sum_fr = 0.0;
float error_rl = 0.0, error_prev_rl = 0.0, error_sum_rl = 0.0;
float error_rr = 0.0, error_prev_rr = 0.0, error_sum_rr = 0.0;

// Odometry variables
float odom_x = 0.0;
float odom_y = 0.0;
float odom_theta = 0.0;
float odom_linear_x = 0.0;  // Commanded linear_x, used for reporting in odom msg
float odom_angular_z = 0.0; // Commanded angular_z, used for reporting in odom msg

// Time tracking
unsigned long last_control_time = 0;
unsigned long last_vel_calc_time = 0;
unsigned long last_cmd_time = 0;
// unsigned long last_diagnostic_time = 0; // Managed by diagnostic_timer
unsigned long system_uptime = 0;

// Connection status
bool connected_to_agent = false;

// micro-ROS variables
rcl_subscription_t twist_subscriber;
rcl_publisher_t joint_state_publisher;
rcl_publisher_t odom_publisher;
rcl_publisher_t diagnostic_publisher;
rcl_timer_t control_timer;
rcl_timer_t joint_state_timer;
rcl_timer_t odom_timer;
rcl_timer_t diagnostic_timer;

geometry_msgs__msg__Twist twist_msg;
sensor_msgs__msg__JointState joint_state_msg;
nav_msgs__msg__Odometry odom_msg;
std_msgs__msg__String diagnostic_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// Joint state message buffer
#define JOINT_COUNT 4
const char* joint_names[JOINT_COUNT] = {
  "front_left_wheel_joint",
  "front_right_wheel_joint",
  "rear_left_wheel_joint",
  "rear_right_wheel_joint"
};
double positions[JOINT_COUNT];  // Global, data assigned to msg
double velocities[JOINT_COUNT]; // Global, data assigned to msg
double efforts[JOINT_COUNT];    // Global, data assigned to msg

// Diagnostic buffer
char diagnostic_buffer[100];

// ***** MODIFIED ***** Flag to ensure joint_state_msg memory is allocated only once
bool joint_msg_memory_initialized = false;

// Function prototypes
void IRAM_ATTR encoderISR_FL_A();
void IRAM_ATTR encoderISR_FR_A();
void IRAM_ATTR encoderISR_RL_A();
void IRAM_ATTR encoderISR_RR_A();
void setupMotors();
void setupEncoders();
void setMotorSpeed(int motor_channel, float speed_normalized); // speed_normalized: -1.0 to 1.0
void controlCallback(rcl_timer_t* timer, int64_t last_call_time);
void jointStateCallback(rcl_timer_t* timer, int64_t last_call_time);
void odomCallback(rcl_timer_t* timer, int64_t last_call_time);
void diagnosticCallback(rcl_timer_t* timer, int64_t last_call_time);
void twistCallback(const void* msgin);
float calculatePID(float target, float current, float& error, float& prev_error, float& error_sum, float dt);
void calculateWheelVelocitiesFromTwist(float linear_x, float angular_z);
void updateOdometry(float dt);
void stopMotors();
void initMicroROS();
void blinkLED(int times, int duration_ms);
void error_loop_handler(const char* message); // For critical errors

void setup() {
  Serial.begin(115200);
  // delay(1000); // Optional, wait for serial monitor
  Serial.println("ESP32 4 Motor Controller with micro-ROS starting...");

  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LOW);
  
  setupMotors();
  setupEncoders();

  initMicroROS(); // This can call error_loop_handler if critical init fails

  last_control_time = millis();
  last_vel_calc_time = millis();
  last_cmd_time = millis();
  // last_diagnostic_time = millis(); // No longer needed here
  system_uptime = 0;

  blinkLED(3, 150);
  Serial.println("Initialization complete. Attempting to connect to micro-ROS agent...");
}

void loop() {
  system_uptime = millis() / 1000;
  
  static bool was_connected = false;
  if (!connected_to_agent) { // Try to connect/reconnect if not already connected
      connected_to_agent = rmw_uros_ping_agent(100, 1) == RMW_RET_OK; // Short ping
  }

  if (connected_to_agent) {
    if (!was_connected) {
      Serial.println("Connected to micro-ROS agent!");
      digitalWrite(STATUS_LED_PIN, HIGH); // Solid LED for connected
      was_connected = true;
    }
    
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)); // Process ROS events
    
    if (millis() - last_cmd_time > WATCHDOG_TIMEOUT) {
      if (target_vel_fl != 0 || target_vel_fr != 0 || target_vel_rl != 0 || target_vel_rr != 0) {
        Serial.println("Watchdog timeout: Stopping motors.");
        stopMotors();
      }
    }
  } else { // Not connected or lost connection
    if (was_connected) {
      Serial.println("Connection to micro-ROS agent lost!");
      stopMotors();
      digitalWrite(STATUS_LED_PIN, LOW); // LED off for disconnected
      was_connected = false;
      // Optional: de-initialize ROS entities if a more robust re-init is needed
      // For now, initMicroROS() will be called which attempts re-initialization.
    }
    
    static unsigned long last_reconnect_attempt = 0;
    if (millis() - last_reconnect_attempt > 2000) { // Try to re-establish connection every 2 seconds
      last_reconnect_attempt = millis();
      blinkLED(1, 50); // Quick blink for attempt
      Serial.println("Attempting to (re)connect to agent and re-init ROS...");
      
      // Attempt to set up transport again (in case IP changed or WiFi issue)
      // Note: set_microros_wifi_transports can block.
      // If it fails, initMicroROS will also likely fail to create entities.
      set_microros_wifi_transports("alkady1", "omarhohoo@1", "192.168.1.8", 8888); // YOUR WIFI AND AGENT DETAILS
      delay(100); // Small delay after transport setup

      // Re-initialize ROS entities. If this fails critically, it will call error_loop_handler.
      initMicroROS(); 
      
      // After initMicroROS, try pinging again to update connected_to_agent
      connected_to_agent = rmw_uros_ping_agent(100, 1) == RMW_RET_OK;
      if(connected_to_agent){
          Serial.println("Reconnected successfully after initMicroROS!");
          digitalWrite(STATUS_LED_PIN, HIGH);
          was_connected = true; // Ensure status is updated
      } else {
          Serial.println("Still not connected after re-init attempt.");
      }
    }
  }
}

void error_loop_handler(const char* message) {
  Serial.print("CRITICAL ERROR: ");
  Serial.println(message);
  Serial.println("System halted. Please reset.");
  while(true) {
    blinkLED(1, 250); // Slower, more noticeable error blink
    delay(750); // Longer pause
  }
}

void initMicroROS() {
  Serial.println("Initializing micro-ROS entities...");
  rcl_ret_t ret;

  // If WiFi transport isn't set here, ensure it's set before calling this function.
  // Moved set_microros_wifi_transports to the loop for reconnection logic.
  // If called from setup, it should be there too.
  // For setup:
  if (millis() < 5000) { // Crude check if we are in early setup phase
      set_microros_wifi_transports("alkady1", "omarhohoo@1", "192.168.1.8", 8888); // YOUR WIFI AND AGENT DETAILS
      delay(2000); // Give WiFi time to connect during initial setup
  }
  
  allocator = rcl_get_default_allocator();

  // Initialize support and create node
  ret = rclc_support_init(&support, 0, NULL, &allocator);
  if (ret != RCL_RET_OK) {
    error_loop_handler("Failed to init rclc_support.");
    return; // Will halt in error_loop_handler
  }
  ret = rclc_node_init_default(&node, "esp32_motor_controller", "", &support);
  if (ret != RCL_RET_OK) {
    error_loop_handler("Failed to init rclc_node.");
    return;
  }
  Serial.println("ROS2 node created.");

  // Initialize joint state message
  joint_state_msg.header.frame_id.data = (char*)"base_link"; // String literal, safe
  joint_state_msg.header.frame_id.size = strlen(joint_state_msg.header.frame_id.data);
  joint_state_msg.header.frame_id.capacity = joint_state_msg.header.frame_id.size + 1;
  
  // ***** MODIFIED ***** Memory allocation for joint names, only if not already done
  if (!joint_msg_memory_initialized) {
    joint_state_msg.name.capacity = JOINT_COUNT;
    joint_state_msg.name.size = JOINT_COUNT;
    joint_state_msg.name.data = (rosidl_runtime_c__String*)malloc(JOINT_COUNT * sizeof(rosidl_runtime_c__String));
    if (joint_state_msg.name.data == NULL) {
        error_loop_handler("Failed to allocate joint_state_msg.name.data"); return;
    }
    
    for (int i = 0; i < JOINT_COUNT; i++) {
      size_t len = strlen(joint_names[i]);
      joint_state_msg.name.data[i].data = (char*)malloc((len + 1) * sizeof(char));
      if (joint_state_msg.name.data[i].data == NULL) {
          char err_buf[50]; sprintf(err_buf, "Failed to alloc joint_state_msg.name.data[%d]", i);
          error_loop_handler(err_buf); return;
      }
      joint_state_msg.name.data[i].size = len;
      joint_state_msg.name.data[i].capacity = len + 1;
      memcpy(joint_state_msg.name.data[i].data, joint_names[i], len + 1); // Include null terminator
    }
    joint_msg_memory_initialized = true;
  }
  // Assign pointers to global data arrays (safe)
  joint_state_msg.position.data = positions;
  joint_state_msg.position.size = JOINT_COUNT;
  joint_state_msg.position.capacity = JOINT_COUNT;
  joint_state_msg.velocity.data = velocities;
  joint_state_msg.velocity.size = JOINT_COUNT;
  joint_state_msg.velocity.capacity = JOINT_COUNT;
  joint_state_msg.effort.data = efforts;
  joint_state_msg.effort.size = JOINT_COUNT;
  joint_state_msg.effort.capacity = JOINT_COUNT;

  // Initialize odometry message
  odom_msg.header.frame_id.data = (char*)"odom"; // String literal
  odom_msg.header.frame_id.size = strlen(odom_msg.header.frame_id.data);
  odom_msg.header.frame_id.capacity = odom_msg.header.frame_id.size + 1;
  odom_msg.child_frame_id.data = (char*)"base_link"; // String literal
  odom_msg.child_frame_id.size = strlen(odom_msg.child_frame_id.data);
  odom_msg.child_frame_id.capacity = odom_msg.child_frame_id.size + 1;

  // Initialize diagnostic message (uses global buffer, safe)
  diagnostic_msg.data.capacity = sizeof(diagnostic_buffer);
  diagnostic_msg.data.size = 0;
  diagnostic_msg.data.data = diagnostic_buffer;

  // Create subscription
  ret = rclc_subscription_init_default(
    &twist_subscriber, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel");
  if (ret != RCL_RET_OK) { error_loop_handler("Failed to init twist_subscriber."); return; }

  // Create publishers
  ret = rclc_publisher_init_default(
    &joint_state_publisher, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState), "joint_states");
  if (ret != RCL_RET_OK) { error_loop_handler("Failed to init joint_state_publisher."); return; }
    
  ret = rclc_publisher_init_default(
    &odom_publisher, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "odom");
  if (ret != RCL_RET_OK) { error_loop_handler("Failed to init odom_publisher."); return; }
    
  ret = rclc_publisher_init_default(
    &diagnostic_publisher, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "esp32/diagnostics");
  if (ret != RCL_RET_OK) { error_loop_handler("Failed to init diagnostic_publisher."); return; }

  Serial.println("Publishers and subscriber created.");

  // Create timers
  ret = rclc_timer_init_default(&control_timer, &support, RCL_MS_TO_NS(1000 / CONTROL_FREQUENCY_HZ), controlCallback);
  if (ret != RCL_RET_OK) { error_loop_handler("Failed to init control_timer."); return; }

  ret = rclc_timer_init_default(&joint_state_timer, &support, RCL_MS_TO_NS(100), jointStateCallback); // 10Hz
  if (ret != RCL_RET_OK) { error_loop_handler("Failed to init joint_state_timer."); return; }
    
  ret = rclc_timer_init_default(&odom_timer, &support, RCL_MS_TO_NS(1000 / ODOMETRY_FREQUENCY_HZ), odomCallback);
  if (ret != RCL_RET_OK) { error_loop_handler("Failed to init odom_timer."); return; }
    
  ret = rclc_timer_init_default(&diagnostic_timer, &support, RCL_MS_TO_NS(5000), diagnosticCallback); // 0.2Hz
  if (ret != RCL_RET_OK) { error_loop_handler("Failed to init diagnostic_timer."); return; }
  Serial.println("Timers created.");

  // Create executor (ensure number of handles matches added entities)
  // 1 sub + 4 timers = 5 handles
  ret = rclc_executor_init(&executor, &support.context, 5, &allocator);
  if (ret != RCL_RET_OK) { error_loop_handler("Failed to init executor."); return; }

  rclc_executor_add_timer(&executor, &control_timer);
  rclc_executor_add_timer(&executor, &joint_state_timer);
  rclc_executor_add_timer(&executor, &odom_timer);
  rclc_executor_add_timer(&executor, &diagnostic_timer);
  rclc_executor_add_subscription(&executor, &twist_subscriber, &twist_msg, &twistCallback, ON_NEW_DATA);
  Serial.println("Executor configured. micro-ROS initialization sequence complete.");
}

// ***** MODIFIED ***** Setup motors with ESP32 LEDC PWM
void setupMotors() {
  pinMode(MOTOR_FL_DIR1_PIN, OUTPUT);
  pinMode(MOTOR_FL_DIR2_PIN, OUTPUT);
  pinMode(MOTOR_FR_DIR1_PIN, OUTPUT);
  pinMode(MOTOR_FR_DIR2_PIN, OUTPUT);
  pinMode(MOTOR_RL_DIR1_PIN, OUTPUT);
  pinMode(MOTOR_RL_DIR2_PIN, OUTPUT);
  pinMode(MOTOR_RR_DIR1_PIN, OUTPUT);
  pinMode(MOTOR_RR_DIR2_PIN, OUTPUT);

  // Initialize LEDC PWM channels
  ledcSetup(MOTOR_FL_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttachPin(MOTOR_FL_EN_PIN, MOTOR_FL_CHANNEL);
  
  ledcSetup(MOTOR_FR_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttachPin(MOTOR_FR_EN_PIN, MOTOR_FR_CHANNEL);

  ledcSetup(MOTOR_RL_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttachPin(MOTOR_RL_EN_PIN, MOTOR_RL_CHANNEL);

  ledcSetup(MOTOR_RR_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttachPin(MOTOR_RR_EN_PIN, MOTOR_RR_CHANNEL);
  
  // Initialize motors to stopped state (PWM duty 0)
  ledcWrite(MOTOR_FL_CHANNEL, 0);
  ledcWrite(MOTOR_FR_CHANNEL, 0);
  ledcWrite(MOTOR_RL_CHANNEL, 0);
  ledcWrite(MOTOR_RR_CHANNEL, 0);

  // Set direction pins to default (brake or coast, depending on driver)
  digitalWrite(MOTOR_FL_DIR1_PIN, LOW); digitalWrite(MOTOR_FL_DIR2_PIN, LOW);
  digitalWrite(MOTOR_FR_DIR1_PIN, LOW); digitalWrite(MOTOR_FR_DIR2_PIN, LOW);
  digitalWrite(MOTOR_RL_DIR1_PIN, LOW); digitalWrite(MOTOR_RL_DIR2_PIN, LOW);
  digitalWrite(MOTOR_RR_DIR1_PIN, LOW); digitalWrite(MOTOR_RR_DIR2_PIN, LOW);

  Serial.println("Motors initialized using LEDC PWM.");
}

void setupEncoders() {
  pinMode(MOTOR_FL_ENC_A_PIN, INPUT_PULLUP); pinMode(MOTOR_FL_ENC_B_PIN, INPUT_PULLUP);
  pinMode(MOTOR_FR_ENC_A_PIN, INPUT_PULLUP); pinMode(MOTOR_FR_ENC_B_PIN, INPUT_PULLUP);
  pinMode(MOTOR_RL_ENC_A_PIN, INPUT_PULLUP); pinMode(MOTOR_RL_ENC_B_PIN, INPUT_PULLUP);
  pinMode(MOTOR_RR_ENC_A_PIN, INPUT_PULLUP); pinMode(MOTOR_RR_ENC_B_PIN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(MOTOR_FL_ENC_A_PIN), encoderISR_FL_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTOR_FR_ENC_A_PIN), encoderISR_FR_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTOR_RL_ENC_A_PIN), encoderISR_RL_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTOR_RR_ENC_A_PIN), encoderISR_RR_A, CHANGE);

  Serial.println("Encoders initialized with interrupts.");
}

// Encoder ISRs - Ensure logic matches your encoder wiring and desired direction
void IRAM_ATTR encoderISR_FL_A() { // Front Left
  if (digitalRead(MOTOR_FL_ENC_A_PIN) == digitalRead(MOTOR_FL_ENC_B_PIN)) {
    encoder_fl_count--; // Or ++ depending on wiring
  } else {
    encoder_fl_count++; // Or --
  }
}
void IRAM_ATTR encoderISR_FR_A() { // Front Right
  if (digitalRead(MOTOR_FR_ENC_A_PIN) == digitalRead(MOTOR_FR_ENC_B_PIN)) {
    encoder_fr_count++; // Or -- (often opposite of left for same robot forward motion)
  } else {
    encoder_fr_count--; // Or ++
  }
}
void IRAM_ATTR encoderISR_RL_A() { // Rear Left
  if (digitalRead(MOTOR_RL_ENC_A_PIN) == digitalRead(MOTOR_RL_ENC_B_PIN)) {
    encoder_rl_count--; // Or ++
  } else {
    encoder_rl_count++; // Or --
  }
}
void IRAM_ATTR encoderISR_RR_A() { // Rear Right
  if (digitalRead(MOTOR_RR_ENC_A_PIN) == digitalRead(MOTOR_RR_ENC_B_PIN)) {
    encoder_rr_count++; // Or --
  } else {
    encoder_rr_count--; // Or ++
  }
}


// ***** MODIFIED ***** Set motor speed using ESP32 LEDC PWM
// motor_channel is MOTOR_FL_CHANNEL, etc.
// speed_normalized is from -1.0 (full reverse) to 1.0 (full forward)
void setMotorSpeed(int motor_channel, float speed_normalized) {
  int pwm_value;
  bool forward_direction; // True for forward, false for reverse for this motor
  int dir1_pin, dir2_pin;
  
  speed_normalized = constrain(speed_normalized, -1.0, 1.0);
  
  forward_direction = (speed_normalized >= 0.0);
  pwm_value = (int)(abs(speed_normalized) * PWM_MAX_DUTY);
  pwm_value = constrain(pwm_value, 0, PWM_MAX_DUTY); // Ensure it's within 0-255
  
  switch (motor_channel) {
    case MOTOR_FL_CHANNEL:
      dir1_pin = MOTOR_FL_DIR1_PIN; dir2_pin = MOTOR_FL_DIR2_PIN; break;
    case MOTOR_FR_CHANNEL:
      dir1_pin = MOTOR_FR_DIR1_PIN; dir2_pin = MOTOR_FR_DIR2_PIN; break;
    case MOTOR_RL_CHANNEL:
      dir1_pin = MOTOR_RL_DIR1_PIN; dir2_pin = MOTOR_RL_DIR2_PIN; break;
    case MOTOR_RR_CHANNEL:
      dir1_pin = MOTOR_RR_DIR1_PIN; dir2_pin = MOTOR_RR_DIR2_PIN; break;
    default: return; // Invalid motor channel
  }
  
  // Standard H-Bridge logic: DIR1=HIGH, DIR2=LOW for forward (example)
  // Adjust if your H-bridge or motor wiring is different
  digitalWrite(dir1_pin, forward_direction ? HIGH : LOW);
  digitalWrite(dir2_pin, forward_direction ? LOW : HIGH);
  
  ledcWrite(motor_channel, pwm_value);
}

void stopMotors() {
  setMotorSpeed(MOTOR_FL_CHANNEL, 0.0);
  setMotorSpeed(MOTOR_FR_CHANNEL, 0.0);
  setMotorSpeed(MOTOR_RL_CHANNEL, 0.0);
  setMotorSpeed(MOTOR_RR_CHANNEL, 0.0);
  
  target_vel_fl = 0.0; target_vel_fr = 0.0;
  target_vel_rl = 0.0; target_vel_rr = 0.0;
  
  // Optionally reset PID errors/sums here if desired upon full stop
  error_sum_fl = 0; error_sum_fr = 0; error_sum_rl = 0; error_sum_rr = 0;
}

float calculatePID(float target, float current, float& error, float& prev_error, float& error_sum, float dt) {
  if (dt == 0) return 0; // Avoid division by zero if dt is somehow 0
  error = target - current;
  
  error_sum += error * dt;
  // Anti-windup for integral term
  if (KI != 0) { // Avoid division by zero if KI is zero
    error_sum = constrain(error_sum, MIN_PID_OUTPUT / KI, MAX_PID_OUTPUT / KI);
  } else {
    error_sum = 0; // No integral action if KI is zero
  }
  
  float error_delta = (error - prev_error) / dt;
  prev_error = error;
  
  float output = KP * error + KI * error_sum + KD * error_delta;
  return constrain(output, MIN_PID_OUTPUT, MAX_PID_OUTPUT);
}

void controlCallback(rcl_timer_t* timer, int64_t last_call_time) {
  if (timer == NULL) return; // Guard

  unsigned long now = millis();
  float dt_control = (now - last_control_time) / 1000.0f;
  last_control_time = now;
  if (dt_control <= 0) dt_control = 1.0f / CONTROL_FREQUENCY_HZ; // Fallback if time wrap or no change

  // Calculate current wheel velocities from encoders
  unsigned long vel_time_now = millis();
  float dt_vel = (vel_time_now - last_vel_calc_time) / 1000.0f;
  last_vel_calc_time = vel_time_now;

  if (dt_vel <= 0) { // If no time has passed, skip velocity update to avoid div by zero
      // Keep previous velocities or set to zero if it's the first run
  } else {
    // Pulses to radians: (2 * PI rad / rev) / (pulses_per_rev_motor * gear_ratio)
    float pulses_to_rad = (2.0 * PI) / (ENCODER_PPR * GEAR_RATIO);
    
    // Atomically read encoder counts (optional, but good practice if needed elsewhere)
    long fl_count = encoder_fl_count; long fr_count = encoder_fr_count;
    long rl_count = encoder_rl_count; long rr_count = encoder_rr_count;
    
    current_vel_fl = ((fl_count - encoder_fl_prev) * pulses_to_rad) / dt_vel;
    current_vel_fr = ((fr_count - encoder_fr_prev) * pulses_to_rad) / dt_vel;
    current_vel_rl = ((rl_count - encoder_rl_prev) * pulses_to_rad) / dt_vel;
    current_vel_rr = ((rr_count - encoder_rr_prev) * pulses_to_rad) / dt_vel;
    
    encoder_fl_prev = fl_count; encoder_fr_prev = fr_count;
    encoder_rl_prev = rl_count; encoder_rr_prev = rr_count;
  }
  
  // Update odometry (using actual dt_control from this loop)
  updateOdometry(dt_control); 
  
  // PID controllers
  float output_fl_pid = calculatePID(target_vel_fl, current_vel_fl, error_fl, error_prev_fl, error_sum_fl, dt_control);
  float output_fr_pid = calculatePID(target_vel_fr, current_vel_fr, error_fr, error_prev_fr, error_sum_fr, dt_control);
  float output_rl_pid = calculatePID(target_vel_rl, current_vel_rl, error_rl, error_prev_rl, error_sum_rl, dt_control);
  float output_rr_pid = calculatePID(target_vel_rr, current_vel_rr, error_rr, error_prev_rr, error_sum_rr, dt_control);
  
  // Scale PID outputs (-MAX_PID_OUTPUT to +MAX_PID_OUTPUT) to motor speed (-1.0 to 1.0)
  setMotorSpeed(MOTOR_FL_CHANNEL, output_fl_pid / MAX_PID_OUTPUT);
  setMotorSpeed(MOTOR_FR_CHANNEL, output_fr_pid / MAX_PID_OUTPUT);
  setMotorSpeed(MOTOR_RL_CHANNEL, output_rl_pid / MAX_PID_OUTPUT);
  setMotorSpeed(MOTOR_RR_CHANNEL, output_rr_pid / MAX_PID_OUTPUT);
}

void jointStateCallback(rcl_timer_t* timer, int64_t last_call_time) {
  if (timer == NULL || !connected_to_agent) return;

  struct timespec ts;
  clock_gettime(CLOCK_REALTIME, &ts); // Or use a micro-ROS provided time function if available and synced
  joint_state_msg.header.stamp.sec = ts.tv_sec;
  joint_state_msg.header.stamp.nanosec = ts.tv_nsec;
  
  float enc_to_rad = (2.0 * PI) / (ENCODER_PPR * GEAR_RATIO);
  positions[0] = (double)encoder_fl_count * enc_to_rad;
  positions[1] = (double)encoder_fr_count * enc_to_rad;
  positions[2] = (double)encoder_rl_count * enc_to_rad;
  positions[3] = (double)encoder_rr_count * enc_to_rad;
  
  velocities[0] = (double)current_vel_fl;
  velocities[1] = (double)current_vel_fr;
  velocities[2] = (double)current_vel_rl;
  velocities[3] = (double)current_vel_rr;
  
  // Efforts could be estimated from PWM duty cycle if desired, or PID output
  efforts[0] = 0.0; efforts[1] = 0.0; efforts[2] = 0.0; efforts[3] = 0.0;
  
  rcl_publish(&joint_state_publisher, &joint_state_msg, NULL);
}

void odomCallback(rcl_timer_t* timer, int64_t last_call_time) {
  if (timer == NULL || !connected_to_agent) return;

  struct timespec ts;
  clock_gettime(CLOCK_REALTIME, &ts);
  odom_msg.header.stamp.sec = ts.tv_sec;
  odom_msg.header.stamp.nanosec = ts.tv_nsec;
  
  odom_msg.pose.pose.position.x = (double)odom_x;
  odom_msg.pose.pose.position.y = (double)odom_y;
  odom_msg.pose.pose.position.z = 0.0;
  
  float sin_half_theta = sin(odom_theta / 2.0f);
  float cos_half_theta = cos(odom_theta / 2.0f);
  odom_msg.pose.pose.orientation.x = 0.0;
  odom_msg.pose.pose.orientation.y = 0.0;
  odom_msg.pose.pose.orientation.z = (double)sin_half_theta;
  odom_msg.pose.pose.orientation.w = (double)cos_half_theta;
  
  // Report the *commanded* velocities in the odom twist part, 
  // or calculate actual robot body velocities if preferred.
  // The current `odom_linear_x` and `odom_angular_z` are set from the last twist command.
  odom_msg.twist.twist.linear.x = (double)odom_linear_x;
  odom_msg.twist.twist.linear.y = 0.0;
  odom_msg.twist.twist.angular.z = (double)odom_angular_z;
  
  // Simplified covariance
  for (int i = 0; i < 36; i++) { odom_msg.pose.covariance[i] = 0.0; odom_msg.twist.covariance[i] = 0.0; }
  odom_msg.pose.covariance[0] = 0.01;  // x
  odom_msg.pose.covariance[7] = 0.01;  // y
  odom_msg.pose.covariance[35] = 0.05; // yaw
  odom_msg.twist.covariance[0] = 0.01; // vx
  odom_msg.twist.covariance[35] = 0.05;// vyaw
  
  rcl_publish(&odom_publisher, &odom_msg, NULL);
}

void diagnosticCallback(rcl_timer_t* timer, int64_t last_call_time) {
  if (timer == NULL || !connected_to_agent) return;

  // Placeholder values for voltage and temp - implement actual readings if sensors exist
  float voltage = 5.0; // Example: Read from ADC mapped to battery voltage
  float temp = 25.0;   // Example: Read from internal ESP32 temp sensor or external one
  
  snprintf(diagnostic_buffer, sizeof(diagnostic_buffer), 
           "{\"uptime_s\":%lu,\"voltage_V\":%.2f,\"temp_C\":%.1f,\"agent_conn\":%d}",
           system_uptime, voltage, temp, connected_to_agent ? 1 : 0);
  
  diagnostic_msg.data.size = strlen(diagnostic_buffer);
  rcl_publish(&diagnostic_publisher, &diagnostic_msg, NULL);
}

void twistCallback(const void* msgin) {
  const geometry_msgs__msg__Twist* msg = (const geometry_msgs__msg__Twist*)msgin;
  
  float linear_x_cmd = msg->linear.x;
  float angular_z_cmd = msg->angular.z;
  
  calculateWheelVelocitiesFromTwist(linear_x_cmd, angular_z_cmd);
  
  last_cmd_time = millis(); // Update watchdog timer
  
  // Store commanded velocities for reporting in odometry message
  odom_linear_x = linear_x_cmd;
  odom_angular_z = angular_z_cmd;
}

// Differential drive kinematics for target wheel velocities
void calculateWheelVelocitiesFromTwist(float linear_x, float angular_z) {
  // v_left = linear_x - angular_z * (WHEEL_BASE_WIDTH / 2)
  // v_right = linear_x + angular_z * (WHEEL_BASE_WIDTH / 2)
  // wheel_ang_vel = v_wheel / WHEEL_RADIUS
  
  float v_robot_linear_component = linear_x;
  float v_robot_angular_component_at_wheel = angular_z * (WHEEL_BASE_WIDTH / 2.0f);
  
  float v_left_wheels  = v_robot_linear_component - v_robot_angular_component_at_wheel;
  float v_right_wheels = v_robot_linear_component + v_robot_angular_component_at_wheel;
  
  // Convert linear wheel velocity (m/s) to angular wheel velocity (rad/s)
  target_vel_fl = v_left_wheels / WHEEL_RADIUS;
  target_vel_rl = v_left_wheels / WHEEL_RADIUS; // Assuming FL and RL are mechanically linked or should behave same
  
  target_vel_fr = v_right_wheels / WHEEL_RADIUS;
  target_vel_rr = v_right_wheels / WHEEL_RADIUS; // Assuming FR and RR
}

void updateOdometry(float dt) {
  if (dt <= 0) return;
  // Average actual wheel angular velocities (rad/s) for left and right sides
  float avg_ang_vel_left  = (current_vel_fl + current_vel_rl) / 2.0f;
  float avg_ang_vel_right = (current_vel_fr + current_vel_rr) / 2.0f;
  
  // Convert to linear wheel velocities (m/s)
  float v_left_actual  = avg_ang_vel_left  * WHEEL_RADIUS;
  float v_right_actual = avg_ang_vel_right * WHEEL_RADIUS;
  
  // Calculate robot's actual linear and angular velocity
  float v_robot_actual     = (v_right_actual + v_left_actual) / 2.0f;
  float omega_robot_actual = (v_right_actual - v_left_actual) / WHEEL_BASE_WIDTH;
  
  // Integrate to update pose
  float delta_x = v_robot_actual * cos(odom_theta) * dt;
  float delta_y = v_robot_actual * sin(odom_theta) * dt;
  float delta_theta = omega_robot_actual * dt;
  
  odom_x += delta_x;
  odom_y += delta_y;
  odom_theta += delta_theta;
  
  // Normalize odom_theta to [-PI, PI]
  while (odom_theta > PI) odom_theta -= (2.0f * PI);
  while (odom_theta < -PI) odom_theta += (2.0f * PI);
}

void blinkLED(int times, int duration_ms) {
  for (int i = 0; i < times; i++) {
    digitalWrite(STATUS_LED_PIN, HIGH);
    delay(duration_ms);
    digitalWrite(STATUS_LED_PIN, LOW);
    if (i < times -1 ) delay(duration_ms); // Don't delay after last blink if not needed
  }
}

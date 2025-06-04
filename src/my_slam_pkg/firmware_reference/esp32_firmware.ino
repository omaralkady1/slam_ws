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

// --- IMPORTANT NOTE ---
// This version uses analogWrite() as a WORKAROUND for PWM control
// because ledcSetup/ledcAttachPin/ledcWrite might be unavailable
// in older/corrupted ESP32 Arduino Core installations.
// --> UPDATE YOUR ESP32 CORE for proper LEDC PWM control! <--
// The PWM_FREQUENCY and PWM_RESOLUTION defines below may not be effective.
// --- END IMPORTANT NOTE ---


// Constants for motor pins
#define MOTOR_FL_EN_PIN 25    // Front Left Motor Enable Pin (PWM) - VERIFY PIN 25 IS FREE!
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
#define STATUS_LED_PIN 2      // Built-in LED on most ESP32 boards

// PWM properties (NOTE: MAY NOT BE EFFECTIVE WITH analogWrite WORKAROUND)
#define PWM_FREQUENCY 20000   // Desired frequency (e.g., 20 kHz)
#define PWM_RESOLUTION 8      // Desired resolution (e.g., 8-bit)
#define PWM_MAX_DUTY 255      // Maximum duty cycle for 8-bit resolution

// Motor channels/identifiers (used for tracking and function calls)
#define MOTOR_FL_ID 0 // Using simple IDs instead of LEDC channels now
#define MOTOR_FR_ID 1
#define MOTOR_RL_ID 2
#define MOTOR_RR_ID 3

// Robot dimensions - ADJUST TO MATCH YOUR ROBOT
#define WHEEL_RADIUS 0.05       // meters
#define WHEEL_BASE_WIDTH 0.34   // meters (distance between left and right wheels)
#define WHEEL_BASE_LENGTH 0.30  // meters (distance between front and rear wheels)

// Encoder resolution (pulses per revolution) - ADJUST CAREFULLY
#define ENCODER_PPR 11          // Pulses per revolution of the MOTOR SHAFT (before gearbox)
#define GEAR_RATIO 1.0          // Gearbox reduction ratio (e.g., 50.0 for 50:1 gearbox)

// PID Constants - THESE REQUIRE TUNING
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

// Serial connection baudrate
#define SERIAL_BAUDRATE 115200

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

// Flag to ensure joint_state_msg memory is allocated only once
bool joint_msg_memory_initialized = false;

// Function prototypes
void IRAM_ATTR encoderISR_FL_A();
void IRAM_ATTR encoderISR_FR_A();
void IRAM_ATTR encoderISR_RL_A();
void IRAM_ATTR encoderISR_RR_A();
void setupMotors(); // Modified to use analogWrite
void setupEncoders();
void setMotorSpeed(int motor_id, float speed_normalized); // Modified to use analogWrite
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

//*****************************************************************************
// SETUP FUNCTION
//*****************************************************************************
void setup() {
  Serial.begin(SERIAL_BAUDRATE);
  Serial.println("ESP32 4 Motor Controller (USB Serial) starting...");

  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LOW);

  setupMotors(); // Uses the analogWrite workaround version
  setupEncoders();

  // Initialize micro-ROS transport over USB serial
  set_microros_serial_transports(Serial);
  delay(2000); // Give Serial time to initialize
  
  initMicroROS(); // Can call error_loop_handler if critical init fails

  last_control_time = millis();
  last_vel_calc_time = millis();
  last_cmd_time = millis();
  system_uptime = 0;

  blinkLED(3, 150);
  Serial.println("Initialization complete. Attempting to connect to micro-ROS agent...");
}

//*****************************************************************************
// MAIN LOOP FUNCTION
//*****************************************************************************
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

    // Watchdog check
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
    }

    // Reconnection attempt logic
    static unsigned long last_reconnect_attempt = 0;
    if (millis() - last_reconnect_attempt > 2000) { // Try every 2 seconds
      last_reconnect_attempt = millis();
      blinkLED(1, 50); // Quick blink for attempt
      Serial.println("Attempting to (re)connect to agent...");

      // Re-initialize ROS entities
      initMicroROS();

      // Try pinging again
      connected_to_agent = rmw_uros_ping_agent(100, 1) == RMW_RET_OK;
      if(connected_to_agent){
          Serial.println("Reconnected successfully!");
          digitalWrite(STATUS_LED_PIN, HIGH);
          was_connected = true;
      } else {
          Serial.println("Still not connected after re-init attempt.");
      }
    }
  }
}

//*****************************************************************************
// ERROR HANDLING FUNCTION
//*****************************************************************************
void error_loop_handler(const char* message) {
  Serial.print("CRITICAL ERROR: ");
  Serial.println(message);
  Serial.println("System halted. Please reset.");
  while(true) {
    blinkLED(1, 250); // Slower, noticeable error blink
    delay(750);
  }
}

//*****************************************************************************
// micro-ROS INITIALIZATION
//*****************************************************************************
void initMicroROS() {
  Serial.println("Initializing micro-ROS entities...");
  rcl_ret_t ret;

  allocator = rcl_get_default_allocator();

  // Initialize support
  ret = rclc_support_init(&support, 0, NULL, &allocator);
  if (ret != RCL_RET_OK) { error_loop_handler("Failed to init rclc_support."); return; }

  // Initialize node
  ret = rclc_node_init_default(&node, "esp32_motor_controller", "", &support);
  if (ret != RCL_RET_OK) { error_loop_handler("Failed to init rclc_node."); return; }
  Serial.println("ROS2 node created.");

  // --- Initialize Joint State Message ---
  joint_state_msg.header.frame_id.data = (char*)"base_link";
  joint_state_msg.header.frame_id.size = strlen(joint_state_msg.header.frame_id.data);
  joint_state_msg.header.frame_id.capacity = joint_state_msg.header.frame_id.size + 1;

  // Allocate memory for joint names only once
  if (!joint_msg_memory_initialized) {
    joint_state_msg.name.capacity = JOINT_COUNT;
    joint_state_msg.name.size = JOINT_COUNT;
    joint_state_msg.name.data = (rosidl_runtime_c__String*)malloc(JOINT_COUNT * sizeof(rosidl_runtime_c__String));
    if (joint_state_msg.name.data == NULL) { error_loop_handler("Failed to allocate joint_state_msg.name.data"); return; }

    bool alloc_ok = true;
    for (int i = 0; i < JOINT_COUNT; i++) {
      size_t len = strlen(joint_names[i]);
      joint_state_msg.name.data[i].data = (char*)malloc((len + 1) * sizeof(char));
      if (joint_state_msg.name.data[i].data == NULL) {
          char err_buf[50]; sprintf(err_buf, "Failed to alloc joint_state_msg.name.data[%d]", i);
          error_loop_handler(err_buf); alloc_ok = false; break; // Exit loop on first failure
      }
      joint_state_msg.name.data[i].size = len;
      joint_state_msg.name.data[i].capacity = len + 1;
      memcpy(joint_state_msg.name.data[i].data, joint_names[i], len + 1);
    }
    if (!alloc_ok) return; // Stop initialization if allocation failed

    joint_msg_memory_initialized = true;
    Serial.println("Joint state message names allocated.");
  }
  // Assign pointers to global data arrays
  joint_state_msg.position.data = positions; joint_state_msg.position.size = JOINT_COUNT; joint_state_msg.position.capacity = JOINT_COUNT;
  joint_state_msg.velocity.data = velocities; joint_state_msg.velocity.size = JOINT_COUNT; joint_state_msg.velocity.capacity = JOINT_COUNT;
  joint_state_msg.effort.data = efforts; joint_state_msg.effort.size = JOINT_COUNT; joint_state_msg.effort.capacity = JOINT_COUNT;

  // --- Initialize Odometry Message ---
  odom_msg.header.frame_id.data = (char*)"odom";
  odom_msg.header.frame_id.size = strlen(odom_msg.header.frame_id.data);
  odom_msg.header.frame_id.capacity = odom_msg.header.frame_id.size + 1;
  odom_msg.child_frame_id.data = (char*)"base_link";
  odom_msg.child_frame_id.size = strlen(odom_msg.child_frame_id.data);
  odom_msg.child_frame_id.capacity = odom_msg.child_frame_id.size + 1;

  // --- Initialize Diagnostic Message ---
  diagnostic_msg.data.capacity = sizeof(diagnostic_buffer);
  diagnostic_msg.data.size = 0;
  diagnostic_msg.data.data = diagnostic_buffer;

  // --- Create Subscription ---
  ret = rclc_subscription_init_default(&twist_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel");
  if (ret != RCL_RET_OK) { error_loop_handler("Failed to init twist_subscriber."); return; }

  // --- Create Publishers ---
  ret = rclc_publisher_init_default(&joint_state_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState), "joint_states");
  if (ret != RCL_RET_OK) { error_loop_handler("Failed to init joint_state_publisher."); return; }
  ret = rclc_publisher_init_default(&odom_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "odom");
  if (ret != RCL_RET_OK) { error_loop_handler("Failed to init odom_publisher."); return; }
  ret = rclc_publisher_init_default(&diagnostic_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "esp32/diagnostics");
  if (ret != RCL_RET_OK) { error_loop_handler("Failed to init diagnostic_publisher."); return; }
  Serial.println("Publishers and subscriber created.");

  // --- Create Timers ---
  ret = rclc_timer_init_default(&control_timer, &support, RCL_MS_TO_NS(1000 / CONTROL_FREQUENCY_HZ), controlCallback);
  if (ret != RCL_RET_OK) { error_loop_handler("Failed to init control_timer."); return; }
  ret = rclc_timer_init_default(&joint_state_timer, &support, RCL_MS_TO_NS(100), jointStateCallback); // 10Hz
  if (ret != RCL_RET_OK) { error_loop_handler("Failed to init joint_state_timer."); return; }
  ret = rclc_timer_init_default(&odom_timer, &support, RCL_MS_TO_NS(1000 / ODOMETRY_FREQUENCY_HZ), odomCallback);
  if (ret != RCL_RET_OK) { error_loop_handler("Failed to init odom_timer."); return; }
  ret = rclc_timer_init_default(&diagnostic_timer, &support, RCL_MS_TO_NS(5000), diagnosticCallback); // 0.2Hz
  if (ret != RCL_RET_OK) { error_loop_handler("Failed to init diagnostic_timer."); return; }
  Serial.println("Timers created.");

  // --- Create Executor ---
  ret = rclc_executor_init(&executor, &support.context, 5, &allocator); // 1 sub + 4 timers = 5 handles
  if (ret != RCL_RET_OK) { error_loop_handler("Failed to init executor."); return; }
  rclc_executor_add_timer(&executor, &control_timer);
  rclc_executor_add_timer(&executor, &joint_state_timer);
  rclc_executor_add_timer(&executor, &odom_timer);
  rclc_executor_add_timer(&executor, &diagnostic_timer);
  rclc_executor_add_subscription(&executor, &twist_subscriber, &twist_msg, &twistCallback, ON_NEW_DATA);
  Serial.println("Executor configured. micro-ROS initialization sequence complete.");
}


//*****************************************************************************
// MOTOR & ENCODER SETUP FUNCTIONS
//*****************************************************************************

// ***** MODIFIED: Using analogWrite WORKAROUND *****
void setupMotors() {
  pinMode(MOTOR_FL_DIR1_PIN, OUTPUT); pinMode(MOTOR_FL_DIR2_PIN, OUTPUT);
  pinMode(MOTOR_FR_DIR1_PIN, OUTPUT); pinMode(MOTOR_FR_DIR2_PIN, OUTPUT);
  pinMode(MOTOR_RL_DIR1_PIN, OUTPUT); pinMode(MOTOR_RL_DIR2_PIN, OUTPUT);
  pinMode(MOTOR_RR_DIR1_PIN, OUTPUT); pinMode(MOTOR_RR_DIR2_PIN, OUTPUT);

  // Set EN pins as OUTPUT for analogWrite
  pinMode(MOTOR_FL_EN_PIN, OUTPUT);
  pinMode(MOTOR_FR_EN_PIN, OUTPUT);
  pinMode(MOTOR_RL_EN_PIN, OUTPUT);
  pinMode(MOTOR_RR_EN_PIN, OUTPUT);

  // Initialize motors to stopped state (PWM duty 0)
  analogWrite(MOTOR_FL_EN_PIN, 0);
  analogWrite(MOTOR_FR_EN_PIN, 0);
  analogWrite(MOTOR_RL_EN_PIN, 0);
  analogWrite(MOTOR_RR_EN_PIN, 0);

  // Set direction pins to default (brake/coast)
  digitalWrite(MOTOR_FL_DIR1_PIN, LOW); digitalWrite(MOTOR_FL_DIR2_PIN, LOW);
  digitalWrite(MOTOR_FR_DIR1_PIN, LOW); digitalWrite(MOTOR_FR_DIR2_PIN, LOW);
  digitalWrite(MOTOR_RL_DIR1_PIN, LOW); digitalWrite(MOTOR_RL_DIR2_PIN, LOW);
  digitalWrite(MOTOR_RR_DIR1_PIN, LOW); digitalWrite(MOTOR_RR_DIR2_PIN, LOW);

  Serial.println("Motors initialized using basic analogWrite (PWM frequency/resolution may not match defines). UPDATE ESP32 CORE FOR LEDC!");
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

//*****************************************************************************
// ENCODER INTERRUPT SERVICE ROUTINES (ISRs)
//*****************************************************************************
// Adjust logic based on your wiring and desired positive direction count
void IRAM_ATTR encoderISR_FL_A() { // Front Left
  if (digitalRead(MOTOR_FL_ENC_A_PIN) == digitalRead(MOTOR_FL_ENC_B_PIN)) { encoder_fl_count--; } else { encoder_fl_count++; }
}
void IRAM_ATTR encoderISR_FR_A() { // Front Right (often opposite FL for forward motion)
  if (digitalRead(MOTOR_FR_ENC_A_PIN) == digitalRead(MOTOR_FR_ENC_B_PIN)) { encoder_fr_count++; } else { encoder_fr_count--; }
}
void IRAM_ATTR encoderISR_RL_A() { // Rear Left
  if (digitalRead(MOTOR_RL_ENC_A_PIN) == digitalRead(MOTOR_RL_ENC_B_PIN)) { encoder_rl_count--; } else { encoder_rl_count++; }
}
void IRAM_ATTR encoderISR_RR_A() { // Rear Right (often opposite RL for forward motion)
  if (digitalRead(MOTOR_RR_ENC_A_PIN) == digitalRead(MOTOR_RR_ENC_B_PIN)) { encoder_rr_count++; } else { encoder_rr_count--; }
}


//*****************************************************************************
// MOTOR CONTROL FUNCTIONS
//*****************************************************************************

// ***** MODIFIED: Set motor speed using basic analogWrite WORKAROUND *****
// motor_id: Use MOTOR_FL_ID, MOTOR_FR_ID, etc.
// speed_normalized: -1.0 (full reverse) to 1.0 (full forward)
void setMotorSpeed(int motor_id, float speed_normalized) {
  int pwm_value;
  bool forward_direction;
  int dir1_pin, dir2_pin, en_pin;

  speed_normalized = constrain(speed_normalized, -1.0, 1.0);
  forward_direction = (speed_normalized >= 0.0);
  pwm_value = (int)(abs(speed_normalized) * PWM_MAX_DUTY); // PWM_MAX_DUTY is 255
  pwm_value = constrain(pwm_value, 0, PWM_MAX_DUTY);

  // Map motor_id to pins
  switch (motor_id) {
    case MOTOR_FL_ID:
      dir1_pin = MOTOR_FL_DIR1_PIN; dir2_pin = MOTOR_FL_DIR2_PIN; en_pin = MOTOR_FL_EN_PIN; break;
    case MOTOR_FR_ID:
      dir1_pin = MOTOR_FR_DIR1_PIN; dir2_pin = MOTOR_FR_DIR2_PIN; en_pin = MOTOR_FR_EN_PIN; break;
    case MOTOR_RL_ID:
      dir1_pin = MOTOR_RL_DIR1_PIN; dir2_pin = MOTOR_RL_DIR2_PIN; en_pin = MOTOR_RL_EN_PIN; break;
    case MOTOR_RR_ID:
      dir1_pin = MOTOR_RR_DIR1_PIN; dir2_pin = MOTOR_RR_DIR2_PIN; en_pin = MOTOR_RR_EN_PIN; break;
    default: return; // Invalid motor ID
  }

  // Set direction (adjust HIGH/LOW logic if your H-bridge is different)
  digitalWrite(dir1_pin, forward_direction ? HIGH : LOW);
  digitalWrite(dir2_pin, forward_direction ? LOW : HIGH);

  // Set speed using analogWrite
  analogWrite(en_pin, pwm_value);
}

void stopMotors() {
  setMotorSpeed(MOTOR_FL_ID, 0.0);
  setMotorSpeed(MOTOR_FR_ID, 0.0);
  setMotorSpeed(MOTOR_RL_ID, 0.0);
  setMotorSpeed(MOTOR_RR_ID, 0.0);

  target_vel_fl = 0.0; target_vel_fr = 0.0;
  target_vel_rl = 0.0; target_vel_rr = 0.0;

  // Reset PID integral sums on stop to prevent windup issues on restart
  error_sum_fl = 0; error_sum_fr = 0; error_sum_rl = 0; error_sum_rr = 0;
}

//*****************************************************************************
// PID CONTROLLER
//*****************************************************************************
float calculatePID(float target, float current, float& error, float& prev_error, float& error_sum, float dt) {
  if (dt <= 0) return 0; // Avoid division by zero or weird behavior

  error = target - current;

  // Integral term with anti-windup
  error_sum += error * dt;
  if (KI != 0) { // Avoid division by zero
      error_sum = constrain(error_sum, MIN_PID_OUTPUT / KI, MAX_PID_OUTPUT / KI);
  } else {
      error_sum = 0; // No integral action if KI is zero
  }

  // Derivative term
  float error_delta = (error - prev_error) / dt;
  prev_error = error; // Update previous error for next iteration

  // PID output calculation
  float output = (KP * error) + (KI * error_sum) + (KD * error_delta);

  // Clamp output to valid PWM range
  return constrain(output, MIN_PID_OUTPUT, MAX_PID_OUTPUT);
}

//*****************************************************************************
// TIMER CALLBACKS (Control, Odometry, Diagnostics)
//*****************************************************************************
void controlCallback(rcl_timer_t* timer, int64_t last_call_time) {
  if (timer == NULL) return;

  unsigned long now = millis();
  float dt_control = (now - last_control_time) / 1000.0f;
  last_control_time = now;
  // Basic protection against dt being zero or negative after rollover/timing issues
  if (dt_control <= 0) dt_control = 1.0f / CONTROL_FREQUENCY_HZ;

  // --- Calculate Current Wheel Velocities ---
  unsigned long vel_time_now = millis();
  float dt_vel = (vel_time_now -
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
#define MOTOR_RR_ENC_A_PIN 17
#define MOTOR_RR_ENC_B_PIN 3

#define STATUS_LED_PIN 2

// === ROBOT PARAMETERS ===
#define JOINT_COUNT 4
#define WHEEL_RADIUS 0.13           // meters
#define WHEEL_BASE_WIDTH 0.34       // meters
#define ENCODER_PPR 11              // Physical pulses per revolution on motor shaft
#define GEAR_RATIO 1.0              // Gearbox reduction ratio
// Counts per revolution for the wheel after gearing and quadrature decoding (RISING edge on one channel)
#define COUNTS_PER_REVOLUTION (ENCODER_PPR * GEAR_RATIO * 2) 

// IMPROVEMENT: Define physical limits for better control mapping
#define MAX_ROBOT_SPEED_MS 1.0      // TUNABLE: Max linear speed in m/s
#define MAX_WHEEL_SPEED_RADS (MAX_ROBOT_SPEED_MS / WHEEL_RADIUS) // Max wheel angular speed in rad/s

// PWM Configuration
#define PWM_FREQUENCY 5000          // Higher PWM frequency for quieter motors and smoother response
#define PWM_RESOLUTION 8
#define PWM_MAX_DUTY 255
#define PWM_MIN_DUTY 60             // TUNABLE: The minimum PWM required to overcome motor stiction

// Control parameters
#define CONTROL_LOOP_FREQUENCY_HZ 100 // Frequency for publishing states and checking commands
#define DIAGNOSTIC_FREQUENCY_HZ 2     // Frequency for publishing diagnostic data
#define WATCHDOG_TIMEOUT_MS 250       // Reduced timeout for faster stopping

// === GLOBAL VARIABLES ===
rcl_subscription_t twist_subscriber;
rcl_publisher_t joint_state_publisher;
geometry_msgs__msg__Twist twist_msg;
sensor_msgs__msg__JointState joint_state_msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// Encoder data (volatile for ISR safety)
volatile long encoder_counts[JOINT_COUNT] = {0, 0, 0, 0};
long prev_encoder_counts[JOINT_COUNT] = {0, 0, 0, 0};

// Joint state variables
double joint_positions[JOINT_COUNT] = {0.0, 0.0, 0.0, 0.0};
double joint_velocities[JOINT_COUNT] = {0.0, 0.0, 0.0, 0.0};

// Timing variables
unsigned long last_control_time = 0;
unsigned long last_cmd_vel_time = 0;
unsigned long last_diagnostic_time = 0;

bool ros_connected = false;

// === FUNCTION PROTOTYPES ===
void IRAM_ATTR encoderISR_FL();
void IRAM_ATTR encoderISR_FR();
void IRAM_ATTR encoderISR_RL();
void IRAM_ATTR encoderISR_RR();
void setupHardware();
bool setupMicroROS();
void initJointStateMessage(); // Helper for message initialization
void twistCallback(const void* msgin);
void updateAndPublish();
void setMotorPower(int motor_id, float power); // -1.0 to 1.0
void stopAllMotors();
void blinkLED(int times, int duration_ms);


//*****************************************************************************
// ROBUST ENCODER ISRs
//*****************************************************************************
void IRAM_ATTR encoderISR_FL() {
  digitalRead(MOTOR_FL_ENC_B_PIN) ? encoder_counts[0]-- : encoder_counts[0]++;
}

void IRAM_ATTR encoderISR_FR() {
  digitalRead(MOTOR_FR_ENC_B_PIN) ? encoder_counts[1]++ : encoder_counts[1]--;
}

void IRAM_ATTR encoderISR_RL() {
  digitalRead(MOTOR_RL_ENC_B_PIN) ? encoder_counts[2]-- : encoder_counts[2]++;
}

void IRAM_ATTR encoderISR_RR() {
  digitalRead(MOTOR_RR_ENC_B_PIN) ? encoder_counts[3]++ : encoder_counts[3]--;
}

//*****************************************************************************
// SETUP FUNCTION
//*****************************************************************************
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("=== ESP32 4-Motor Robot Controller - Optimized ===");
  setupHardware();
  last_cmd_vel_time = millis();
}

//*****************************************************************************
// MAIN LOOP
//*****************************************************************************
void loop() {
  if (!ros_connected) {
    ros_connected = setupMicroROS();
    if (!ros_connected) {
      stopAllMotors();
      delay(1000);
    } else {
      digitalWrite(STATUS_LED_PIN, HIGH);
    }
  } else {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));
    unsigned long now = millis();
    if (now - last_control_time >= (1000 / CONTROL_LOOP_FREQUENCY_HZ)) {
      last_control_time = now;
      updateAndPublish();
    }
    if (now - last_cmd_vel_time > WATCHDOG_TIMEOUT_MS) {
      stopAllMotors();
    }
  }
}

//*****************************************************************************
// HARDWARE & ROS INITIALIZATION
//*****************************************************************************
void setupHardware() {
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LOW);

  pinMode(MOTOR_FL_DIR1_PIN, OUTPUT); pinMode(MOTOR_FL_DIR2_PIN, OUTPUT);
  pinMode(MOTOR_FR_DIR1_PIN, OUTPUT); pinMode(MOTOR_FR_DIR2_PIN, OUTPUT);
  pinMode(MOTOR_RL_DIR1_PIN, OUTPUT); pinMode(MOTOR_RL_DIR2_PIN, OUTPUT);
  pinMode(MOTOR_RR_DIR1_PIN, OUTPUT); pinMode(MOTOR_RR_DIR2_PIN, OUTPUT);

  ledcAttach(MOTOR_FL_EN_PIN, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttach(MOTOR_FR_EN_PIN, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttach(MOTOR_RL_EN_PIN, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttach(MOTOR_RR_EN_PIN, PWM_FREQUENCY, PWM_RESOLUTION);
  
  stopAllMotors();
  Serial.printf("Motors initialized with %d Hz PWM.\n", PWM_FREQUENCY);

  pinMode(MOTOR_FL_ENC_A_PIN, INPUT_PULLUP); pinMode(MOTOR_FL_ENC_B_PIN, INPUT_PULLUP);
  pinMode(MOTOR_FR_ENC_A_PIN, INPUT_PULLUP); pinMode(MOTOR_FR_ENC_B_PIN, INPUT_PULLUP);
  pinMode(MOTOR_RL_ENC_A_PIN, INPUT_PULLUP); pinMode(MOTOR_RL_ENC_B_PIN, INPUT_PULLUP);
  pinMode(MOTOR_RR_ENC_A_PIN, INPUT_PULLUP); pinMode(MOTOR_RR_ENC_B_PIN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(MOTOR_FL_ENC_A_PIN), encoderISR_FL, RISING);
  attachInterrupt(digitalPinToInterrupt(MOTOR_FR_ENC_A_PIN), encoderISR_FR, RISING);
  attachInterrupt(digitalPinToInterrupt(MOTOR_RL_ENC_A_PIN), encoderISR_RL, RISING);
  attachInterrupt(digitalPinToInterrupt(MOTOR_RR_ENC_A_PIN), encoderISR_RR, RISING);
  
  Serial.printf("Encoders initialized (%d counts/rev).\n", COUNTS_PER_REVOLUTION);
  blinkLED(2, 100);
}

bool setupMicroROS() {
  Serial.println("Attempting to connect to micro-ROS agent...");
#ifdef USE_SERIAL_TRANSPORT
  set_microros_transports();
#endif
#ifdef USE_WIFI_TRANSPORT
  WiFi.begin(ssid, password);
  unsigned long start_time = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start_time < 10000) {
    delay(500); Serial.print(".");
  }
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("\nWiFi connection failed."); return false;
  }
  Serial.printf("\nWiFi connected! IP: %s\n", WiFi.localIP().toString().c_str());
  set_microros_wifi_transports((char*)ssid, (char*)password, (char*)agent_ip, agent_port);
#endif

  allocator = rcl_get_default_allocator();
  if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) return false;
  if (rclc_node_init_default(&node, "esp32_hardware_interface", "", &support) != RCL_RET_OK) return false;

  if (rclc_subscription_init_default(&twist_subscriber, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel") != RCL_RET_OK) return false;
  if (rclc_publisher_init_default(&joint_state_publisher, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState), "joint_states") != RCL_RET_OK) return false;
  if (rclc_executor_init(&executor, &support.context, 1, &allocator) != RCL_RET_OK) return false;
  if (rclc_executor_add_subscription(&executor, &twist_subscriber, &twist_msg, &twistCallback, ON_NEW_DATA) != RCL_RET_OK) return false;
  
  initJointStateMessage();
  Serial.println("micro-ROS connection successful.");
  return true;
}

void initJointStateMessage() {
  static bool initialized = false;
  if (initialized) return;

  const char* joint_names[JOINT_COUNT] = {
    "front_left_wheel_joint", "front_right_wheel_joint", 
    "rear_left_wheel_joint", "rear_right_wheel_joint"
  };

  sensor_msgs__msg__JointState__init(&joint_state_msg);
  joint_state_msg.name.capacity = JOINT_COUNT;
  joint_state_msg.name.size = JOINT_COUNT;
  joint_state_msg.name.data = (rosidl_runtime_c__String*)malloc(JOINT_COUNT * sizeof(rosidl_runtime_c__String));

  for (int i = 0; i < JOINT_COUNT; i++) {
    // FIX: The problematic ...__String__init call is removed.
    // We manually populate the struct members directly.
    size_t len = strlen(joint_names[i]);
    joint_state_msg.name.data[i].data = (char*)malloc((len + 1) * sizeof(char));
    strcpy(joint_state_msg.name.data[i].data, joint_names[i]);
    joint_state_msg.name.data[i].size = len;
    joint_state_msg.name.data[i].capacity = len + 1;
  }

  joint_state_msg.position.data = joint_positions;
  joint_state_msg.position.size = JOINT_COUNT;
  joint_state_msg.position.capacity = JOINT_COUNT;
  
  joint_state_msg.velocity.data = joint_velocities;
  joint_state_msg.velocity.size = JOINT_COUNT;
  joint_state_msg.velocity.capacity = JOINT_COUNT;
  
  joint_state_msg.effort.data = NULL;
  joint_state_msg.effort.size = 0;
  joint_state_msg.effort.capacity = 0;

  initialized = true;
  Serial.println("JointState message memory initialized.");
}

//*****************************************************************************
// CONTROL AND STATE FUNCTIONS
//*****************************************************************************
void updateAndPublish() {
  static unsigned long prev_time = 0;
  if (prev_time == 0) { prev_time = millis(); return; }
  
  unsigned long now = millis();
  float dt = (now - prev_time) / 1000.0f;
  prev_time = now;
  
  long current_counts[JOINT_COUNT];
  noInterrupts();
  for (int i = 0; i < JOINT_COUNT; i++) { current_counts[i] = encoder_counts[i]; }
  interrupts();
  
  for (int i = 0; i < JOINT_COUNT; i++) {
    long count_diff = current_counts[i] - prev_encoder_counts[i];
    prev_encoder_counts[i] = current_counts[i];
    
    joint_positions[i] = (double)current_counts[i] * (2.0 * PI / COUNTS_PER_REVOLUTION);
    double velocity_rad_s = (double)count_diff * (2.0 * PI / COUNTS_PER_REVOLUTION) / dt;
    joint_velocities[i] = joint_velocities[i] * 0.8 + velocity_rad_s * 0.2;
  }
  
  joint_state_msg.header.stamp.sec = now / 1000;
  joint_state_msg.header.stamp.nanosec = (now % 1000) * 1000000;
  rcl_publish(&joint_state_publisher, &joint_state_msg, NULL);
}

void twistCallback(const void* msgin) {
  const geometry_msgs__msg__Twist* msg = (const geometry_msgs__msg__Twist*)msgin;
  
  float linear_x = msg->linear.x;
  float angular_z = msg->angular.z;

  float left_wheel_vel_rads = (linear_x - angular_z * WHEEL_BASE_WIDTH / 2.0) / WHEEL_RADIUS;
  float right_wheel_vel_rads = (linear_x + angular_z * WHEEL_BASE_WIDTH / 2.0) / WHEEL_RADIUS;
  float left_power = left_wheel_vel_rads / MAX_WHEEL_SPEED_RADS;
  float right_power = right_wheel_vel_rads / MAX_WHEEL_SPEED_RADS;

  setMotorPower(0, left_power);
  setMotorPower(1, right_power);
  setMotorPower(2, left_power);
  setMotorPower(3, right_power);
  
  last_cmd_vel_time = millis();
}

void setMotorPower(int motor_id, float power) {
  power = constrain(power, -1.0, 1.0);
  bool forward = (power >= 0);
  int pwm_value;
  
  if (abs(power) < 0.01) {
    pwm_value = 0;
  } else {
    pwm_value = map(abs(power) * 100, 0, 100, PWM_MIN_DUTY, PWM_MAX_DUTY);
  }

  switch (motor_id) {
    case 0: // FL
      digitalWrite(MOTOR_FL_DIR1_PIN, forward ? HIGH : LOW);
      digitalWrite(MOTOR_FL_DIR2_PIN, forward ? LOW : HIGH);
      ledcWrite(MOTOR_FL_EN_PIN, pwm_value);
      break;
    case 1: // FR
      digitalWrite(MOTOR_FR_DIR1_PIN, forward ? HIGH : LOW);
      digitalWrite(MOTOR_FR_DIR2_PIN, forward ? LOW : HIGH);
      ledcWrite(MOTOR_FR_EN_PIN, pwm_value);
      break;
    case 2: // RL
      digitalWrite(MOTOR_RL_DIR1_PIN, forward ? HIGH : LOW);
      digitalWrite(MOTOR_RL_DIR2_PIN, forward ? LOW : HIGH);
      ledcWrite(MOTOR_RL_EN_PIN, pwm_value);
      break;
    case 3: // RR
      digitalWrite(MOTOR_RR_DIR1_PIN, forward ? HIGH : LOW);
      digitalWrite(MOTOR_RR_DIR2_PIN, forward ? LOW : HIGH);
      ledcWrite(MOTOR_RR_EN_PIN, pwm_value);
      break;
  }
}

void stopAllMotors() {
  for (int i = 0; i < JOINT_COUNT; i++) {
    setMotorPower(i, 0.0);
  }
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
#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <sensor_msgs/msg/joint_state.h>
#include <sensor_msgs/msg/imu.h>
#include <nav_msgs/msg/odometry.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/string.h>

// IMU Library
#include <Wire.h>
#include <MPU6050.h>

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

// IMU pins (I2C)
#define IMU_SDA_PIN 21
#define IMU_SCL_PIN 22

// === OPTIMIZED ROBOT PARAMETERS ===
#define JOINT_COUNT 4
#define WHEEL_RADIUS 0.05           // meters - CRITICAL: Calibrate this!
#define WHEEL_BASE_WIDTH 0.34       // meters - CRITICAL: Measure this!
#define ENCODER_PPR 11              // Physical pulses per revolution
#define GEAR_RATIO 1.0              // Gearbox reduction ratio

// OPTIMIZED PWM Configuration for smoother control
#define PWM_FREQUENCY 20000         // 20 kHz - Higher frequency for smoother motion
#define PWM_RESOLUTION 8            // 8-bit resolution
#define PWM_MAX_DUTY 255

// OPTIMIZED Control parameters
#define JOINT_STATE_FREQUENCY_HZ 100 // INCREASED to 100Hz for better control
#define IMU_FREQUENCY_HZ 100         // IMU update frequency
#define ODOMETRY_FREQUENCY_HZ 50     // Odometry publishing frequency
#define DIAGNOSTIC_FREQUENCY_HZ 5    // Increased diagnostic frequency
#define WATCHDOG_TIMEOUT_MS 200      // REDUCED timeout for faster response

// OPTIMIZED Speed scaling and response
#define MAX_SPEED_SCALE 1.0f         // Full speed available
#define MIN_PWM_THRESHOLD 30         // Minimum PWM to overcome motor dead zone
#define ACCELERATION_LIMIT 0.1f      // Smooth acceleration limiting

// IMU and Odometry parameters
#define IMU_CALIBRATION_SAMPLES 1000 // Number of samples for gyro calibration
#define GYRO_ALPHA 0.98f             // Complementary filter coefficient
#define ACCEL_ALPHA 0.02f            // Complementary filter coefficient
#define YAW_DRIFT_CORRECTION 0.999f  // Yaw drift correction factor

// === GLOBAL VARIABLES ===
// Encoder data (volatile for ISR safety)
volatile long encoder_counts[JOINT_COUNT] = {0, 0, 0, 0};
long encoder_prev[JOINT_COUNT] = {0, 0, 0, 0};

// Joint state variables with filtering
double joint_positions[JOINT_COUNT] = {0.0, 0.0, 0.0, 0.0};
double joint_velocities[JOINT_COUNT] = {0.0, 0.0, 0.0, 0.0};
double joint_efforts[JOINT_COUNT] = {0.0, 0.0, 0.0, 0.0};

// OPTIMIZED Motor control with smoothing
float motor_speeds[JOINT_COUNT] = {0.0, 0.0, 0.0, 0.0};
float target_motor_speeds[JOINT_COUNT] = {0.0, 0.0, 0.0, 0.0};
float previous_motor_speeds[JOINT_COUNT] = {0.0, 0.0, 0.0, 0.0};

// Velocity filtering for smooth motion
float velocity_filter_alpha = 0.7f;  // Low-pass filter coefficient

// IMU variables
MPU6050 mpu;
float gyro_offset[3] = {0.0, 0.0, 0.0};  // Gyro calibration offsets
float accel_offset[3] = {0.0, 0.0, 0.0}; // Accel calibration offsets
bool imu_initialized = false;

// IMU raw data
int16_t ax, ay, az;
int16_t gx, gy, gz;

// Processed IMU data
float accel_x, accel_y, accel_z;
float gyro_x, gyro_y, gyro_z;
float roll, pitch, yaw = 0.0;
float roll_accel, pitch_accel;

// Odometry variables
float robot_x = 0.0, robot_y = 0.0, robot_theta = 0.0;
float robot_vx = 0.0, robot_vy = 0.0, robot_vtheta = 0.0;
float wheel_left_pos = 0.0, wheel_right_pos = 0.0;
float wheel_left_pos_prev = 0.0, wheel_right_pos_prev = 0.0;

// Timing variables with microsecond precision
unsigned long last_joint_state_time = 0;
unsigned long last_imu_time = 0;
unsigned long last_odometry_time = 0;
unsigned long last_cmd_time = 0;
unsigned long last_diagnostic_time = 0;
unsigned long last_encoder_time = 0;
unsigned long last_control_time = 0;

// Connection management
bool ros_connected = false;
int connection_attempts = 0;

// micro-ROS entities
rcl_subscription_t twist_subscriber;
rcl_publisher_t joint_state_publisher;
rcl_publisher_t imu_publisher;
rcl_publisher_t odometry_publisher;
rcl_publisher_t diagnostic_publisher;

// Messages
geometry_msgs__msg__Twist twist_msg;
sensor_msgs__msg__JointState joint_state_msg;
sensor_msgs__msg__Imu imu_msg;
nav_msgs__msg__Odometry odometry_msg;
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

char diagnostic_buffer[400];
bool joint_state_memory_initialized = false;
bool imu_memory_initialized = false;
bool odometry_memory_initialized = false;

// Performance monitoring
unsigned long loop_count = 0;
unsigned long max_loop_time = 0;

// === FUNCTION PROTOTYPES ===
void IRAM_ATTR encoderISR_FL();
void IRAM_ATTR encoderISR_FR(); 
void IRAM_ATTR encoderISR_RL();
void IRAM_ATTR encoderISR_RR();

void setupHardware();
void setupMotors();
void setupEncoders();
void setupIMU();
void setupMicroROS();
bool initMicroROS();
void cleanupMicroROS();

void calibrateIMU();
void updateIMU();
void updateJointStates();
void updateOdometry();
void smoothMotorControl();
void setMotorSpeed(int motor_id, float speed_normalized);
void stopAllMotors();

void twistCallback(const void* msgin);
void publishJointStates();
void publishIMU();
void publishOdometry();
void publishDiagnostics();

void blinkLED(int times, int duration_ms);

//*****************************************************************************
// OPTIMIZED ENCODER ISRs - Using both A and B channels for accuracy
//*****************************************************************************
void IRAM_ATTR encoderISR_FL() {
  bool a = digitalRead(MOTOR_FL_ENC_A_PIN);
  bool b = digitalRead(MOTOR_FL_ENC_B_PIN);
  
  if (a == b) {
    encoder_counts[0]++;
  } else {
    encoder_counts[0]--;
  }
}

void IRAM_ATTR encoderISR_FR() {
  bool a = digitalRead(MOTOR_FR_ENC_A_PIN);
  bool b = digitalRead(MOTOR_FR_ENC_B_PIN);
  
  if (a == b) {
    encoder_counts[1]++;
  } else {
    encoder_counts[1]--;
  }
}

void IRAM_ATTR encoderISR_RL() {
  bool a = digitalRead(MOTOR_RL_ENC_A_PIN);
  bool b = digitalRead(MOTOR_RL_ENC_B_PIN);
  
  if (a == b) {
    encoder_counts[2]++;
  } else {
    encoder_counts[2]--;
  }
}

void IRAM_ATTR encoderISR_RR() {
  bool a = digitalRead(MOTOR_RR_ENC_A_PIN);
  bool b = digitalRead(MOTOR_RR_ENC_B_PIN);
  
  if (a == b) {
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
  Serial.println("=== ESP32 Optimized 4-Motor Controller with IMU ===");
  Serial.println("Initializing hardware for smooth motion and odometry...");

  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LOW);

  setupHardware();
  setupMicroROS();

  last_joint_state_time = micros();
  last_imu_time = micros();
  last_odometry_time = micros();
  last_cmd_time = millis();
  last_diagnostic_time = millis();
  last_encoder_time = micros();
  last_control_time = micros();

  blinkLED(3, 200);
  Serial.println("Setup complete. Attempting ROS connection...");
}

//*****************************************************************************
// OPTIMIZED MAIN LOOP
//*****************************************************************************
void loop() {
  unsigned long loop_start = micros();
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
        Serial.println("Optimized controller with IMU connected!");
        digitalWrite(STATUS_LED_PIN, HIGH);
        connection_attempts = 0;
      } else {
        Serial.println("Connection failed, retrying...");
        blinkLED(2, 100);
      }
    }
  }

  if (ros_connected) {
    // Process ROS callbacks with minimal blocking
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(0));
    
    // Safety watchdog with faster response
    if (current_time - last_cmd_time > WATCHDOG_TIMEOUT_MS) {
      for (int i = 0; i < JOINT_COUNT; i++) {
        target_motor_speeds[i] = 0.0f;
      }
    }
    
    // High-frequency motor control for smooth motion
    unsigned long control_interval = 1000000 / (JOINT_STATE_FREQUENCY_HZ * 2); // 200Hz control
    if (micros() - last_control_time >= control_interval) {
      smoothMotorControl();
      last_control_time = micros();
    }
    
    // Update IMU at high frequency
    if (micros() - last_imu_time >= (1000000 / IMU_FREQUENCY_HZ)) {
      updateIMU();
      publishIMU();
      last_imu_time = micros();
    }
    
    // Update joint states at high frequency
    if (micros() - last_joint_state_time >= (1000000 / JOINT_STATE_FREQUENCY_HZ)) {
      updateJointStates();
      publishJointStates();
      last_joint_state_time = micros();
    }
    
    // Update and publish odometry
    if (micros() - last_odometry_time >= (1000000 / ODOMETRY_FREQUENCY_HZ)) {
      updateOdometry();
      publishOdometry();
      last_odometry_time = micros();
    }
    
    // Publish diagnostics
    if (current_time - last_diagnostic_time >= (1000 / DIAGNOSTIC_FREQUENCY_HZ)) {
      publishDiagnostics();
      last_diagnostic_time = current_time;
    }
  } else {
    // If not connected, stop all motors for safety
    stopAllMotors();
  }

  // Performance monitoring
  loop_count++;
  unsigned long loop_time = micros() - loop_start;
  if (loop_time > max_loop_time) {
    max_loop_time = loop_time;
  }

  // Minimal delay to prevent watchdog reset
  delayMicroseconds(100);
}

//*****************************************************************************
// HARDWARE SETUP
//*****************************************************************************
void setupHardware() {
  setupMotors();
  setupEncoders();
  setupIMU();
  Serial.println("Optimized hardware initialization complete.");
}

void setupMotors() {
  // Configure direction pins
  pinMode(MOTOR_FL_DIR1_PIN, OUTPUT); pinMode(MOTOR_FL_DIR2_PIN, OUTPUT);
  pinMode(MOTOR_FR_DIR1_PIN, OUTPUT); pinMode(MOTOR_FR_DIR2_PIN, OUTPUT);
  pinMode(MOTOR_RL_DIR1_PIN, OUTPUT); pinMode(MOTOR_RL_DIR2_PIN, OUTPUT);
  pinMode(MOTOR_RR_DIR1_PIN, OUTPUT); pinMode(MOTOR_RR_DIR2_PIN, OUTPUT);

  // Setup HIGH FREQUENCY PWM for smoother control
  ledcAttach(MOTOR_FL_EN_PIN, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttach(MOTOR_FR_EN_PIN, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttach(MOTOR_RL_EN_PIN, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttach(MOTOR_RR_EN_PIN, PWM_FREQUENCY, PWM_RESOLUTION);

  stopAllMotors();
  Serial.printf("Motors initialized with %d Hz PWM for smooth control\n", PWM_FREQUENCY);
}

void setupEncoders() {
  // Configure encoder pins with pullups
  pinMode(MOTOR_FL_ENC_A_PIN, INPUT_PULLUP); pinMode(MOTOR_FL_ENC_B_PIN, INPUT_PULLUP);
  pinMode(MOTOR_FR_ENC_A_PIN, INPUT_PULLUP); pinMode(MOTOR_FR_ENC_B_PIN, INPUT_PULLUP);
  pinMode(MOTOR_RL_ENC_A_PIN, INPUT_PULLUP); pinMode(MOTOR_RL_ENC_B_PIN, INPUT_PULLUP);
  pinMode(MOTOR_RR_ENC_A_PIN, INPUT_PULLUP); pinMode(MOTOR_RR_ENC_B_PIN, INPUT_PULLUP);

  // Attach interrupts on BOTH edges for maximum resolution
  attachInterrupt(digitalPinToInterrupt(MOTOR_FL_ENC_A_PIN), encoderISR_FL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTOR_FR_ENC_A_PIN), encoderISR_FR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTOR_RL_ENC_A_PIN), encoderISR_RL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTOR_RR_ENC_A_PIN), encoderISR_RR, CHANGE);

  Serial.printf("High-resolution encoders initialized (%d PPR)\n", ENCODER_PPR);
}

void setupIMU() {
  Wire.begin(IMU_SDA_PIN, IMU_SCL_PIN);
  Wire.setClock(400000); // 400kHz I2C for faster communication
  
  Serial.println("Initializing MPU6050...");
  mpu.initialize();

  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed!");
    imu_initialized = false;
    return;
  }

  Serial.println("MPU6050 connected successfully");
  
  // Configure MPU6050 for optimal performance
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_1000);    // ±1000°/s
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_4);     // ±4g
  mpu.setDLPFMode(MPU6050_DLPF_BW_42);                // 42Hz low-pass filter

  Serial.println("Calibrating IMU... Keep robot stationary!");
  calibrateIMU();
  
  imu_initialized = true;
  Serial.println("IMU initialization complete");
}

void calibrateIMU() {
  float gx_sum = 0, gy_sum = 0, gz_sum = 0;
  float ax_sum = 0, ay_sum = 0, az_sum = 0;
  
  for (int i = 0; i < IMU_CALIBRATION_SAMPLES; i++) {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    
    gx_sum += gx;
    gy_sum += gy;
    gz_sum += gz;
    ax_sum += ax;
    ay_sum += ay;
    az_sum += az;
    
    if (i % 100 == 0) {
      Serial.printf("Calibration progress: %d%%\n", (i * 100) / IMU_CALIBRATION_SAMPLES);
    }
    delay(2);
  }
  
  gyro_offset[0] = gx_sum / IMU_CALIBRATION_SAMPLES;
  gyro_offset[1] = gy_sum / IMU_CALIBRATION_SAMPLES;
  gyro_offset[2] = gz_sum / IMU_CALIBRATION_SAMPLES;
  
  // Accel offset (assuming robot is level, az should be ~16384 for 1g)
  accel_offset[0] = ax_sum / IMU_CALIBRATION_SAMPLES;
  accel_offset[1] = ay_sum / IMU_CALIBRATION_SAMPLES;
  accel_offset[2] = (az_sum / IMU_CALIBRATION_SAMPLES) - 16384; // Remove 1g
  
  Serial.printf("Gyro offsets: X=%.2f, Y=%.2f, Z=%.2f\n", 
                gyro_offset[0], gyro_offset[1], gyro_offset[2]);
  Serial.printf("Accel offsets: X=%.2f, Y=%.2f, Z=%.2f\n", 
                accel_offset[0], accel_offset[1], accel_offset[2]);
}

//*****************************************************************************
// MICRO-ROS SETUP
//*****************************************************************************
void setupMicroROS() {
  set_microros_transports();
  Serial.println("micro-ROS optimized transport configured");
}

bool initMicroROS() {
  allocator = rcl_get_default_allocator();

  if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) {
    Serial.println("Failed to initialize RCL support");
    return false;
  }

  if (rclc_node_init_default(&node, "esp32_optimized_controller", "", &support) != RCL_RET_OK) {
    Serial.println("Failed to initialize RCL node");
    return false;
  }

  // Initialize joint state message (only once)
  if (!joint_state_memory_initialized) {
    joint_state_msg.header.frame_id.data = (char*)"base_link";
    joint_state_msg.header.frame_id.size = strlen("base_link");
    joint_state_msg.header.frame_id.capacity = strlen("base_link") + 1;

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

  // Initialize IMU message
  if (!imu_memory_initialized) {
    imu_msg.header.frame_id.data = (char*)"imu_link";
    imu_msg.header.frame_id.size = strlen("imu_link");
    imu_msg.header.frame_id.capacity = strlen("imu_link") + 1;
    
    // Set covariance matrices (diagonal values for now)
    for (int i = 0; i < 9; i++) {
      imu_msg.orientation_covariance[i] = (i % 4 == 0) ? 0.01 : 0.0;
      imu_msg.angular_velocity_covariance[i] = (i % 4 == 0) ? 0.01 : 0.0;
      imu_msg.linear_acceleration_covariance[i] = (i % 4 == 0) ? 0.01 : 0.0;
    }
    
    imu_memory_initialized = true;
  }

  // Initialize odometry message
  if (!odometry_memory_initialized) {
    odometry_msg.header.frame_id.data = (char*)"odom";
    odometry_msg.header.frame_id.size = strlen("odom");
    odometry_msg.header.frame_id.capacity = strlen("odom") + 1;
    
    odometry_msg.child_frame_id.data = (char*)"base_link";
    odometry_msg.child_frame_id.size = strlen("base_link");
    odometry_msg.child_frame_id.capacity = strlen("base_link") + 1;
    
    // Set covariance matrices
    for (int i = 0; i < 36; i++) {
      odometry_msg.pose.covariance[i] = (i % 7 == 0) ? 0.1 : 0.0;
      odometry_msg.twist.covariance[i] = (i % 7 == 0) ? 0.1 : 0.0;
    }
    
    odometry_memory_initialized = true;
  }

  diagnostic_msg.data.capacity = sizeof(diagnostic_buffer);
  diagnostic_msg.data.size = 0;
  diagnostic_msg.data.data = diagnostic_buffer;

  // Create subscribers and publishers
  if (rclc_subscription_init_default(&twist_subscriber, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel") != RCL_RET_OK) {
    Serial.println("Failed to create twist subscriber");
    return false;
  }

  if (rclc_publisher_init_default(&joint_state_publisher, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState), "joint_states") != RCL_RET_OK) {
    Serial.println("Failed to create joint state publisher");
    return false;
  }

  if (rclc_publisher_init_default(&imu_publisher, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "imu/data") != RCL_RET_OK) {
    Serial.println("Failed to create IMU publisher");
    return false;
  }

  if (rclc_publisher_init_default(&odometry_publisher, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "odom") != RCL_RET_OK) {
    Serial.println("Failed to create odometry publisher");
    return false;
  }

  if (rclc_publisher_init_default(&diagnostic_publisher, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "esp32/diagnostics") != RCL_RET_OK) {
    Serial.println("Failed to create diagnostic publisher");
    return false;
  }

  if (rclc_executor_init(&executor, &support.context, 1, &allocator) != RCL_RET_OK) {
    Serial.println("Failed to create executor");
    return false;
  }

  rclc_executor_add_subscription(&executor, &twist_subscriber, &twist_msg, &twistCallback, ON_NEW_DATA);

  Serial.println("Optimized ROS2 interface with IMU and odometry initialized");
  return true;
}

void cleanupMicroROS() {
  if (ros_connected) {
    rclc_executor_fini(&executor);
    rcl_publisher_fini(&diagnostic_publisher, &node);
    rcl_publisher_fini(&odometry_publisher, &node);
    rcl_publisher_fini(&imu_publisher, &node);
    rcl_publisher_fini(&joint_state_publisher, &node);
    rcl_subscription_fini(&twist_subscriber, &node);
    rcl_node_fini(&node);
    rclc_support_fini(&support);
    ros_connected = false;
  }
}

//*****************************************************************************
// IMU AND ODOMETRY FUNCTIONS
//*****************************************************************************
void updateIMU() {
  if (!imu_initialized) return;
  
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  // Apply calibration offsets
  accel_x = (ax - accel_offset[0]) / 16384.0; // Convert to g
  accel_y = (ay - accel_offset[1]) / 16384.0;
  accel_z = (az - accel_offset[2]) / 16384.0;
  
  gyro_x = (gx - gyro_offset[0]) / 131.0; // Convert to deg/s, then to rad/s
  gyro_y = (gy - gyro_offset[1]) / 131.0;
  gyro_z = (gz - gyro_offset[2]) / 131.0;
  
  // Convert to rad/s
  gyro_x *= PI / 180.0;
  gyro_y *= PI / 180.0;
  gyro_z *= PI / 180.0;
  
  // Calculate roll and pitch from accelerometer
  roll_accel = atan2(accel_y, accel_z);
  pitch_accel = atan2(-accel_x, sqrt(accel_y * accel_y + accel_z * accel_z));
  
  // Complementary filter for roll and pitch
  float dt = 1.0f / IMU_FREQUENCY_HZ;
  roll = GYRO_ALPHA * (roll + gyro_x * dt) + ACCEL_ALPHA * roll_accel;
  pitch = GYRO_ALPHA * (pitch + gyro_y * dt) + ACCEL_ALPHA * pitch_accel;
  
  // Integrate yaw from gyroscope (with drift correction)
  yaw += gyro_z * dt;
  yaw *= YAW_DRIFT_CORRECTION; // Apply small drift correction
}

//*****************************************************************************
// OPTIMIZED CONTROL FUNCTIONS
//*****************************************************************************
void updateJointStates() {
  unsigned long now = micros();
  float dt = (now - last_encoder_time) / 1000000.0f;
  
  if (dt <= 0 || dt > 0.02f) {
    dt = 1.0f / JOINT_STATE_FREQUENCY_HZ;
  }
  
  last_encoder_time = now;

  for (int i = 0; i < JOINT_COUNT; i++) {
    long current_counts;
    
    // Atomic read of encoder counts
    noInterrupts();
    current_counts = encoder_counts[i];
    interrupts();
    
    long count_diff = current_counts - encoder_prev[i];
    encoder_prev[i] = current_counts;
    
    // Convert encoder counts to radians with proper scaling
    double position_increment = (double)count_diff * 2.0 * PI / (ENCODER_PPR * GEAR_RATIO);
    joint_positions[i] += position_increment;
    
    // Calculate velocity with filtering for smooth readings
    if (dt > 0) {
      double raw_velocity = position_increment / dt;
      // Apply low-pass filter to reduce noise
      joint_velocities[i] = velocity_filter_alpha * joint_velocities[i] + (1.0 - velocity_filter_alpha) * raw_velocity;
    }
    
    // Effort represents current motor command
    joint_efforts[i] = motor_speeds[i];
  }
}

void smoothMotorControl() {
  // Apply smooth acceleration limiting to prevent jerky motion
  for (int i = 0; i < JOINT_COUNT; i++) {
    float speed_diff = target_motor_speeds[i] - motor_speeds[i];
    
    // Limit acceleration
    if (fabs(speed_diff) > ACCELERATION_LIMIT) {
      if (speed_diff > 0) {
        motor_speeds[i] += ACCELERATION_LIMIT;
      } else {
        motor_speeds[i] -= ACCELERATION_LIMIT;
      }
    } else {
      motor_speeds[i] = target_motor_speeds[i];
    }
    
    // Apply motor speed if it changed significantly
    if (fabs(motor_speeds[i] - previous_motor_speeds[i]) > 0.01f) {
      setMotorSpeed(i, motor_speeds[i]);
      previous_motor_speeds[i] = motor_speeds[i];
    }
  }
}

void setMotorSpeed(int motor_id, float speed_normalized) {
  // Limit speed and apply scaling
  speed_normalized = constrain(speed_normalized * MAX_SPEED_SCALE, -1.0f, 1.0f);
  
  // Convert to PWM value with dead zone compensation
  int pwm_value = 0;
  if (fabs(speed_normalized) > 0.01f) { // Only move if command is significant
    pwm_value = (int)(fabs(speed_normalized) * (PWM_MAX_DUTY - MIN_PWM_THRESHOLD) + MIN_PWM_THRESHOLD);
    pwm_value = constrain(pwm_value, MIN_PWM_THRESHOLD, PWM_MAX_DUTY);
  }
  
  bool forward = (speed_normalized >= 0.0f);
  
  // Set direction and PWM with optimized timing
  switch (motor_id) {
    case 0: // Front Left
      digitalWrite(MOTOR_FL_DIR1_PIN, forward ? HIGH : LOW);
      digitalWrite(MOTOR_FL_DIR2_PIN, forward ? LOW : HIGH);
      ledcWrite(MOTOR_FL_EN_PIN, pwm_value);
      break;
      
    case 1: // Front Right
      digitalWrite(MOTOR_FR_DIR1_PIN, forward ? HIGH : LOW);
      digitalWrite(MOTOR_FR_DIR2_PIN, forward ? LOW : HIGH);
      ledcWrite(MOTOR_FR_EN_PIN, pwm_value);
      break;
      
    case 2: // Rear Left
      digitalWrite(MOTOR_RL_DIR1_PIN, forward ? HIGH : LOW);
      digitalWrite(MOTOR_RL_DIR2_PIN, forward ? LOW : HIGH);
      ledcWrite(MOTOR_RL_EN_PIN, pwm_value);
      break;
      
    case 3: // Rear Right
      digitalWrite(MOTOR_RR_DIR1_PIN, forward ? HIGH : LOW);
      digitalWrite(MOTOR_RR_DIR2_PIN, forward ? LOW : HIGH);
      ledcWrite(MOTOR_RR_EN_PIN, pwm_value);
      break;
  }
}

void stopAllMotors() {
  for (int i = 0; i < JOINT_COUNT; i++) {
    target_motor_speeds[i] = 0.0f;
    motor_speeds[i] = 0.0f;
    setMotorSpeed(i, 0.0f);
  }
}

//*****************************************************************************
// ROS CALLBACKS AND PUBLISHERS
//*****************************************************************************
void twistCallback(const void* msgin) {
  const geometry_msgs__msg__Twist* msg = (const geometry_msgs__msg__Twist*)msgin;
  
  float linear_x = msg->linear.x;
  float angular_z = msg->angular.z;
  
  // Convert twist to wheel velocities using PRECISE differential drive kinematics
  float velocity_scale = 1.0f / WHEEL_RADIUS;
  
  float left_vel = (linear_x - angular_z * WHEEL_BASE_WIDTH / 2.0) * velocity_scale;
  float right_vel = (linear_x + angular_z * WHEEL_BASE_WIDTH / 2.0) * velocity_scale;
  
  // Normalize to motor speed range with better scaling
  const float MAX_WHEEL_SPEED = 8.0f; // Reduced for more precise control
  left_vel = constrain(left_vel / MAX_WHEEL_SPEED, -1.0f, 1.0f);
  right_vel = constrain(right_vel / MAX_WHEEL_SPEED, -1.0f, 1.0f);
  
  // Set TARGET speeds for smooth acceleration
  target_motor_speeds[0] = left_vel;   // Front left
  target_motor_speeds[1] = right_vel;  // Front right
  target_motor_speeds[2] = left_vel;   // Rear left
  target_motor_speeds[3] = right_vel;  // Rear right
  
  last_cmd_time = millis();
  
  // Reduced debug output to prevent serial buffer overflow
  if (loop_count % 100 == 0) {
    Serial.printf("Cmd: L=%.2f R=%.2f\n", left_vel, right_vel);
  }
}

void publishJointStates() {
  if (!ros_connected) return;
  
  // Update timestamp with microsecond precision
  int64_t time_ns = rmw_uros_epoch_nanos();
  joint_state_msg.header.stamp.sec = time_ns / 1000000000LL;
  joint_state_msg.header.stamp.nanosec = time_ns % 1000000000LL;
  
  rcl_ret_t ret = rcl_publish(&joint_state_publisher, &joint_state_msg, NULL);
  if (ret != RCL_RET_OK && loop_count % 1000 == 0) {
    Serial.println("Joint state publish failed");
  }
}

void publishIMU() {
  if (!ros_connected || !imu_initialized) return;
  
  // Update timestamp
  int64_t time_ns = rmw_uros_epoch_nanos();
  imu_msg.header.stamp.sec = time_ns / 1000000000LL;
  imu_msg.header.stamp.nanosec = time_ns % 1000000000LL;
  
  // Convert Euler angles to quaternion
  float cy = cos(yaw * 0.5);
  float sy = sin(yaw * 0.5);
  float cp = cos(pitch * 0.5);
  float sp = sin(pitch * 0.5);
  float cr = cos(roll * 0.5);
  float sr = sin(roll * 0.5);
  
  imu_msg.orientation.w = cr * cp * cy + sr * sp * sy;
  imu_msg.orientation.x = sr * cp * cy - cr * sp * sy;
  imu_msg.orientation.y = cr * sp * cy + sr * cp * sy;
  imu_msg.orientation.z = cr * cp * sy - sr * sp * cy;
  
  // Angular velocity (rad/s)
  imu_msg.angular_velocity.x = gyro_x;
  imu_msg.angular_velocity.y = gyro_y;
  imu_msg.angular_velocity.z = gyro_z;
  
  // Linear acceleration (m/s²)
  imu_msg.linear_acceleration.x = accel_x * 9.81;
  imu_msg.linear_acceleration.y = accel_y * 9.81;
  imu_msg.linear_acceleration.z = accel_z * 9.81;
  
  rcl_ret_t ret = rcl_publish(&imu_publisher, &imu_msg, NULL);
  if (ret != RCL_RET_OK && loop_count % 1000 == 0) {
    Serial.println("IMU publish failed");
  }
}

void publishOdometry() {
  if (!ros_connected) return;
  
  // Update timestamp
  int64_t time_ns = rmw_uros_epoch_nanos();
  odometry_msg.header.stamp.sec = time_ns / 1000000000LL;
  odometry_msg.header.stamp.nanosec = time_ns % 1000000000LL;
  
  // Position
  odometry_msg.pose.pose.position.x = robot_x;
  odometry_msg.pose.pose.position.y = robot_y;
  odometry_msg.pose.pose.position.z = 0.0;
  
  // Orientation (quaternion from yaw)
  float cy = cos(robot_theta * 0.5);
  float sy = sin(robot_theta * 0.5);
  odometry_msg.pose.pose.orientation.w = cy;
  odometry_msg.pose.pose.orientation.x = 0.0;
  odometry_msg.pose.pose.orientation.y = 0.0;
  odometry_msg.pose.pose.orientation.z = sy;
  
  // Velocity
  odometry_msg.twist.twist.linear.x = robot_vx;
  odometry_msg.twist.twist.linear.y = robot_vy;
  odometry_msg.twist.twist.linear.z = 0.0;
  
  odometry_msg.twist.twist.angular.x = 0.0;
  odometry_msg.twist.twist.angular.y = 0.0;
  odometry_msg.twist.twist.angular.z = robot_vtheta;
  
  rcl_ret_t ret = rcl_publish(&odometry_publisher, &odometry_msg, NULL);
  if (ret != RCL_RET_OK && loop_count % 1000 == 0) {
    Serial.println("Odometry publish failed");
  }
}

void publishDiagnostics() {
  if (!ros_connected) return;
  
  // Comprehensive diagnostic with performance metrics
  snprintf(diagnostic_buffer, sizeof(diagnostic_buffer),
    "ESP32+IMU: Up=%lus, Enc=[%ld,%ld,%ld,%ld], Pos=(%.2f,%.2f,%.1f°), IMU=(%.1f,%.1f,%.1f°), MaxLoop=%luµs",
    millis() / 1000,
    encoder_counts[0], encoder_counts[1], encoder_counts[2], encoder_counts[3],
    robot_x, robot_y, robot_theta * 180.0 / PI,
    roll * 180.0 / PI, pitch * 180.0 / PI, yaw * 180.0 / PI,
    max_loop_time
  );
  
  diagnostic_msg.data.size = strlen(diagnostic_buffer);
  rcl_publish(&diagnostic_publisher, &diagnostic_msg, NULL);
  
  // Reset max loop time
  max_loop_time = 0;
}

void blinkLED(int times, int duration_ms) {
  for (int i = 0; i < times; i++) {
    digitalWrite(STATUS_LED_PIN, HIGH);
    delay(duration_ms);
    digitalWrite(STATUS_LED_PIN, LOW);
    if (i < times - 1) delay(duration_ms);
  }
}

void updateOdometry() {
  // Calculate wheel positions from encoders
  float left_pos = ((encoder_counts[0] + encoder_counts[2]) / 2.0) * 2.0 * PI / (ENCODER_PPR * GEAR_RATIO);
  float right_pos = ((encoder_counts[1] + encoder_counts[3]) / 2.0) * 2.0 * PI / (ENCODER_PPR * GEAR_RATIO);
  
  // Calculate wheel position changes
  float left_delta = left_pos - wheel_left_pos_prev;
  float right_delta = right_pos - wheel_right_pos_prev;
  
  wheel_left_pos_prev = left_pos;
  wheel_right_pos_prev = right_pos;
  
  // Calculate linear and angular displacement
  float distance = (left_delta + right_delta) / 2.0;
  float angle_delta = (right_delta - left_delta) / WHEEL_BASE_WIDTH;
  
  // Fusion with IMU yaw (weighted average)
  float encoder_yaw = robot_theta + angle_delta;
  float imu_weight = 0.7f; // Favor IMU for yaw
  float encoder_weight = 0.3f;
  float fused_yaw = imu_weight * yaw + encoder_weight * encoder_yaw;
  
  // Update robot position using fused yaw
  float avg_theta = robot_theta + angle_delta / 2.0;
  robot_x += distance * cos(avg_theta);
  robot_y += distance * sin(avg_theta);
  robot_theta = fused_yaw;
  
  // Normalize theta to [-PI, PI]
  while (robot_theta > PI) robot_theta -= 2.0 * PI;
  while (robot_theta < -PI) robot_theta += 2.0 * PI;
  
  // Calculate velocities
  float dt = 1.0f / ODOMETRY_FREQUENCY_HZ;
  robot_vx = distance / dt;
  robot_vy = 0.0; // Differential drive robot
  robot_vtheta = angle_delta / dt;
}
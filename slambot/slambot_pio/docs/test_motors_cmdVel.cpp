#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rclc/subscription.h>
#include <geometry_msgs/msg/twist.h>

// ------------ MOTOR PINS & CONFIG ------------
const int MOTOR_A_PWM_PIN = 13;
const int MOTOR_B_PWM_PIN = 32;
const int MOTOR_A_IN1_PIN = 5;
const int MOTOR_A_IN2_PIN = 4;
const int MOTOR_B_IN3_PIN = 19;
const int MOTOR_B_IN4_PIN = 18;

const int PWM_CHANNEL_A = 0;
const int PWM_CHANNEL_B = 1;
const int PWM_FREQUENCY = 5000;
const int PWM_RESOLUTION = 8;

// ------------ micro-ROS VARIABLES ------------
rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;
rclc_executor_t executor;
rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg;

// ------------ MOTOR CONTROL FUNCTIONS ------------
void stopMotors() {
  ledcWrite(PWM_CHANNEL_A, 0);
  ledcWrite(PWM_CHANNEL_B, 0);
  digitalWrite(MOTOR_A_IN1_PIN, LOW);
  digitalWrite(MOTOR_A_IN2_PIN, LOW);
  digitalWrite(MOTOR_B_IN3_PIN, LOW);
  digitalWrite(MOTOR_B_IN4_PIN, LOW);
}

void moveForward(uint8_t speed) {
  digitalWrite(MOTOR_A_IN1_PIN, HIGH);
  digitalWrite(MOTOR_A_IN2_PIN, LOW);
  digitalWrite(MOTOR_B_IN3_PIN, HIGH);
  digitalWrite(MOTOR_B_IN4_PIN, LOW);
  ledcWrite(PWM_CHANNEL_A, speed);
  ledcWrite(PWM_CHANNEL_B, speed);
}

void moveBackward(uint8_t speed) {
  digitalWrite(MOTOR_A_IN1_PIN, LOW);
  digitalWrite(MOTOR_A_IN2_PIN, HIGH);
  digitalWrite(MOTOR_B_IN3_PIN, LOW);
  digitalWrite(MOTOR_B_IN4_PIN, HIGH);
  ledcWrite(PWM_CHANNEL_A, speed);
  ledcWrite(PWM_CHANNEL_B, speed);
}

void turnLeft(uint8_t speed) {
  digitalWrite(MOTOR_A_IN1_PIN, LOW);
  digitalWrite(MOTOR_A_IN2_PIN, LOW);
  digitalWrite(MOTOR_B_IN3_PIN, HIGH);
  digitalWrite(MOTOR_B_IN4_PIN, LOW);
  ledcWrite(PWM_CHANNEL_A, 0);
  ledcWrite(PWM_CHANNEL_B, speed);
}

void turnRight(uint8_t speed) {
  digitalWrite(MOTOR_A_IN1_PIN, HIGH);
  digitalWrite(MOTOR_A_IN2_PIN, LOW);
  digitalWrite(MOTOR_B_IN3_PIN, LOW);
  digitalWrite(MOTOR_B_IN4_PIN, LOW);
  ledcWrite(PWM_CHANNEL_A, speed);
  ledcWrite(PWM_CHANNEL_B, 0);
}

// ------------ CMD_VEL CALLBACK FUNCTION ------------
void cmdVelCallback(const void * msgin) {
  const geometry_msgs__msg__Twist * twist = (const geometry_msgs__msg__Twist *)msgin;

  float linear = twist->linear.x;
  float angular = twist->angular.z;

  int speed = 180; // PWM value range (0-255)

  if (linear > 0.1) {
    moveForward(speed);
  } else if (linear < -0.1) {
    moveBackward(speed);
  } else if (angular > 0.1) {
    turnLeft(speed);
  } else if (angular < -0.1) {
    turnRight(speed);
  } else {
    stopMotors();
  }
}

// ------------ SETUP ------------
void setup() {
  Serial.begin(115200);
  delay(2000);

  // Motor pin configuration
  pinMode(MOTOR_A_IN1_PIN, OUTPUT);
  pinMode(MOTOR_A_IN2_PIN, OUTPUT);
  pinMode(MOTOR_B_IN3_PIN, OUTPUT);
  pinMode(MOTOR_B_IN4_PIN, OUTPUT);

  ledcSetup(PWM_CHANNEL_A, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_B, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttachPin(MOTOR_A_PWM_PIN, PWM_CHANNEL_A);
  ledcAttachPin(MOTOR_B_PWM_PIN, PWM_CHANNEL_B);

  stopMotors();

  // micro-ROS setup
  set_microros_serial_transports(Serial);
  allocator = rcl_get_default_allocator();

  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "motor_driver_node", "", &support);

  rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "/cmd_vel"
  );

  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(&executor, &subscriber, &msg, &cmdVelCallback, ON_NEW_DATA);
}

// ------------ LOOP ------------
void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
}

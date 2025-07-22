#include <Arduino.h>
#include <Wire.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

// ROS 2 Messages
#include <sensor_msgs/msg/imu.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/int32.h>

// Hardware Interface Classes
#include "MotorControl.h"
#include "EncoderSensor.h"
#include "MPU9250.h"

// -------------------- Pin Definitions --------------------
#define SDA_PIN 23
#define SCL_PIN 22

// -------------------- Calibration Parameters --------------------
#define ACCEL_BIAS_X -14.14
#define ACCEL_SCALE_X 1.0

#define ACCEL_BIAS_Y -9.71
#define ACCEL_SCALE_Y 1.0

#define ACCEL_BIAS_Z 0.35
#define ACCEL_SCALE_Z 1.0

#define GYRO_BIAS_X 0 //0.062163
#define GYRO_BIAS_Y 0 //0.243808
#define GYRO_BIAS_Z 0 //-0.000411

// -------------------- Encoders and Motor --------------------
EncoderSensor encoderLeft(27, 25);
EncoderSensor encoderRight(26, 14);
MotorControl motorControl(13, 5, 4, 32, 18, 19, 0, 1, 5000, 8);

// -------------------- IMU --------------------
MPU9250 IMU(Wire, 0x68);

// -------------------- Robot Parameters --------------------
#define speed 200

// -------------------- micro-ROS Entities --------------------
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rclc_executor_t executor;

// -------------------- Publishers and Subscriber --------------------
rcl_publisher_t imu_pub;
rcl_publisher_t enc_left_pub;
rcl_publisher_t enc_right_pub;
rcl_subscription_t cmd_vel_sub;

// -------------------- Message Instances --------------------
sensor_msgs__msg__Imu imu_msg;
std_msgs__msg__Int32 enc_left_msg;
std_msgs__msg__Int32 enc_right_msg;
geometry_msgs__msg__Twist cmd_vel_msg;

// -------------------- /cmd_vel Callback --------------------
void cmdVelCallback(const void *msg_in)
{
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msg_in;

  float linear_x = msg->linear.x;    // m/s
  float angular_z = msg->angular.z;  // rad/s

  motorControl.driveWithVelocities(linear_x, angular_z);
}


void cmdVelCallbackOLD(const void *msg_in)
{
  const geometry_msgs__msg__Twist *msg =
      (const geometry_msgs__msg__Twist *)msg_in;

  float linear_x = msg->linear.x;
  float angular_z = msg->angular.z;

  if (linear_x > 0.0f) {
    motorControl.forward(speed);
  } else if (linear_x < 0.0f) {
    motorControl.backward(speed);
  } else if (angular_z > 0.0f) {
    motorControl.turnLeft(speed);
  } else if (angular_z < 0.0f) {
    motorControl.turnRight(speed);
  } else {
    motorControl.stop();
  }
}

// -------------------- Setup --------------------
void setup()
{
  Serial.begin(115200);
  set_microros_serial_transports(Serial);

  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "robot_interface_node", "", &support);

  rclc_publisher_init_default(&imu_pub, &node,
                              ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "/imu/data");

  rclc_publisher_init_default(&enc_left_pub, &node,
                              ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "/encoder_left");

  rclc_publisher_init_default(&enc_right_pub, &node,
                              ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "/encoder_right");

  rclc_subscription_init_default(
      &cmd_vel_sub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/cmd_vel");

  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(&executor, &cmd_vel_sub, &cmd_vel_msg, &cmdVelCallback, ON_NEW_DATA);

  Wire.begin(SDA_PIN, SCL_PIN);
  IMU.begin();
  motorControl.begin();
  encoderLeft.begin();
  encoderRight.begin();
}

// -------------------- Loop --------------------
void loop()
{
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

  IMU.readSensor();

  // IMU orientation placeholder
  imu_msg.orientation.x = 0.0;
  imu_msg.orientation.y = 0.0;
  imu_msg.orientation.z = 0.0;
  imu_msg.orientation.w = 1.0;

  // Apply Gyro calibration (bias remove only)
  imu_msg.angular_velocity.x = IMU.getGyroX_rads() - GYRO_BIAS_X;
  imu_msg.angular_velocity.y = IMU.getGyroY_rads() - GYRO_BIAS_Y;
  imu_msg.angular_velocity.z = IMU.getGyroZ_rads() - GYRO_BIAS_Z;


  // Apply Accel calibration (bias and scale)
  imu_msg.linear_acceleration.x = (IMU.getAccelX_mss() - ACCEL_BIAS_X) * ACCEL_SCALE_X;
  imu_msg.linear_acceleration.y = (IMU.getAccelY_mss() - ACCEL_BIAS_Y) * ACCEL_SCALE_Y;
  imu_msg.linear_acceleration.z = (IMU.getAccelZ_mss() - ACCEL_BIAS_Z) * ACCEL_SCALE_Z;

  rcl_publish(&imu_pub, &imu_msg, NULL);

  long leftCount = encoderLeft.getCount();
  long rightCount = encoderRight.getCount();

  enc_left_msg.data = (int32_t)leftCount;
  enc_right_msg.data = (int32_t)rightCount;

  rcl_publish(&enc_left_pub, &enc_left_msg, NULL);
  rcl_publish(&enc_right_pub, &enc_right_msg, NULL);

  delay(50);
}

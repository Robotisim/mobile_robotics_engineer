#include <Wire.h>
#include "MPU9250.h"

#define SDA_PIN 23
#define SCL_PIN 22

MPU9250 IMU(Wire, 0x68);

// ====> Replace these with your calibrated values from previous step
#define ACCEL_BIAS_X -14.14
#define ACCEL_SCALE_X 1.0

#define ACCEL_BIAS_Y -9.71
#define ACCEL_SCALE_Y 1.0

#define ACCEL_BIAS_Z 0.35
#define ACCEL_SCALE_Z 1.0

#define GYRO_BIAS_X 0.062163
#define GYRO_BIAS_Y 0.243808
#define GYRO_BIAS_Z -0.000411

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);

  if (IMU.begin() != 1) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  // Apply accelerometer calibration
  IMU.setAccelCalX(ACCEL_BIAS_X, ACCEL_SCALE_X);
  IMU.setAccelCalY(ACCEL_BIAS_Y, ACCEL_SCALE_Y);
  IMU.setAccelCalZ(ACCEL_BIAS_Z, ACCEL_SCALE_Z);

  // Apply gyroscope calibration
  IMU.setGyroBiasX_rads(GYRO_BIAS_X);
  IMU.setGyroBiasY_rads(GYRO_BIAS_Y);
  IMU.setGyroBiasZ_rads(GYRO_BIAS_Z);

  Serial.println("Logging IMU Data (with calibration applied)...");
  delay(2000);
}

void loop() {
  IMU.readSensor();

  // Get calibrated accelerometer values (m/s^2)
  float ax = IMU.getAccelX_mss();
  float ay = IMU.getAccelY_mss();
  float az = IMU.getAccelZ_mss();

  // Get calibrated gyroscope values (rad/s)
  float gx = IMU.getGyroX_rads();
  float gy = IMU.getGyroY_rads();
  float gz = IMU.getGyroZ_rads();

  // Log to Serial
  Serial.print("Accel (m/s^2): X: "); Serial.print(ax);
  Serial.print(" | Y: "); Serial.print(ay);
  Serial.print(" | Z: "); Serial.println(az);

  Serial.print("Gyro (rad/s): X: "); Serial.print(gx);
  Serial.print(" | Y: "); Serial.print(gy);
  Serial.print(" | Z: "); Serial.println(gz);

  Serial.println("-----------------------------");

  delay(100);
}

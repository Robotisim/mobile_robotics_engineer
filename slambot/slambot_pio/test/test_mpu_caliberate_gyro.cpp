#include "MPU9250.h"

// Create IMU object (change Wire pins according to your ESP32 wiring)
MPU9250 IMU(Wire, 0x68);
int status;

void setup() {
  // Start I2C communication
  Wire.begin(23, 22);

  // Start Serial communication
  Serial.begin(115200);
  while (!Serial) {}

  // Start IMU
  status = IMU.begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check wiring or power cycle the device");
    Serial.print("Status: ");
    Serial.println(status);
    while (1) {}
  }

  Serial.println("Reading Gyro before Calibration...");
  for (int i = 0; i < 50; i++) {
    IMU.readSensor();
    Serial.print("Gyro X (rad/s): "); Serial.print(IMU.getGyroX_rads(), 6);
    Serial.print("\tGyro Y (rad/s): "); Serial.print(IMU.getGyroY_rads(), 6);
    Serial.print("\tGyro Z (rad/s): "); Serial.println(IMU.getGyroZ_rads(), 6);
    delay(200);
  }

  Serial.println("\nNow Calibrating Gyro... Please keep IMU VERY still...");
  delay(2000);

  // Start calibration (takes few seconds)
  IMU.calibrateGyro();

  Serial.println("\nCalibration Complete!");
  Serial.println("Gyro Biases:");
  Serial.print("X Bias (rad/s): "); Serial.println(IMU.getGyroBiasX_rads(), 6);
  Serial.print("Y Bias (rad/s): "); Serial.println(IMU.getGyroBiasY_rads(), 6);
  Serial.print("Z Bias (rad/s): "); Serial.println(IMU.getGyroBiasZ_rads(), 6);

  delay(2000);

  Serial.println("\nReading Gyro after Calibration (Bias removed)...");

  for (int i = 0; i < 50; i++) {
    IMU.readSensor();
    Serial.print("Gyro X (rad/s): "); Serial.print(IMU.getGyroX_rads(), 6);
    Serial.print("\tGyro Y (rad/s): "); Serial.print(IMU.getGyroY_rads(), 6);
    Serial.print("\tGyro Z (rad/s): "); Serial.println(IMU.getGyroZ_rads(), 6);
    delay(200);
  }
}

void loop() {
  // Nothing in loop
}

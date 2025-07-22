#include <Wire.h>
#include "MPU9250.h"

#define SDA_PIN 23
#define SCL_PIN 22

MPU9250 IMU(Wire, 0x68);

float ax_max = -1000, ax_min = 1000;
float ay_max = -1000, ay_min = 1000;
float az_max = -1000, az_min = 1000;

void waitForUser(const char* message) {
  Serial.println(message);
  while (Serial.read() != '\n') {
    delay(100);
  }
}

void captureAccel() {
  IMU.readSensor();
  float ax = IMU.getAccelX_mss();
  float ay = IMU.getAccelY_mss();
  float az = IMU.getAccelZ_mss();

  if (ax > ax_max) ax_max = ax;
  if (ax < ax_min) ax_min = ax;

  if (ay > ay_max) ay_max = ay;
  if (ay < ay_min) ay_min = ay;

  if (az > az_max) az_max = az;
  if (az < az_min) az_min = az;

  Serial.print("Accel X: "); Serial.print(ax);
  Serial.print(" | Y: "); Serial.print(ay);
  Serial.print(" | Z: "); Serial.println(az);
}

void setup() {
  Serial.begin(115200);

  // Initialize I2C with custom SDA/SCL pins
  Wire.begin(SDA_PIN, SCL_PIN);

  if (IMU.begin() != 1) {
    Serial.println("IMU init failed!");
    while (1);
  }

  Serial.println("=== Accelerometer Calibration ===");
  Serial.println("Place sensor as instructed and press ENTER each time...");
  delay(2000);

  // X+
  waitForUser("Put X UP and press ENTER");
  captureAccel();

  // X-
  waitForUser("Put X DOWN and press ENTER");
  captureAccel();

  // Y+
  waitForUser("Put Y UP and press ENTER");
  captureAccel();

  // Y-
  waitForUser("Put Y DOWN and press ENTER");
  captureAccel();

  // Z+
  waitForUser("Put Z UP and press ENTER");
  captureAccel();

  // Z-
  waitForUser("Put Z DOWN and press ENTER");
  captureAccel();

  // Calculate bias and scale
  float ax_bias = (ax_max + ax_min) / 2.0;
  float ax_scale = 9.81 / ((ax_max - ax_min) / 2.0);

  float ay_bias = (ay_max + ay_min) / 2.0;
  float ay_scale = 9.81 / ((ay_max - ay_min) / 2.0);

  float az_bias = (az_max + az_min) / 2.0;
  float az_scale = 9.81 / ((az_max - az_min) / 2.0);

  Serial.println("=== Calibration Complete ===");
  Serial.print("X Bias: "); Serial.print(ax_bias);
  Serial.print(" | X Scale: "); Serial.println(ax_scale);

  Serial.print("Y Bias: "); Serial.print(ay_bias);
  Serial.print(" | Y Scale: "); Serial.println(ay_scale);

  Serial.print("Z Bias: "); Serial.print(az_bias);
  Serial.print(" | Z Scale: "); Serial.println(az_scale);

  Serial.println("\nCopy below lines for IMU.setAccelCalX/Y/Z:");

  Serial.print("IMU.setAccelCalX("); Serial.print(ax_bias); Serial.print(", "); Serial.print(ax_scale); Serial.println(");");
  Serial.print("IMU.setAccelCalY("); Serial.print(ay_bias); Serial.print(", "); Serial.print(ay_scale); Serial.println(");");
  Serial.print("IMU.setAccelCalZ("); Serial.print(az_bias); Serial.print(", "); Serial.print(az_scale); Serial.println(");");
}

void loop() {
  // Calibration done, nothing here
}

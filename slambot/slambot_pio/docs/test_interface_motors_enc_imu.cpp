/*
  Unified Robot Control + Sensor Readings

  This example:
    - Initializes and reads from MPU9250 via I2C
    - Controls motors (forward, backward, left turn, right turn)
    - Reads encoder counts for both left and right motors
    - Prints sensor (MPU and Encoders) data to Serial

  Make sure you have the following libraries available:
    - Bolder Flight Systems MPU9250 (or equivalent)
    - MotorControl.h
    - EncoderSensor.h
*/

#include <Arduino.h>
#include <Wire.h>
#include "MotorControl.h"
#include "EncoderSensor.h"
#include "MPU9250.h"

// -------------------- PIN DEFINITIONS -------------------- //

// I2C pins (adjust these if needed for your board)
#define SDA_PIN 41
#define SCL_PIN 42
MPU9250 IMU(Wire, 0x68);

// Motor controller
MotorControl motorControl;

// Encoders
EncoderSensor encoderLeft(6, 4);
EncoderSensor encoderRight(5, 7);



int status;

void setup() {
  // Initialize Serial
  Serial.begin(115200);
  while(!Serial) {
    // Wait for Serial to be available (for boards like Leonardo/ESP32)
  }

  // Initialize I2C
  Wire.begin(SDA_PIN, SCL_PIN);  // Adjust for your board if needed

  // Initialize IMU
  status = IMU.begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while(1) {
      // Stay here if IMU failed to initialize
    }
  }

  // Initialize Motor Control
  motorControl.begin();

  // Initialize Encoder Sensors
  encoderLeft.begin();
  encoderRight.begin();

  Serial.println("Robot + MPU9250 Initialized");
  delay(2000);
}

void loop() {
  // -------------------- READ MPU9250 -------------------- //
  IMU.readSensor();
  float accelX = IMU.getAccelX_mss();
  float accelY = IMU.getAccelY_mss();
  float accelZ = IMU.getAccelZ_mss();
  float gyroX = IMU.getGyroX_rads();
  float gyroY = IMU.getGyroY_rads();
  float gyroZ = IMU.getGyroZ_rads();
  // float magX  = IMU.getMagX_uT();
  // float magY  = IMU.getMagY_uT();
  // float magZ  = IMU.getMagZ_uT();
  // float tempC = IMU.getTemperature_C();

  // Print out Acceleration for demonstration
  Serial.print("Accel (m/s^2) X: ");
  Serial.print(accelX, 6);
  Serial.print("\tY: ");
  Serial.print(accelY, 6);
  Serial.print("\tZ: ");
  Serial.println(accelZ, 6);
  Serial.print("Gyro (m/s^2) X: ");
  Serial.print(gyroX, 6);
  Serial.print("\tY: ");
  Serial.print(gyroY, 6);
  Serial.print("\tZ: ");
  Serial.println(gyroZ, 6);
  // -------------------- DRIVE MOTORS -------------------- //
  // Example motion sequence:
  Serial.println("Moving Forward");
  motorControl.forward(200);    // speed = 200 (adjust as needed)
  delay(2000);
  motorControl.stop();
  delay(500);

  Serial.println("Moving Backward");
  motorControl.backward(200);
  delay(2000);
  motorControl.stop();
  delay(500);

  Serial.println("Turning Left");
  motorControl.turnLeft(200);
  delay(2000);
  motorControl.stop();
  delay(500);

  Serial.println("Turning Right");
  motorControl.turnRight(200);
  delay(2000);
  motorControl.stop();
  delay(500);

  // -------------------- READ ENCODERS -------------------- //
  long leftCount = encoderLeft.getCount();
  long rightCount = encoderRight.getCount();
  Serial.print("Left Encoder Count: ");
  Serial.print(leftCount);
  Serial.print("\tRight Encoder Count: ");
  Serial.println(rightCount);

  // -------------------- DELAY BEFORE NEXT LOOP -------------------- //
  delay(1000);
}

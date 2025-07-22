#include <Arduino.h>

// --- Pin Definitions for L298D Motor Driver ---
const int MOTOR_A_PWM_PIN = 13;
const int MOTOR_B_PWM_PIN = 32;

const int MOTOR_A_IN1_PIN = 5;
const int MOTOR_A_IN2_PIN = 4;

const int MOTOR_B_IN3_PIN = 19;
const int MOTOR_B_IN4_PIN = 18;

// --- LEDC PWM Configuration ---
const int PWM_CHANNEL_A = 0;
const int PWM_CHANNEL_B = 1;
const int PWM_FREQUENCY   = 5000;
int PWM_RESOLUTION  = 8;


// --- Motor Control Functions ---

void stopMotors() {
  // Set PWM duty cycle to 0 for both channels
  ledcWrite(PWM_CHANNEL_A, 0);
  ledcWrite(PWM_CHANNEL_B, 0);

  // Set all direction pins to LOW (motors off)
  digitalWrite(MOTOR_A_IN1_PIN, LOW);
  digitalWrite(MOTOR_A_IN2_PIN, LOW);
  digitalWrite(MOTOR_B_IN3_PIN, LOW);
  digitalWrite(MOTOR_B_IN4_PIN, LOW);

}

void moveForward(uint8_t speed) {

  // For forward motion, set left motor to drive forward:
  digitalWrite(MOTOR_A_IN1_PIN, HIGH);
  digitalWrite(MOTOR_A_IN2_PIN, LOW);

  // and right motor to drive forward:
  digitalWrite(MOTOR_B_IN3_PIN, HIGH);
  digitalWrite(MOTOR_B_IN4_PIN, LOW);

  ledcWrite(PWM_CHANNEL_A, speed);
  ledcWrite(PWM_CHANNEL_B, speed);

}

void moveBackward(uint8_t speed) {

  // For backward motion, reverse the motor polarity:
  digitalWrite(MOTOR_A_IN1_PIN, LOW);
  digitalWrite(MOTOR_A_IN2_PIN, HIGH);

  digitalWrite(MOTOR_B_IN3_PIN, LOW);
  digitalWrite(MOTOR_B_IN4_PIN, HIGH);

  ledcWrite(PWM_CHANNEL_A, speed);
  ledcWrite(PWM_CHANNEL_B, speed);


}

void turnLeft(uint8_t speed) {

  // Stop left motor
  digitalWrite(MOTOR_A_IN1_PIN, LOW);
  digitalWrite(MOTOR_A_IN2_PIN, LOW);
  // Drive right motor forward
  digitalWrite(MOTOR_B_IN3_PIN, HIGH);
  digitalWrite(MOTOR_B_IN4_PIN, LOW);

  ledcWrite(PWM_CHANNEL_A, 0);
  ledcWrite(PWM_CHANNEL_B, speed);


}

void turnRight(uint8_t speed) {

  // Drive left motor forward
  digitalWrite(MOTOR_A_IN1_PIN, HIGH);
  digitalWrite(MOTOR_A_IN2_PIN, LOW);
  // Stop right motor
  digitalWrite(MOTOR_B_IN3_PIN, LOW);
  digitalWrite(MOTOR_B_IN4_PIN, LOW);

  ledcWrite(PWM_CHANNEL_A, speed);
  ledcWrite(PWM_CHANNEL_B, 0);


}

void setup() {
  Serial.begin(115200);


// Configure motor direction pins as outputs
pinMode(MOTOR_A_IN1_PIN, OUTPUT);
pinMode(MOTOR_A_IN2_PIN, OUTPUT);
pinMode(MOTOR_B_IN3_PIN, OUTPUT);
pinMode(MOTOR_B_IN4_PIN, OUTPUT);

// Configure LEDC PWM channels
ledcSetup(PWM_CHANNEL_A, PWM_FREQUENCY, PWM_RESOLUTION);
ledcSetup(PWM_CHANNEL_B, PWM_FREQUENCY, PWM_RESOLUTION);

// Attach LEDC channels to their corresponding PWM pins
ledcAttachPin(MOTOR_A_PWM_PIN, PWM_CHANNEL_A);
ledcAttachPin(MOTOR_B_PWM_PIN, PWM_CHANNEL_B);

// Start with motors stopped
stopMotors();


  Serial.println("Prepare Robot");
  delay(3000);
}

void loop() {

  Serial.println("Moving Forward");
  moveForward(200);
  delay(2000);
  stopMotors();
  delay(500);

  Serial.println("Moving Backward");
  moveBackward(200);
  delay(2000);
  stopMotors();
  delay(500);

  Serial.println("Turning Left");
  turnLeft(200);
  delay(2000);
  stopMotors();
  delay(500);

  Serial.println("Turning Right");
  turnRight(200);
  delay(2000);
  stopMotors();
  delay(500);




}

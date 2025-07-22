#include <Arduino.h>

// Motor PWM and Direction Pins
const int MOTOR_L_PWM = 32;
const int MOTOR_R_PWM = 33;
const int MOTOR_L_IN1 = 4;
const int MOTOR_L_IN2 = 16;
const int MOTOR_R_IN3 = 17;
const int MOTOR_R_IN4 = 5;

// Encoder Pins
const int ENCODER_L_A = 27;
const int ENCODER_L_B = 25;
const int ENCODER_R_A = 26;
const int ENCODER_R_B = 14;

// Encoder counts
volatile long encoderLCount = 0;
volatile long encoderRCount = 0;
long lastLCount = 0;
long lastRCount = 0;

// Timing
unsigned long lastPrintTime = 0;
const unsigned long interval = 1000; // 1 second

int test_pwm = 255; // Full speed for calibration



unsigned long motorStartTime = 0;
bool motorStopped = false;


void IRAM_ATTR encoderLISR() {
  if (digitalRead(ENCODER_L_B) == HIGH)
    encoderLCount++;
  else
    encoderLCount--;
}

void IRAM_ATTR encoderRISR() {
  if (digitalRead(ENCODER_R_B) == HIGH)
    encoderRCount++;
  else
    encoderRCount--;
}

void setupMotors() {
  pinMode(MOTOR_L_IN1, OUTPUT);
  pinMode(MOTOR_L_IN2, OUTPUT);
  pinMode(MOTOR_R_IN3, OUTPUT);
  pinMode(MOTOR_R_IN4, OUTPUT);

  ledcSetup(0, 20000, 8);
  ledcAttachPin(MOTOR_L_PWM, 0);

  ledcSetup(1, 20000, 8);
  ledcAttachPin(MOTOR_R_PWM, 1);
}

void setMotorPWM(int pwmL, int pwmR) {
  digitalWrite(MOTOR_L_IN1, HIGH);
  digitalWrite(MOTOR_L_IN2, LOW);
  digitalWrite(MOTOR_R_IN3, HIGH);
  digitalWrite(MOTOR_R_IN4, LOW);

  ledcWrite(0, pwmL);
  ledcWrite(1, pwmR);
}


void setup() {
  Serial.begin(115200);

  pinMode(ENCODER_L_A, INPUT_PULLUP);
  pinMode(ENCODER_L_B, INPUT_PULLUP);
  pinMode(ENCODER_R_A, INPUT_PULLUP);
  pinMode(ENCODER_R_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCODER_L_A), encoderLISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_R_A), encoderRISR, RISING);

  setupMotors();
  setMotorPWM(test_pwm, test_pwm);  // Start motors
  motorStartTime = millis();
  lastPrintTime = millis();
}

void loop() {
  unsigned long currentMillis = millis();

  // Print ticks/sec every 1 second
  if (currentMillis - lastPrintTime >= interval) {
    lastPrintTime = currentMillis;

    long currentL = encoderLCount;
    long currentR = encoderRCount;

    long deltaL = currentL - lastLCount;
    long deltaR = currentR - lastRCount;

    lastLCount = currentL;
    lastRCount = currentR;

    Serial.print("PWM: ");
    Serial.print(test_pwm);
    Serial.print(" | Ticks/sec L: ");
    Serial.print(deltaL);
    Serial.print(" | Ticks/sec R: ");
    Serial.println(deltaR);
  }

  // Stop motors after 5 seconds
  if (!motorStopped && currentMillis - motorStartTime >= 5000) {
    setMotorPWM(0, 0);
    motorStopped = true;
    Serial.println("⛔ Motors stopped after 5 seconds");
  }
}

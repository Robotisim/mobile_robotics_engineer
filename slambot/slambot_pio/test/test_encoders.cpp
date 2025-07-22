#include <Arduino.h>

// Global variables to hold encoder counts
volatile long encoderLCount = 0;
volatile long encoderRCount = 0;


// Define pin names
const int LEFT_MOTOR_A = 27;
const int LEFT_MOTOR_B = 25;
const int RIGHT_MOTOR_A = 26;
const int RIGHT_MOTOR_B = 14;

// Interrupt Service Routine for Encoder 1 (Left Motor)
void IRAM_ATTR encoderLISR() {
  if (digitalRead(LEFT_MOTOR_B) == HIGH) {
    encoderLCount++;
  } else {
    encoderLCount--;
  }
}

// Interrupt Service Routine for Encoder 2 (Right Motor)
void IRAM_ATTR encoderRISR() {
  if (digitalRead(RIGHT_MOTOR_B) == HIGH) {
    encoderRCount++;
  } else {
    encoderRCount--;
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(LEFT_MOTOR_A, INPUT_PULLUP);
  pinMode(LEFT_MOTOR_B, INPUT_PULLUP);
  pinMode(RIGHT_MOTOR_A, INPUT_PULLUP);
  pinMode(RIGHT_MOTOR_B, INPUT_PULLUP);

  // Attach interrupts on the A channels of the encoders
  attachInterrupt(digitalPinToInterrupt(LEFT_MOTOR_A), encoderLISR, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_MOTOR_A), encoderRISR, RISING);
}

void loop() {
  Serial.print("Encoder 1 Count: ");
  Serial.print(encoderLCount);
  Serial.print("   Encoder 2 Count: ");
  Serial.println(encoderRCount);
  delay(300);
}

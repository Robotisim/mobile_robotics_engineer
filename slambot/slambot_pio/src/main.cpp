#include <Arduino.h>
#include <Wire.h>

// Hardware and Controller Classes
#include "MotorControl.h"
#include "EncoderSensor.h"
#include "MPU9250.h"


// --- Robot Physical Constants ---
#define TICKS_PER_REV 2400.0f
#define WHEEL_RADIUS 0.033f

// --- Hardware Objects ---
EncoderSensor encoderLeft(27, 25);
EncoderSensor encoderRight(26, 14);
MotorControl motorControl(13, 5, 4, 32, 18, 19, 0, 1, 5000, 8);

// --- P Controller Setup ---
#define KP 0.8f
#define PWM_MIN -255
#define PWM_MAX 255

// --- Global State Variables ---
volatile float target_speed_L_ticks_s = 0.0;
volatile float target_speed_R_ticks_s = 0.0;
volatile float current_speed_L_ticks_s = 0.0;
volatile float current_speed_R_ticks_s = 0.0;

long prev_ticks_L = 0;
long prev_ticks_R = 0;
unsigned long prev_update_time_micros = 0;

// --- Communication & Control Parameters ---
#define BAUD_RATE 115200
#define REPORT_INTERVAL_MS 20      // How often to send sensor data (50 Hz)
#define P_UPDATE_INTERVAL_MS 10    // How often to run the P loop (100 Hz)
#define COMMAND_TIMEOUT_MS 500     // Stop motors if no command received

unsigned long lastReportTime_ms = 0;
unsigned long lastPidTime_ms = 0;
unsigned long lastCommandTime_ms = 0;

// --- Core Functions ---
void handleSerialCommands() {
    if (Serial.available() > 0) {
        String command = Serial.readStringUntil('\n');
        if (command.startsWith("v")) {
            float left_rad_s, right_rad_s;
            if (sscanf(command.c_str(), "v %f %f", &left_rad_s, &right_rad_s) == 2) {
                target_speed_L_ticks_s = (left_rad_s / (2.0f * M_PI)) * TICKS_PER_REV;
                target_speed_R_ticks_s = (right_rad_s / (2.0f * M_PI)) * TICKS_PER_REV;
                lastCommandTime_ms = millis();
            }
        }
    }
}

void updateCurrentSpeeds() {
    unsigned long current_time_micros = micros();
    float dt = (current_time_micros - prev_update_time_micros) / 1000000.0f;

    if (dt <= 0.0001) return; // Avoid division by zero if dt is too small

    long current_ticks_L = encoderLeft.getCount();
    long current_ticks_R = encoderRight.getCount();

    // Calculate speed in ticks per second
    current_speed_L_ticks_s = (float)(current_ticks_L - prev_ticks_L) / dt;
    current_speed_R_ticks_s = (float)(current_ticks_R - prev_ticks_R) / dt;

    prev_ticks_L = current_ticks_L;
    prev_ticks_R = current_ticks_R;
    prev_update_time_micros = current_time_micros;

}

void runControlLoop() {

    // 1. Get current speeds from encoders
    updateCurrentSpeeds();

    // 2. Calculate error for each motor
    float error_L = target_speed_L_ticks_s - current_speed_L_ticks_s;
    float error_R = target_speed_R_ticks_s - current_speed_R_ticks_s;

    // 3. Compute P-controller output (PWM) and clamp to valid range
    int pwm_L = (int)constrain(KP * error_L, PWM_MIN, PWM_MAX);
    int pwm_R = (int)constrain(KP * error_R, PWM_MIN, PWM_MAX);

    // 4. Command the motors with the new PWM values
    motorControl.setMotorSpeeds(pwm_L, pwm_R);

}

void publishSensorData() {

    long leftCount = encoderLeft.getCount();
    long rightCount = encoderRight.getCount();

    Serial.printf("e %ld %ld\n", leftCount, rightCount);

}

// --- Arduino Setup & Loop ---

void setup() {
    Serial.begin(BAUD_RATE);
    motorControl.begin();
    encoderLeft.begin();
    encoderRight.begin();

    delay(100);
    prev_update_time_micros = micros();
    Serial.println("ESP32 P-Control Driver Ready.");
}

void loop() {
    handleSerialCommands();

    // Run the high-frequency P-control loop
    if (millis() - lastPidTime_ms >= P_UPDATE_INTERVAL_MS) {
        runControlLoop();
        lastPidTime_ms = millis();
    }

    // Publish sensor data at a slightly lower frequency
    if (millis() - lastReportTime_ms >= REPORT_INTERVAL_MS) {
        publishSensorData();
        lastReportTime_ms = millis();
    }

    // Safety stop: if no commands received, stop the robot
    if (millis() - lastCommandTime_ms > COMMAND_TIMEOUT_MS) {
        if (target_speed_L_ticks_s != 0 || target_speed_R_ticks_s != 0) {
            target_speed_L_ticks_s = 0.0;
            target_speed_R_ticks_s = 0.0;
        }
    }

}
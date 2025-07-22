#include <Arduino.h>
#include <math.h>  // for M_PI

// Single test‐case struct (only one axis non-zero per test)
struct TestCase {
  float linear_x;
  float angular_z;
  const char* description;
};

// All tests in one array
TestCase testCases[] = {
  {0.5f,  0.0f,  "Linear"},
  {0.6f,  0.0f,  "Linear"},
  {0.7f,  0.0f,  "Linear"},
  {0.8f,  0.0f,  "Linear"},
  {0.9f,  0.0f,  "Linear"},
  {1.0f,  0.0f,  "Linear"},
  {1.1f,  0.0f,  "Linear"},
  {-1.1f,  0.0f,  "Linear"},
  {1.3f,  0.0f,  "Linear 1.3 m/s"},
  {0.0f,  0.5f,  "Angular"},
  {0.0f,  0.6f,  "Angular"},
  {0.0f,  0.7f,  "Angular"},
  {0.0f,  0.8f,  "Angular"},
  {0.0f,  0.9f,  "Angular"},
  {0.0f,  1.0f,  "Angular"},
  {0.0f,  1.1f,  "Angular"},
  {0.0f, -1.3f,  "Angular -1.3 rad/s"},
};
const int numTests = sizeof(testCases) / sizeof(TestCase);

// Robot constants
const float WHEEL_RADIUS    = 0.065f / 2.0f;  // m
const float WHEEL_BASE      = 0.165f;         // m
const float TICKS_PER_REV   = 380.0f;
const float WHEEL_CIRC      = 2 * M_PI * WHEEL_RADIUS;

// “Raw” max inputs used for converting to ticks/sec
const float MAX_RAW_LINEAR  = 1.3f;  // m/s
const float MAX_RAW_ANGULAR = 1.3f;  // rad/s

// Precompute for raw‐to‐ticks conversion
const float MAX_TICKS_LIN   = (MAX_RAW_LINEAR  / WHEEL_CIRC) * TICKS_PER_REV;
const float MAX_WHEEL_TURN  = (MAX_RAW_ANGULAR * WHEEL_BASE / 2.0f);
const float MAX_TICKS_TURN  = (MAX_WHEEL_TURN  / WHEEL_CIRC) * TICKS_PER_REV;

// Desired mapping breakpoints
const float IN_LOW  = 0.5f;    // input at which PWM_LOW is reached
const int   PWM_LOW = 170;     // PWM at input = IN_LOW
const float IN_HIGH = 1.3f;    // input at which PWM_HIGH is reached
const int   PWM_HIGH= 255;     // PWM at input = IN_HIGH

// Timing
unsigned long lastPrint = 0;
const unsigned long INTERVAL = 2000;  // 2 s
int currentTest = 0;

// Linearly interpolate x in [x0..x1] to y in [y0..y1]
float lerp(float x, float x0, float x1, float y0, float y1) {
  float t = (x - x0) / (x1 - x0);
  if      (t <= 0) return y0;
  else if (t >= 1) return y1;
  else             return y0 + t * (y1 - y0);
}

// Map raw tick‐based PWM to the desired [170..255] at [0.5..1.3]
int remapPWM(float raw_pwm, float cmd_input) {
  float sign = raw_pwm < 0 ? -1.0f : 1.0f;
  float abs_pwm = fabs(raw_pwm);
  // Only remap if cmd_input ≥ IN_LOW, else scale 0→IN_LOW to 0→PWM_LOW
  float target = lerp(cmd_input,
                      0.0f, IN_LOW,    // below 0.5 goes from 0
                      0.0f, PWM_LOW)   // up to PWM_LOW
            + lerp(cmd_input,
                      IN_LOW, IN_HIGH,
                      PWM_LOW, PWM_HIGH);
  // But we want a *continuous* piecewise function: below 0.5 -> linear 0..170,
  // above -> 170..255. So clamp:
  float final_pwm = lerp(cmd_input,
                         0.0f, IN_LOW,
                         0.0f, PWM_LOW);
  if (cmd_input > IN_LOW) {
    final_pwm = lerp(cmd_input,
                     IN_LOW, IN_HIGH,
                     PWM_LOW, PWM_HIGH);
  }
  // preserve raw sign
  return static_cast<int>(sign * final_pwm);
}

void setup() {
  Serial.begin(115200);
  while (!Serial);
}

void loop() {
  if (millis() - lastPrint < INTERVAL) return;
  lastPrint = millis();

  auto &tc = testCases[currentTest];

  // 1) Compute wheel linear velocities (m/s)
  float vL = tc.linear_x - (tc.angular_z * WHEEL_BASE / 2.0f);
  float vR = tc.linear_x + (tc.angular_z * WHEEL_BASE / 2.0f);

  // 2) Convert to ticks/sec
  float tL = (vL / WHEEL_CIRC) * TICKS_PER_REV;
  float tR = (vR / WHEEL_CIRC) * TICKS_PER_REV;

  // 3) Convert ticks to “raw” PWM in [–255..+255]
  bool isTurn = (tc.linear_x == 0.0f && tc.angular_z != 0.0f);
  float rawL = (tL / (isTurn ? MAX_TICKS_TURN : MAX_TICKS_LIN)) * 255.0f;
  float rawR = (tR / (isTurn ? MAX_TICKS_TURN : MAX_TICKS_LIN)) * 255.0f;

  // 4) Remap so that cmd_input 0.5 → 170, 1.3 → 255
  float cmdInput = isTurn ? fabs(tc.angular_z) : fabs(tc.linear_x);
  int pwmL = remapPWM(rawL, cmdInput);
  int pwmR = remapPWM(rawR, cmdInput);

  // Print
  Serial.printf("[%s]  →  PWM L=%d  R=%d\n",
                tc.description, pwmL, pwmR);

  // Next
  currentTest = (currentTest + 1) % numTests;
}

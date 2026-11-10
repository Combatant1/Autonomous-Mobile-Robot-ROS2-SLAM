#include <Arduino.h>
#include <math.h>

// ==========================================================
// 4-WHEEL SKID-STEER ROBOT  -  ROS 2 <-> ESP32 MOTOR BRIDGE
// ESP32 NodeMCU-32S + TWO L298N MOTOR DRIVERS
// Arduino ESP32 Core 3.x
//
// Per-wheel PI speed control (X4 quadrature decoding, measured
// PWM->RPM feedforward tables) driven by the ROS 2 hardware
// interface (amr_firmware_esp32 / AmrInterface) over USB serial,
// 115200 baud.
//
// SERIAL PROTOCOL (same as the previous firmware)
//
//   ROS -> ESP32 : one token per wheel, "<id><sign><value>,"
//                  e.g.  1p05.00,2n03.20,3p05.00,4n03.20,
//                  id    1 = front right   2 = front left
//                        3 = rear right    4 = rear left
//                  sign  p = forward, n = reverse
//                  value wheel speed in rad/s (magnitude)
//                  No newline is required (newlines are ignored).
//
//   ESP32 -> ROS : same token format, all four wheels, every
//                  SAMPLE_TIME_MS, followed by a newline.
//                  value = measured wheel speed in rad/s (magnitude)
//                  sign  = measured direction (p / n)
//
// BEHAVIOUR
//   |speed| <  37 RPM ( 3.87 rad/s)  ->  that wheel stops
//   |speed| > 140 RPM (14.66 rad/s)  ->  limited to 140 RPM
//   no valid command for 500 ms      ->  ALL wheels stop (watchdog)
//   direction change                 ->  wheel coasts to (almost)
//                                        standstill, then reverses
// ==========================================================

// ==========================================================
// DEBUG OPTION
// 0 = normal operation (ROS protocol only)            <- use this with ROS
// 1 = bench testing from the Arduino Serial Monitor:
//     prints a readable table instead of the ROS report, and the
//     watchdog is relaxed to 10 s. Commands are typed in the same
//     format, e.g.  1p10.00,2p10.00,3p10.00,4p10.00,
// ==========================================================

#define DEBUG_TABLE 0

// ==========================================================
// GENERAL SETTINGS
// ==========================================================

const int PWM_FREQ = 20000;
const int PWM_RESOLUTION = 8;

// Control + report period. The PI gains below were tuned at 250 ms;
// if a wheel oscillates at 100 ms, halve Kp (or set this back to 250).
const unsigned long SAMPLE_TIME_MS = 100;
const float SAMPLE_TIME = SAMPLE_TIME_MS / 1000.0f;

const int MAX_PWM = 255;
const float MAX_TARGET_RPM = 140.0;
const float MIN_TARGET_RPM = 37.0;

const float RAD_S_TO_RPM = 60.0f / (2.0f * PI);
const float RPM_TO_RAD_S = (2.0f * PI) / 60.0f;

// Safety watchdog: no valid command for this long -> stop everything
const unsigned long CMD_TIMEOUT_MS = DEBUG_TABLE ? 10000 : 500;

// When the commanded direction flips, the wheel coasts until it is
// slower than this before the L298N is switched to the other direction
const float REVERSE_SETTLE_RPM = 8.0;

// ==========================================================
// MOTOR 1: FRONT RIGHT
// RIGHT L298N - ENA
// ==========================================================

const int FR_EN = 27;
const int FR_IN1 = 21;
const int FR_IN2 = 22;

const int FR_ENC_A = 32;
const int FR_ENC_B = 4;

const float FR_CPR = 1440.0;

const float Kp_FR = 0.2;
const float Ki_FR = 0.15;

// Direction constants. Test with a small forward command ("p"):
//  * wheel turns backwards            -> set MOTOR_DIR to -1
//  * measured speed reported negative -> set ENC_SIGN to -1
const int FR_MOTOR_DIR = 1;
const int FR_ENC_SIGN = 1;

const int FR_PWM_TABLE[] = {
  0, 145, 150, 155, 160, 170, 180,
  190, 200, 210, 220, 230, 240, 255
};

const float FR_RPM_TABLE[] = {
  0, 26, 40, 50, 65, 90, 110,
  130, 160, 185, 205, 225, 250, 265
};

const int FR_TABLE_SIZE =
  sizeof(FR_PWM_TABLE) / sizeof(FR_PWM_TABLE[0]);

// ==========================================================
// MOTOR 2: FRONT LEFT
// LEFT L298N - ENA
// ==========================================================

const int FL_EN = 25;
const int FL_IN1 = 16;
const int FL_IN2 = 17;

const int FL_ENC_A = 34;
const int FL_ENC_B = 35;

const float FL_CPR = 1440.0;

const float Kp_FL = 0.2;
const float Ki_FL = 0.15;

const int FL_MOTOR_DIR = 1;
const int FL_ENC_SIGN = 1;

const int FL_PWM_TABLE[] = {
  0, 145, 150, 155, 160, 170, 180,
  190, 200, 210, 220, 230, 240, 255
};

const float FL_RPM_TABLE[] = {
  0, 28, 44, 55, 70, 95, 115,
  140, 167, 190, 210, 235, 260, 270
};

const int FL_TABLE_SIZE =
  sizeof(FL_PWM_TABLE) / sizeof(FL_PWM_TABLE[0]);

// ==========================================================
// MOTOR 3: REAR RIGHT
// RIGHT L298N - ENB
// ==========================================================

const int RR_EN = 14;
const int RR_IN1 = 23;
const int RR_IN2 = 13;

const int RR_ENC_A = 5;
const int RR_ENC_B = 15;

const float RR_CPR = 2473.0;

const float Kp_RR = 1.7;
const float Ki_RR = 0.7;

const int RR_MOTOR_DIR = 1;
const int RR_ENC_SIGN = 1;

const int RR_PWM_TABLE[] = {
  0, 30, 50, 75, 100, 125,
  150, 175, 200, 225, 255
};

const float RR_RPM_TABLE[] = {
  0, 36, 48, 67, 90, 110,
  120, 140, 145, 150, 150
};

const int RR_TABLE_SIZE =
  sizeof(RR_PWM_TABLE) / sizeof(RR_PWM_TABLE[0]);

// ==========================================================
// MOTOR 4: REAR LEFT
// LEFT L298N - ENB
// ==========================================================

const int RL_EN = 26;
const int RL_IN1 = 18;
const int RL_IN2 = 19;

const int RL_ENC_A = 36;
const int RL_ENC_B = 39;

const float RL_CPR = 2476.0;

const float Kp_RL = 0.9;
const float Ki_RL = 0.4;

const int RL_MOTOR_DIR = 1;
const int RL_ENC_SIGN = 1;

const int RL_PWM_TABLE[] = {
  0, 30, 50, 75, 100, 125,
  150, 175, 200, 225, 255
};

const float RL_RPM_TABLE[] = {
  0, 37, 47, 60, 77, 92,
  107, 122, 134, 136, 140
};

const int RL_TABLE_SIZE =
  sizeof(RL_PWM_TABLE) / sizeof(RL_PWM_TABLE[0]);

// ==========================================================
// PER-WHEEL LOOK-UP ARRAYS
// Index order = ROS wheel id - 1
// ==========================================================

const int NUM_WHEELS = 4;

enum WheelIndex { FR = 0, FL = 1, RR = 2, RL = 3 };

const int EN_PIN[NUM_WHEELS]  = { FR_EN,  FL_EN,  RR_EN,  RL_EN };
const int IN1_PIN[NUM_WHEELS] = { FR_IN1, FL_IN1, RR_IN1, RL_IN1 };
const int IN2_PIN[NUM_WHEELS] = { FR_IN2, FL_IN2, RR_IN2, RL_IN2 };

const float CPR[NUM_WHEELS] = { FR_CPR, FL_CPR, RR_CPR, RL_CPR };
const float KP[NUM_WHEELS]  = { Kp_FR, Kp_FL, Kp_RR, Kp_RL };
const float KI[NUM_WHEELS]  = { Ki_FR, Ki_FL, Ki_RR, Ki_RL };

const int MOTOR_DIR[NUM_WHEELS] = { FR_MOTOR_DIR, FL_MOTOR_DIR, RR_MOTOR_DIR, RL_MOTOR_DIR };
const int ENC_SIGN[NUM_WHEELS]  = { FR_ENC_SIGN,  FL_ENC_SIGN,  RR_ENC_SIGN,  RL_ENC_SIGN };

const int *const PWM_TABLE[NUM_WHEELS] = {
  FR_PWM_TABLE, FL_PWM_TABLE, RR_PWM_TABLE, RL_PWM_TABLE
};
const float *const RPM_TABLE[NUM_WHEELS] = {
  FR_RPM_TABLE, FL_RPM_TABLE, RR_RPM_TABLE, RL_RPM_TABLE
};
const int TABLE_SIZE[NUM_WHEELS] = {
  FR_TABLE_SIZE, FL_TABLE_SIZE, RR_TABLE_SIZE, RL_TABLE_SIZE
};

// ==========================================================
// ENCODER VARIABLES
// ==========================================================

volatile int32_t countFR = 0;
volatile int32_t countFL = 0;
volatile int32_t countRR = 0;
volatile int32_t countRL = 0;

volatile uint8_t prevFR = 0;
volatile uint8_t prevFL = 0;
volatile uint8_t prevRR = 0;
volatile uint8_t prevRL = 0;

portMUX_TYPE encoderMux = portMUX_INITIALIZER_UNLOCKED;

// Quadrature X4 transition table
DRAM_ATTR const int8_t encoderTable[16] = {
   0, -1,  1,  0,
   1,  0,  0, -1,
  -1,  0,  0,  1,
   0,  1, -1,  0
};

// ==========================================================
// CONTROL STATE (index = wheel)
// ==========================================================

float integral[NUM_WHEELS] = {0, 0, 0, 0};

float rpmSigned[NUM_WHEELS] = {0, 0, 0, 0};   // measured, signed
float rpmAbs[NUM_WHEELS]    = {0, 0, 0, 0};   // measured, magnitude

int pwmOut[NUM_WHEELS] = {0, 0, 0, 0};

// Commanded by ROS
float targetRPM[NUM_WHEELS] = {0, 0, 0, 0};  // magnitude, 0 = stop
int targetDir[NUM_WHEELS]   = {0, 0, 0, 0};  // +1 forward, -1 reverse, 0 stop

// Direction the motor was last driven in (kept while stopped, so a
// reversal request can wait for the wheel to stop spinning)
int motorDir[NUM_WHEELS] = {0, 0, 0, 0};

unsigned long previousMillis = 0;
unsigned long lastCmdMs = 0;

// ==========================================================
// ENCODER INTERRUPT SERVICE ROUTINES
// ==========================================================

void IRAM_ATTR encoderISR_FR() {

  portENTER_CRITICAL_ISR(&encoderMux);

  uint8_t currentState =
    (digitalRead(FR_ENC_A) << 1) |
    digitalRead(FR_ENC_B);

  uint8_t index = (prevFR << 2) | currentState;

  countFR += encoderTable[index];
  prevFR = currentState;

  portEXIT_CRITICAL_ISR(&encoderMux);
}

void IRAM_ATTR encoderISR_FL() {

  portENTER_CRITICAL_ISR(&encoderMux);

  uint8_t currentState =
    (digitalRead(FL_ENC_A) << 1) |
    digitalRead(FL_ENC_B);

  uint8_t index = (prevFL << 2) | currentState;

  countFL += encoderTable[index];
  prevFL = currentState;

  portEXIT_CRITICAL_ISR(&encoderMux);
}

void IRAM_ATTR encoderISR_RR() {

  portENTER_CRITICAL_ISR(&encoderMux);

  uint8_t currentState =
    (digitalRead(RR_ENC_A) << 1) |
    digitalRead(RR_ENC_B);

  uint8_t index = (prevRR << 2) | currentState;

  countRR += encoderTable[index];
  prevRR = currentState;

  portEXIT_CRITICAL_ISR(&encoderMux);
}

void IRAM_ATTR encoderISR_RL() {

  portENTER_CRITICAL_ISR(&encoderMux);

  uint8_t currentState =
    (digitalRead(RL_ENC_A) << 1) |
    digitalRead(RL_ENC_B);

  uint8_t index = (prevRL << 2) | currentState;

  countRL += encoderTable[index];
  prevRL = currentState;

  portEXIT_CRITICAL_ISR(&encoderMux);
}

// ==========================================================
// FEEDFORWARD PWM CALCULATION
// ==========================================================

float getFeedforwardPWM(
  float target,
  const int pwmTable[],
  const float rpmTable[],
  int tableSize
) {

  if (target <= 0) {
    return 0;
  }

  // Do not interpolate through the uncalibrated
  // zero-to-starting-PWM region.
  if (target <= rpmTable[1]) {
    return pwmTable[1];
  }

  if (target >= rpmTable[tableSize - 1]) {
    return MAX_PWM;
  }

  for (int i = 1; i < tableSize - 1; i++) {

    if (target >= rpmTable[i] &&
        target <= rpmTable[i + 1]) {

      float fraction =
        (target - rpmTable[i]) /
        (rpmTable[i + 1] - rpmTable[i]);

      return pwmTable[i] +
        fraction * (pwmTable[i + 1] - pwmTable[i]);
    }
  }

  return pwmTable[1];
}

// ==========================================================
// INDEPENDENT PI CONTROLLER (unchanged)
// ==========================================================

int calculatePI(
  float target,
  float actual,
  float kp,
  float ki,
  float &integral,
  const int pwmTable[],
  const float rpmTable[],
  int tableSize
) {

  if (target <= 0) {

    integral = 0;
    return 0;
  }

  float error = target - actual;

  float feedforward =
    getFeedforwardPWM(
      target,
      pwmTable,
      rpmTable,
      tableSize
    );

  // Candidate integral
  float candidateIntegral =
    constrain(
      integral + error * SAMPLE_TIME,
      -200.0,
      200.0
    );

  float candidateOutput =
    feedforward +
    kp * error +
    ki * candidateIntegral;

  // Conditional integration for anti-windup
  if (
    (candidateOutput >= 0 &&
     candidateOutput <= MAX_PWM) ||

    (candidateOutput > MAX_PWM && error < 0) ||

    (candidateOutput < 0 && error > 0)
  ) {

    integral = candidateIntegral;
  }

  float output =
    feedforward +
    kp * error +
    ki * integral;

  // Minimum calibrated PWM for each motor
  int minimumPWM = pwmTable[1];

  output = constrain(
    output,
    (float)minimumPWM,
    (float)MAX_PWM
  );

  return (int)round(output);
}

// ==========================================================
// MOTOR OUTPUT
// dir: +1 forward, -1 reverse, 0 = coast (bridge disabled)
// ==========================================================

void driveMotor(int wheel, int dir, int pwm) {

  int d = dir * MOTOR_DIR[wheel];

  if (d > 0) {

    digitalWrite(IN1_PIN[wheel], HIGH);
    digitalWrite(IN2_PIN[wheel], LOW);

  } else if (d < 0) {

    digitalWrite(IN1_PIN[wheel], LOW);
    digitalWrite(IN2_PIN[wheel], HIGH);

  } else {

    digitalWrite(IN1_PIN[wheel], LOW);
    digitalWrite(IN2_PIN[wheel], LOW);
    pwm = 0;
  }

  ledcWrite(EN_PIN[wheel], pwm);
}

// ==========================================================
// STOP ALL MOTORS
// ==========================================================

void stopAllMotors() {

  for (int i = 0; i < NUM_WHEELS; i++) {

    targetRPM[i] = 0;
    targetDir[i] = 0;
    pwmOut[i] = 0;
    integral[i] = 0;

    driveMotor(i, 0, 0);
  }
}

// ==========================================================
// ROS COMMAND HANDLING
// ==========================================================

// Apply one wheel command (speed magnitude in rad/s + sign 'p'/'n')
void applyWheelCommand(int wheel, char sign, float radPerSec) {

  float rpm = radPerSec * RAD_S_TO_RPM;

  if (rpm < MIN_TARGET_RPM) {

    // Below the minimum calibrated speed: stop this wheel
    targetRPM[wheel] = 0;
    targetDir[wheel] = 0;

  } else {

    targetRPM[wheel] = (rpm > MAX_TARGET_RPM) ? MAX_TARGET_RPM : rpm;
    targetDir[wheel] = (sign == 'p') ? 1 : -1;
  }

  lastCmdMs = millis();
}

// Non-blocking character-by-character parser for "<id><sign><value>,"
enum RxState { RX_ID, RX_SIGN, RX_VALUE, RX_SKIP };

RxState rxState = RX_ID;
int rxWheel = 0;
char rxSign = 'p';
char rxValue[12];
uint8_t rxLen = 0;

void readSerialInput() {

  while (Serial.available() > 0) {

    char c = (char)Serial.read();

    switch (rxState) {

      case RX_ID:

        if (c >= '1' && c <= '4') {

          rxWheel = c - '1';
          rxState = RX_SIGN;

        } else if (c == ',' || c == '\r' || c == '\n' || c == ' ') {

          // separators / line endings between tokens: ignore

        } else {

          rxState = RX_SKIP;   // garbage: resynchronise at next ','
        }
        break;

      case RX_SIGN:

        if (c == 'p' || c == 'n') {

          rxSign = c;
          rxLen = 0;
          rxState = RX_VALUE;

        } else {

          rxState = (c == ',') ? RX_ID : RX_SKIP;
        }
        break;

      case RX_VALUE:

        if (c == ',') {

          rxValue[rxLen] = '\0';

          char *endPointer;
          float value = strtof(rxValue, &endPointer);

          if (rxLen > 0 &&
              endPointer != rxValue &&
              *endPointer == '\0' &&
              isfinite(value) &&
              value >= 0.0f) {

            applyWheelCommand(rxWheel, rxSign, value);
          }

          rxState = RX_ID;

        } else if ((c >= '0' && c <= '9') || c == '.') {

          if (rxLen < sizeof(rxValue) - 1) {
            rxValue[rxLen++] = c;
          } else {
            rxState = RX_SKIP;   // too long: drop token
          }

        } else {

          rxState = RX_SKIP;
        }
        break;

      case RX_SKIP:

        if (c == ',') {
          rxState = RX_ID;
        }
        break;
    }
  }
}

// ==========================================================
// SETUP
// ==========================================================

void setup() {

  // Room for several 100 Hz command bursts
  Serial.setRxBufferSize(1024);
  Serial.begin(115200);

  // ------------------------------------------
  // MOTOR DRIVER OUTPUT PINS - safe state first
  // ------------------------------------------

  for (int i = 0; i < NUM_WHEELS; i++) {

    pinMode(IN1_PIN[i], OUTPUT);
    pinMode(IN2_PIN[i], OUTPUT);

    digitalWrite(IN1_PIN[i], LOW);
    digitalWrite(IN2_PIN[i], LOW);
  }

  // ------------------------------------------
  // ENCODER INPUT PINS
  // ------------------------------------------

  pinMode(FR_ENC_A, INPUT);
  pinMode(FR_ENC_B, INPUT);

  pinMode(FL_ENC_A, INPUT);
  pinMode(FL_ENC_B, INPUT);

  pinMode(RR_ENC_A, INPUT);
  pinMode(RR_ENC_B, INPUT);

  pinMode(RL_ENC_A, INPUT);
  pinMode(RL_ENC_B, INPUT);

  // ------------------------------------------
  // INITIALIZE ENCODER STATES
  // ------------------------------------------

  prevFR =
    (digitalRead(FR_ENC_A) << 1) |
    digitalRead(FR_ENC_B);

  prevFL =
    (digitalRead(FL_ENC_A) << 1) |
    digitalRead(FL_ENC_B);

  prevRR =
    (digitalRead(RR_ENC_A) << 1) |
    digitalRead(RR_ENC_B);

  prevRL =
    (digitalRead(RL_ENC_A) << 1) |
    digitalRead(RL_ENC_B);

  // ------------------------------------------
  // ATTACH ENCODER INTERRUPTS
  // ------------------------------------------

  attachInterrupt(
    digitalPinToInterrupt(FR_ENC_A),
    encoderISR_FR,
    CHANGE
  );

  attachInterrupt(
    digitalPinToInterrupt(FR_ENC_B),
    encoderISR_FR,
    CHANGE
  );

  attachInterrupt(
    digitalPinToInterrupt(FL_ENC_A),
    encoderISR_FL,
    CHANGE
  );

  attachInterrupt(
    digitalPinToInterrupt(FL_ENC_B),
    encoderISR_FL,
    CHANGE
  );

  attachInterrupt(
    digitalPinToInterrupt(RR_ENC_A),
    encoderISR_RR,
    CHANGE
  );

  attachInterrupt(
    digitalPinToInterrupt(RR_ENC_B),
    encoderISR_RR,
    CHANGE
  );

  attachInterrupt(
    digitalPinToInterrupt(RL_ENC_A),
    encoderISR_RL,
    CHANGE
  );

  attachInterrupt(
    digitalPinToInterrupt(RL_ENC_B),
    encoderISR_RL,
    CHANGE
  );

  // ------------------------------------------
  // CONFIGURE FOUR PWM CHANNELS
  // ------------------------------------------

  bool pwmOK = true;

  pwmOK &= ledcAttach(FR_EN, PWM_FREQ, PWM_RESOLUTION);
  pwmOK &= ledcAttach(FL_EN, PWM_FREQ, PWM_RESOLUTION);
  pwmOK &= ledcAttach(RR_EN, PWM_FREQ, PWM_RESOLUTION);
  pwmOK &= ledcAttach(RL_EN, PWM_FREQ, PWM_RESOLUTION);

  if (!pwmOK) {

    Serial.println("ERROR: PWM INITIALIZATION FAILED!");

    while (true) {
      delay(1000);
    }
  }

  // ------------------------------------------
  // INITIALIZE MOTORS
  // ------------------------------------------

  stopAllMotors();

  previousMillis = millis();
  lastCmdMs = millis();

#if DEBUG_TABLE
  Serial.println();
  Serial.println("DEBUG MODE - type e.g. 1p10.00,2p10.00,3p10.00,4p10.00,");
  Serial.println("TARGET RPM (FR FL RR RL) | MEASURED RPM (FR FL RR RL) | PWM (FR FL RR RL)");
#endif
}

// ==========================================================
// MAIN LOOP
// ==========================================================

void loop() {

  // Read commands from ROS (never blocks)
  readSerialInput();

  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= SAMPLE_TIME_MS) {

    previousMillis = currentMillis;

    // ----------------------------------------
    // WATCHDOG: host went silent -> stop
    // ----------------------------------------

    if (currentMillis - lastCmdMs > CMD_TIMEOUT_MS) {

      for (int i = 0; i < NUM_WHEELS; i++) {
        targetRPM[i] = 0;
        targetDir[i] = 0;
      }
    }

    // ----------------------------------------
    // READ FOUR ENCODER COUNTS
    // ----------------------------------------

    int32_t pulses[NUM_WHEELS];

    portENTER_CRITICAL(&encoderMux);

    pulses[FR] = countFR;
    pulses[FL] = countFL;
    pulses[RR] = countRR;
    pulses[RL] = countRL;

    // Reset counts for next sample
    countFR = 0;
    countFL = 0;
    countRR = 0;
    countRL = 0;

    portEXIT_CRITICAL(&encoderMux);

    // ----------------------------------------
    // WHEEL SPEEDS (signed, forward = positive)
    // ----------------------------------------

    for (int i = 0; i < NUM_WHEELS; i++) {

      rpmSigned[i] =
        ENC_SIGN[i] * (float)pulses[i] * 60.0f /
        (CPR[i] * SAMPLE_TIME);

      rpmAbs[i] = fabsf(rpmSigned[i]);
    }

    // ----------------------------------------
    // FOUR INDEPENDENT PI CONTROLLERS
    // ----------------------------------------

    for (int i = 0; i < NUM_WHEELS; i++) {

      // Stop requested (or below minimum speed, or watchdog)
      if (targetDir[i] == 0) {

        pwmOut[i] = 0;
        integral[i] = 0;
        driveMotor(i, 0, 0);
        continue;
      }

      // Direction change: coast down before flipping the L298N
      if (motorDir[i] != 0 &&
          motorDir[i] != targetDir[i] &&
          rpmAbs[i] > REVERSE_SETTLE_RPM) {

        pwmOut[i] = 0;
        integral[i] = 0;
        driveMotor(i, 0, 0);
        continue;
      }

      if (motorDir[i] != targetDir[i]) {
        integral[i] = 0;
      }

      motorDir[i] = targetDir[i];

      pwmOut[i] = calculatePI(
        targetRPM[i], rpmAbs[i],
        KP[i], KI[i], integral[i],
        PWM_TABLE[i], RPM_TABLE[i],
        TABLE_SIZE[i]
      );

      driveMotor(i, motorDir[i], pwmOut[i]);
    }

    // ----------------------------------------
    // REPORT TO ROS (or readable table in debug mode)
    // ----------------------------------------

#if DEBUG_TABLE

    for (int i = 0; i < NUM_WHEELS; i++) {
      Serial.print(targetDir[i] * targetRPM[i], 1);
      Serial.print(" ");
    }
    Serial.print("| ");

    for (int i = 0; i < NUM_WHEELS; i++) {
      Serial.print(rpmSigned[i], 1);
      Serial.print(" ");
    }
    Serial.print("| ");

    for (int i = 0; i < NUM_WHEELS; i++) {
      Serial.print(pwmOut[i]);
      Serial.print(" ");
    }
    Serial.println();

#else

    // "<id><sign><rad/s>," for all four wheels, then newline
    for (int i = 0; i < NUM_WHEELS; i++) {

      Serial.print(i + 1);
      Serial.print(rpmSigned[i] >= 0 ? 'p' : 'n');
      Serial.print(fabsf(rpmSigned[i]) * RPM_TO_RAD_S, 2);
      Serial.print(",");
    }
    Serial.println();

#endif
  }
}

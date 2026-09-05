#include <PID_v1.h>

/*
 * 4 independently driven/sensed wheels, one L298N per axle (rear board
 * already wired; front board added for the front wheels).
 *
 * Wheel indices used throughout (and in the serial protocol) match the
 * ros2_control joint order in amr_ros2_control.xacro:
 *   1 = front_right   2 = front_left   3 = rear_right   4 = rear_left
 *
 * Rear pins are unchanged from the 2-wheel version. Front pins use GPIOs
 * that are free on this board: GPIO21/22 are free because the MPU6050 is
 * read over I2C from the companion computer, not the ESP32. None of the
 * new pins are UART0 (1/3), SPI flash (6-11), or strapping pins (0/2/5/15).
 */

// ---- L298N #1 (rear) ----
#define L298N_R_enA 25  // PWM  rear-right
#define L298N_R_enB 13  // PWM  rear-left
#define L298N_R_in1 26  // dir  rear-right A
#define L298N_R_in2 27  // dir  rear-right B
#define L298N_R_in3 14  // dir  rear-left A
#define L298N_R_in4 12  // dir  rear-left B

// ---- L298N #2 (front) ----
#define L298N_F_enA 4   // PWM  front-right
#define L298N_F_enB 23  // PWM  front-left
#define L298N_F_in1 16  // dir  front-right A
#define L298N_F_in2 17  // dir  front-right B
#define L298N_F_in3 18  // dir  front-left A
#define L298N_F_in4 19  // dir  front-left B

// ---- Encoders ----
#define front_right_encoder_phaseA 36  // input-only
#define front_right_encoder_phaseB 39  // input-only
#define front_left_encoder_phaseA  21
#define front_left_encoder_phaseB  22
#define rear_right_encoder_phaseA  32
#define rear_right_encoder_phaseB  33
#define rear_left_encoder_phaseA   34  // input-only
#define rear_left_encoder_phaseB   35  // input-only

// Wheel array indices, matching the serial protocol id ('1'-'4') and the
// ros2_control joint order.
enum WheelIdx { FRONT_RIGHT = 0, FRONT_LEFT = 1, REAR_RIGHT = 2, REAR_LEFT = 3, NUM_WHEELS = 4 };

// Encoders (marked volatile for ISR safety)
volatile unsigned long encoder_counter[NUM_WHEELS] = {0, 0, 0, 0};
volatile char wheel_sign[NUM_WHEELS] = {'p', 'p', 'p', 'p'};

unsigned long last_millis = 0;
const unsigned long interval = 100;

// --- Serial command parser state ---
int cmd_wheel_idx = -1;          // which wheel the current command targets, -1 = none
bool is_wheel_forward[NUM_WHEELS] = {true, true, true, true};
char value[10] = "00.00";
uint8_t value_idx = 0;

// PID Setpoint / Measurement / Command, one triplet per wheel
double wheel_cmd_vel[NUM_WHEELS]  = {0.0, 0.0, 0.0, 0.0};   // rad/s, from host
double wheel_meas_vel[NUM_WHEELS] = {0.0, 0.0, 0.0, 0.0};   // rad/s, from encoder
double wheel_cmd[NUM_WHEELS]      = {0.0, 0.0, 0.0, 0.0};   // 0-255, PWM out

// PID Tuning Parameters - front wheels start with the same gains as the
// matching rear wheel; retune per-wheel once you have the real robot moving,
// since motor/gearbox variance means these will likely differ slightly.
double Kp_fr = 11.5, Ki_fr = 7.5, Kd_fr = 0.1;
double Kp_fl = 12.8, Ki_fl = 8.3, Kd_fl = 0.1;
double Kp_rr = 11.5, Ki_rr = 7.5, Kd_rr = 0.1;
double Kp_rl = 12.8, Ki_rl = 8.3, Kd_rl = 0.1;

// PID Controllers - kept as explicit named objects (rather than an array)
// since the PID_v1 library holds pointers into these doubles for the
// lifetime of the object; array-of-PID with per-element pointer wiring is
// easy to get subtly wrong, so this stays explicit like the 2-wheel version.
PID frontRightMotor(&wheel_meas_vel[FRONT_RIGHT], &wheel_cmd[FRONT_RIGHT], &wheel_cmd_vel[FRONT_RIGHT], Kp_fr, Ki_fr, Kd_fr, DIRECT);
PID frontLeftMotor (&wheel_meas_vel[FRONT_LEFT],  &wheel_cmd[FRONT_LEFT],  &wheel_cmd_vel[FRONT_LEFT],  Kp_fl, Ki_fl, Kd_fl, DIRECT);
PID rearRightMotor (&wheel_meas_vel[REAR_RIGHT],  &wheel_cmd[REAR_RIGHT],  &wheel_cmd_vel[REAR_RIGHT],  Kp_rr, Ki_rr, Kd_rr, DIRECT);
PID rearLeftMotor  (&wheel_meas_vel[REAR_LEFT],   &wheel_cmd[REAR_LEFT],   &wheel_cmd_vel[REAR_LEFT],   Kp_rl, Ki_rl, Kd_rl, DIRECT);

// ISR Prototypes with IRAM_ATTR
void IRAM_ATTR frontRightEncoderCallback();
void IRAM_ATTR frontLeftEncoderCallback();
void IRAM_ATTR rearRightEncoderCallback();
void IRAM_ATTR rearLeftEncoderCallback();

void setDirection(int wheel, bool forward);

void setup() {
  Serial.begin(115200);

  // Init L298N Pins
  pinMode(L298N_R_enA, OUTPUT);
  pinMode(L298N_R_enB, OUTPUT);
  pinMode(L298N_R_in1, OUTPUT);
  pinMode(L298N_R_in2, OUTPUT);
  pinMode(L298N_R_in3, OUTPUT);
  pinMode(L298N_R_in4, OUTPUT);

  pinMode(L298N_F_enA, OUTPUT);
  pinMode(L298N_F_enB, OUTPUT);
  pinMode(L298N_F_in1, OUTPUT);
  pinMode(L298N_F_in2, OUTPUT);
  pinMode(L298N_F_in3, OUTPUT);
  pinMode(L298N_F_in4, OUTPUT);

  // Default Forward Direction Setup
  digitalWrite(L298N_R_in1, HIGH);
  digitalWrite(L298N_R_in2, LOW);
  digitalWrite(L298N_R_in3, HIGH);
  digitalWrite(L298N_R_in4, LOW);
  digitalWrite(L298N_F_in1, HIGH);
  digitalWrite(L298N_F_in2, LOW);
  digitalWrite(L298N_F_in3, HIGH);
  digitalWrite(L298N_F_in4, LOW);

  // Init PID
  frontRightMotor.SetMode(AUTOMATIC);
  frontLeftMotor.SetMode(AUTOMATIC);
  rearRightMotor.SetMode(AUTOMATIC);
  rearLeftMotor.SetMode(AUTOMATIC);

  // Init Encoder Inputs
  // GPIO 34/35/36/39 are input-only and have no internal pull resistor.
  pinMode(front_right_encoder_phaseA, INPUT);
  pinMode(front_right_encoder_phaseB, INPUT);
  pinMode(front_left_encoder_phaseA, INPUT_PULLUP);
  pinMode(front_left_encoder_phaseB, INPUT_PULLUP);
  pinMode(rear_right_encoder_phaseA, INPUT_PULLUP);
  pinMode(rear_right_encoder_phaseB, INPUT_PULLUP);
  pinMode(rear_left_encoder_phaseA, INPUT);
  pinMode(rear_left_encoder_phaseB, INPUT);

  // Attach Interrupts
  attachInterrupt(digitalPinToInterrupt(front_right_encoder_phaseA), frontRightEncoderCallback, RISING);
  attachInterrupt(digitalPinToInterrupt(front_left_encoder_phaseA), frontLeftEncoderCallback, RISING);
  attachInterrupt(digitalPinToInterrupt(rear_right_encoder_phaseA), rearRightEncoderCallback, RISING);
  attachInterrupt(digitalPinToInterrupt(rear_left_encoder_phaseA), rearLeftEncoderCallback, RISING);
}

void loop() {
  // --- SERIAL COMMAND PARSER ---
  // Protocol: "<id><sign><value>," repeated per wheel, e.g.
  //   "1p12.34,2n05.67,3p00.00,4p00.00,"
  // id: '1'=front_right '2'=front_left '3'=rear_right '4'=rear_left
  if (Serial.available()) {
    char chr = Serial.read();

    if (chr >= '1' && chr <= '4') {
      cmd_wheel_idx = chr - '1';
      value_idx = 0;
    }
    else if (chr == 'p') {
      if (cmd_wheel_idx >= 0 && !is_wheel_forward[cmd_wheel_idx]) {
        setDirection(cmd_wheel_idx, true);
        is_wheel_forward[cmd_wheel_idx] = true;
      }
    }
    else if (chr == 'n') {
      if (cmd_wheel_idx >= 0 && is_wheel_forward[cmd_wheel_idx]) {
        setDirection(cmd_wheel_idx, false);
        is_wheel_forward[cmd_wheel_idx] = false;
      }
    }
    else if (chr == ',') {
      value[value_idx] = '\0';
      if (cmd_wheel_idx >= 0) {
        wheel_cmd_vel[cmd_wheel_idx] = atof(value);
      }
      value_idx = 0;
      memset(value, 0, sizeof(value));
      cmd_wheel_idx = -1;
    }
    else {
      if (value_idx < sizeof(value) - 1) {
        value[value_idx++] = chr;
      }
    }
  }

  // --- ODOMETRY & CONTROL LOOP ---
  unsigned long current_millis = millis();
  if (current_millis - last_millis >= interval) {
    last_millis = current_millis;

    // Atomic readout of encoder counts to avoid race conditions
    unsigned long ticks[NUM_WHEELS];
    noInterrupts();
    for (int i = 0; i < NUM_WHEELS; i++) {
      ticks[i] = encoder_counter[i];
      encoder_counter[i] = 0;
    }
    interrupts();

    // Velocity calculation: rad/s = (ticks in 100ms * 10) * (60 / 385 RPM) * 0.10472
    for (int i = 0; i < NUM_WHEELS; i++) {
      wheel_meas_vel[i] = (10.0 * ticks[i] * (60.0 / 385.0)) * 0.10472;
    }

    frontRightMotor.Compute();
    frontLeftMotor.Compute();
    rearRightMotor.Compute();
    rearLeftMotor.Compute();

    for (int i = 0; i < NUM_WHEELS; i++) {
      if (wheel_cmd_vel[i] == 0.0) wheel_cmd[i] = 0.0;
    }

    // Report back velocity string to ROS / Host, same id scheme as commands
    for (int i = 0; i < NUM_WHEELS; i++) {
      Serial.print(i + 1);
      Serial.print(wheel_sign[i]);
      Serial.print(wheel_meas_vel[i]);
      Serial.print(",");
    }
    Serial.println();

    analogWrite(L298N_F_enA, (int)wheel_cmd[FRONT_RIGHT]);
    analogWrite(L298N_F_enB, (int)wheel_cmd[FRONT_LEFT]);
    analogWrite(L298N_R_enA, (int)wheel_cmd[REAR_RIGHT]);
    analogWrite(L298N_R_enB, (int)wheel_cmd[REAR_LEFT]);
  }
}

// Drive the two direction pins for one wheel's L298N channel.
void setDirection(int wheel, bool forward) {
  switch (wheel) {
    case FRONT_RIGHT:
      digitalWrite(L298N_F_in1, forward ? HIGH : LOW);
      digitalWrite(L298N_F_in2, forward ? LOW : HIGH);
      break;
    case FRONT_LEFT:
      digitalWrite(L298N_F_in3, forward ? HIGH : LOW);
      digitalWrite(L298N_F_in4, forward ? LOW : HIGH);
      break;
    case REAR_RIGHT:
      digitalWrite(L298N_R_in1, forward ? HIGH : LOW);
      digitalWrite(L298N_R_in2, forward ? LOW : HIGH);
      break;
    case REAR_LEFT:
      digitalWrite(L298N_R_in3, forward ? HIGH : LOW);
      digitalWrite(L298N_R_in4, forward ? LOW : HIGH);
      break;
  }
}

// --- INTERRUPT SERVICE ROUTINES ---
void IRAM_ATTR frontRightEncoderCallback() {
  wheel_sign[FRONT_RIGHT] = (digitalRead(front_right_encoder_phaseB) == HIGH) ? 'p' : 'n';
  encoder_counter[FRONT_RIGHT]++;
}

void IRAM_ATTR frontLeftEncoderCallback() {
  wheel_sign[FRONT_LEFT] = (digitalRead(front_left_encoder_phaseB) == HIGH) ? 'n' : 'p';
  encoder_counter[FRONT_LEFT]++;
}

void IRAM_ATTR rearRightEncoderCallback() {
  wheel_sign[REAR_RIGHT] = (digitalRead(rear_right_encoder_phaseB) == HIGH) ? 'p' : 'n';
  encoder_counter[REAR_RIGHT]++;
}

void IRAM_ATTR rearLeftEncoderCallback() {
  wheel_sign[REAR_LEFT] = (digitalRead(rear_left_encoder_phaseB) == HIGH) ? 'n' : 'p';
  encoder_counter[REAR_LEFT]++;
}
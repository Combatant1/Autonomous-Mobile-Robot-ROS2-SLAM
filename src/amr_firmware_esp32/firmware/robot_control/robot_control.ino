#include <PID_v1.h>

// L298N H-Bridge Connection PINs
#define L298N_enA 25  // PWM (Right Motor)
#define L298N_enB 13  // PWM (Left Motor)
#define L298N_in1 26  // Dir Motor Right A
#define L298N_in2 27  // Dir Motor Right B
#define L298N_in3 14  // Dir Motor Left A
#define L298N_in4 12  // Dir Motor Left B

// Wheel Encoders Connection PINs
#define right_encoder_phaseA 32  // Interrupt 
#define right_encoder_phaseB 33  // Direction 
#define left_encoder_phaseA 34   // Interrupt (Input-only)
#define left_encoder_phaseB 35   // Interrupt (Input-only)

// Encoders (marked volatile for ISR safety)
volatile unsigned long right_encoder_counter = 0;
volatile unsigned long left_encoder_counter = 0;
volatile char right_wheel_sign = 'p';  // 'p' = positive, 'n' = negative
volatile char left_wheel_sign = 'p';   // 'p' = positive, 'n' = negative

unsigned long last_millis = 0;
const unsigned long interval = 100;

// Interpret Serial Messages
bool is_right_wheel_cmd = false;
bool is_left_wheel_cmd = false;
bool is_right_wheel_forward = true;
bool is_left_wheel_forward = true;

char value[10] = "00.00";
uint8_t value_idx = 0;
bool is_cmd_complete = false;

// PID Setpoint / Measurement / Command
double right_wheel_cmd_vel = 0.0;     // rad/s
double left_wheel_cmd_vel = 0.0;      // rad/s
double right_wheel_meas_vel = 0.0;    // rad/s
double left_wheel_meas_vel = 0.0;     // rad/s
double right_wheel_cmd = 0.0;         // 0-255
double left_wheel_cmd = 0.0;          // 0-255

// PID Tuning Parameters
double Kp_r = 11.5, Ki_r = 7.5, Kd_r = 0.1;
double Kp_l = 12.8, Ki_l = 8.3, Kd_l = 0.1;

// PID Controllers
PID rightMotor(&right_wheel_meas_vel, &right_wheel_cmd, &right_wheel_cmd_vel, Kp_r, Ki_r, Kd_r, DIRECT);
PID leftMotor(&left_wheel_meas_vel, &left_wheel_cmd, &left_wheel_cmd_vel, Kp_l, Ki_l, Kd_l, DIRECT);

// ISR Prototypes with IRAM_ATTR
void IRAM_ATTR rightEncoderCallback();
void IRAM_ATTR leftEncoderCallback();

void setup() {
  Serial.begin(115200);

  // Init L298N Pins
  pinMode(L298N_enA, OUTPUT);
  pinMode(L298N_enB, OUTPUT);
  pinMode(L298N_in1, OUTPUT);
  pinMode(L298N_in2, OUTPUT);
  pinMode(L298N_in3, OUTPUT);
  pinMode(L298N_in4, OUTPUT);

  // Default Forward Direction Setup
  digitalWrite(L298N_in1, HIGH);
  digitalWrite(L298N_in2, LOW);
  digitalWrite(L298N_in3, HIGH);
  digitalWrite(L298N_in4, LOW);

  // Init PID
  rightMotor.SetMode(AUTOMATIC);
  leftMotor.SetMode(AUTOMATIC);

  // Init Encoder Inputs
  pinMode(right_encoder_phaseA, INPUT_PULLUP);
  pinMode(right_encoder_phaseB, INPUT_PULLUP);
  pinMode(left_encoder_phaseA, INPUT); // GPIO 34/35 have no internal pullup
  pinMode(left_encoder_phaseB, INPUT);

  // Attach Interrupts
  attachInterrupt(digitalPinToInterrupt(right_encoder_phaseA), rightEncoderCallback, RISING);
  attachInterrupt(digitalPinToInterrupt(left_encoder_phaseA), leftEncoderCallback, RISING);
}

void loop() {
  // --- SERIAL COMMAND PARSER ---
  if (Serial.available()) {
    char chr = Serial.read();

    if (chr == 'r') {
      is_right_wheel_cmd = true;
      is_left_wheel_cmd = false;
      value_idx = 0;
      is_cmd_complete = false;
    }
    else if (chr == 'l') {
      is_right_wheel_cmd = false;
      is_left_wheel_cmd = true;
      value_idx = 0;
    }
    else if (chr == 'p') {
      if (is_right_wheel_cmd && !is_right_wheel_forward) {
        digitalWrite(L298N_in1, HIGH);
        digitalWrite(L298N_in2, LOW);
        is_right_wheel_forward = true;
      } else if (is_left_wheel_cmd && !is_left_wheel_forward) {
        digitalWrite(L298N_in3, HIGH);
        digitalWrite(L298N_in4, LOW);
        is_left_wheel_forward = true;
      }
    }
    else if (chr == 'n') {
      if (is_right_wheel_cmd && is_right_wheel_forward) {
        digitalWrite(L298N_in1, LOW);
        digitalWrite(L298N_in2, HIGH);
        is_right_wheel_forward = false;
      } else if (is_left_wheel_cmd && is_left_wheel_forward) {
        digitalWrite(L298N_in3, LOW);
        digitalWrite(L298N_in4, HIGH);
        is_left_wheel_forward = false;
      }
    }
    else if (chr == ',') {
      value[value_idx] = '\0';
      if (is_right_wheel_cmd) {
        right_wheel_cmd_vel = atof(value);
      } else if (is_left_wheel_cmd) {
        left_wheel_cmd_vel = atof(value);
        is_cmd_complete = true;
      }
      value_idx = 0;
      memset(value, 0, sizeof(value));
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
    noInterrupts();
    unsigned long right_ticks = right_encoder_counter;
    unsigned long left_ticks = left_encoder_counter;
    right_encoder_counter = 0;
    left_encoder_counter = 0;
    interrupts();

    // Velocity calculation: rad/s = (ticks in 100ms * 10) * (60 / 385 RPM) * 0.10472
    right_wheel_meas_vel = (10.0 * right_ticks * (60.0 / 385.0)) * 0.10472;
    left_wheel_meas_vel  = (10.0 * left_ticks * (60.0 / 385.0)) * 0.10472;

    rightMotor.Compute();
    leftMotor.Compute();

    if (right_wheel_cmd_vel == 0.0) right_wheel_cmd = 0.0;
    if (left_wheel_cmd_vel == 0.0) left_wheel_cmd = 0.0;

    // Report back velocity string to ROS / Host
    Serial.print("r");
    Serial.print(right_wheel_sign);
    Serial.print(right_wheel_meas_vel);
    Serial.print(",l");
    Serial.print(left_wheel_sign);
    Serial.print(left_wheel_meas_vel);
    Serial.println(",");

    analogWrite(L298N_enA, (int)right_wheel_cmd);
    analogWrite(L298N_enB, (int)left_wheel_cmd);
  }
}

// --- INTERRUPT SERVICE ROUTINES ---
void IRAM_ATTR rightEncoderCallback() {
  if (digitalRead(right_encoder_phaseB) == HIGH) {
    right_wheel_sign = 'p';
  } else {
    right_wheel_sign = 'n';
  }
  right_encoder_counter++;
}

void IRAM_ATTR leftEncoderCallback() {
  if (digitalRead(left_encoder_phaseB) == HIGH) {
    left_wheel_sign = 'n';
  } else {
    left_wheel_sign = 'p';
  }
  left_encoder_counter++;
}
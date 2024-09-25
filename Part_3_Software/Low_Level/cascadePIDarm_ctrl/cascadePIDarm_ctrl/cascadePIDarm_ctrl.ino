/* Arm Control Software
 *         ---------------------
 *        | Number | Motor Name | --> endpoints, data, analog value to physical angle conversion function, etc.
 *         ---------------------
 *        |   1    |  shoulder  | --> + forwards, - back, 600: 87 deg, 452: 47.85 deg, 627: 104.7 deg --> 3.7391x+274.16
 *         ---------------------
 *        |   2    |   elbow    | --> + down, - up, 300: 30 deg (full down), 580: 104 deg, 684: 133 deg --> 3.7391x+188.553
 *         ---------------------
 *        |   3    |  wrist rot | --> + CW (robot pov), - CCW: 486: 0 deg (middle), 133: -90 (CCW max), 815: 90 deg (CW max) --> 3.78889x+478
 *         ---------------------
 *        |   4    | wrist bend | --> + values go up, - values go down: 468 --> 180 deg, 126 --> 90 deg, 800 --> 270 deg --> 3.74444x-209.333, max = 508 (191.5 deg), min = 275 (129.3 deg)
 *         ---------------------
 *        |   5    | turntable  | --> + values go CCW, - values go CW
 *         --------------------- 
 *        |   6    |    claw    | --> 1200 (closed) --- 2100 (open)
 *         ---------------------
*/

#include <Servo.h>
#include "PID_Eggbot.h"

// Ring buffer size
#define BUF_SIZE 20
#define CHAR_BUF_SIZE 128

// Drop data into temp buffer
char input[CHAR_BUF_SIZE];
int input_idx = 0;

// Ring buffer pointer
// All sensors get updated synchronously
int buf_ptr = 0;

// Shoulder Motor parameters
#define S_LPWM 2  // Teensy Pin controlling LPWM
#define S_RPWM 3  // Teensy Pin controlling RPWM
#define S_FB A6   // Teensy Analog Pin reading potentiometer
// Sensor linear regression constants
#define S_A 3.72746
#define S_B 256.017
// Shoulder Motor PID parameters
double s_p_set, s_p_curr, s_p_prev;
double s_v_set, s_v_curr, s_out;
// Position Control Constants
double s_kpp = 0.0025;
double s_kpd = 0.0001;
// Velocity Control Constants
double s_kvp = 2000;
double s_kvi = 15000;
PID s_vPID(&s_v_curr, &s_out, &s_v_set, s_kvp, s_kvi, 0, DIRECT);
PID s_pPID(&s_p_curr, &s_v_set, &s_p_set, s_kpp, 0, s_kpd, DIRECT);
// Target angle
double s_angle;
// Ring Buffer for potentiometer data
int s_raw_fb[BUF_SIZE];

// Elbow Motor parameters
#define E_LPWM 4
#define E_RPWM 5
#define E_FB A7
// Sensor linear regression constants
#define E_A 3.61924
#define E_B 238.486
// Elbow Motor PID parameters
double e_p_set, e_p_curr, e_p_prev;
double e_v_set, e_v_curr, e_out;
// Position Control Constants
double e_kpp = 0.0023;
double e_kpd = 0.0003;
// Velocity Control Constants
double e_kvp = 4460;
double e_kvi = 450;
PID e_vPID(&e_v_curr, &e_out, &e_v_set, e_kvp, e_kvi, 0, DIRECT);
PID e_pPID(&e_p_curr, &e_v_set, &e_p_set, e_kpp, 0, e_kpd, REVERSE);
// Target angle
double e_angle;
// Ring Buffer for potentiometer data
int e_raw_fb[BUF_SIZE];

// Wrist Rotation Motor parameters
#define X_LPWM 6
#define X_RPWM 7
#define X_FB A8
// Sensor linear regression constants
#define X_A 3.76204
#define X_B 497
// Wrist Rotation Motor PID parameters
double x_p_set, x_p_curr, x_p_prev;
double x_v_set, x_v_curr, x_out;
// Position Control Constants
double x_kpp = 0.0023;
// Velocity Control Constants
double x_kvp = 1000.00;
double x_kvi = 3000.00;
PID x_vPID(&x_v_curr, &x_out, &x_v_set, x_kvp, x_kvi, 0, DIRECT);
PID x_pPID(&x_p_curr, &x_v_set, &x_p_set, x_kpp, 0, 0, DIRECT);
// Target angle
double x_angle;
// Ring Buffer for potentiometer data
int x_raw_fb[BUF_SIZE];

// Wrist Bend Motor parameters
#define Y_LPWM 8
#define Y_RPWM 9
#define Y_FB A9
// Sensor linear regression constants
#define Y_A 3.91947
#define Y_B 258.519
// Wrist Bend Motor PID parameters
double y_p_set, y_p_curr, y_p_prev;
double y_v_set, y_v_curr, y_out;
// Position Control Constants
double y_kpp = 0.002;
// Velocity Control Constants
double y_kvp = 750.00;
double y_kvi = 6000.00;
PID y_vPID(&y_v_curr, &y_out, &y_v_set, y_kvp, y_kvi, 0, DIRECT);
PID y_pPID(&y_p_curr, &y_v_set, &y_p_set, y_kpp, 0, 0, DIRECT);
// Target angle
double y_angle;
// Ring Buffer for potentiometer data
int y_raw_fb[BUF_SIZE];

// Turntable Motor parameters
#define R_LPWM 10
#define R_RPWM 11

// Servo parameters
#define CLAW 12        // Teensy Pin controlling Servo
#define CLAW_MIN 1200  // Minimum value for servo (fully closed)
#define CLAW_MAX 2100  // Maximum value for servo (fully open)
Servo claw;            // Constructor for Servo library

// Temp variables used to store the L_PWM and R_PWM pins of the H-bridge of interest
int left;
int right;

// Sets the minimum motor value (to get rid of the annoying high-pitched whining at low throttles)
int epsilon = 300;

// Turntable control command from KR260
int tt_ctrl = 0;

// Servo control command from KR260
int servo_ctrl = 2100;

// Loop Timer
unsigned long timer;
#define MIN_V 0.05

// Temp vars for Serial
double temp1, temp2, temp3, temp4;
int temp5, temp6;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  // Let's take advantage of the Teensy's 12-bit PWM resolution
  analogWriteResolution(12);

  // Declare the H-bridge PWM pins as outputs
  pinMode(S_LPWM, OUTPUT);
  pinMode(S_RPWM, OUTPUT);
  pinMode(E_LPWM, OUTPUT);
  pinMode(E_RPWM, OUTPUT);
  pinMode(X_LPWM, OUTPUT);
  pinMode(X_RPWM, OUTPUT);
  pinMode(Y_LPWM, OUTPUT);
  pinMode(Y_RPWM, OUTPUT);
  pinMode(R_LPWM, OUTPUT);
  pinMode(R_RPWM, OUTPUT);

  // Default set all motors to off
  analogWrite(S_LPWM, 0);
  analogWrite(S_RPWM, 0);
  analogWrite(E_LPWM, 0);
  analogWrite(E_RPWM, 0);
  analogWrite(X_LPWM, 0);
  analogWrite(X_RPWM, 0);
  analogWrite(Y_LPWM, 0);
  analogWrite(Y_RPWM, 0);
  analogWrite(R_LPWM, 0);
  analogWrite(R_RPWM, 0);

  // Initialize gripper servo
  claw.attach(12);

  // Initialize Serial communication
  Serial.begin(115200);
  //Serial.setTimeout(1);

  // Fill sensor ring buffers
  for (int i = 0; i < BUF_SIZE; ++i) {
    // Read all sensor values sequentially
    s_raw_fb[i] = analogRead(S_FB);
    e_raw_fb[i] = analogRead(E_FB);
    x_raw_fb[i] = analogRead(X_FB);
    y_raw_fb[i] = analogRead(Y_FB);
  }

  // Low-pass the raw values and set as current position
  getFilteredPos();

  // Set setpoint to current angle to prevent arm from moving
  s_p_set = s_p_curr;
  e_p_set = e_p_curr;
  x_p_set = x_p_curr;
  y_p_set = y_p_curr;

  // Set PID sample time
  s_pPID.SetSampleTime(10);
  e_pPID.SetSampleTime(10);
  x_pPID.SetSampleTime(10);
  y_pPID.SetSampleTime(10);
  s_vPID.SetSampleTime(10);
  e_vPID.SetSampleTime(10);
  x_vPID.SetSampleTime(10);
  y_vPID.SetSampleTime(10);

  // Set motor speed limits
  s_pPID.SetOutputLimits(-0.55, 0.55);
  e_pPID.SetOutputLimits(-0.55, 0.55);
  x_pPID.SetOutputLimits(-0.55, 0.55);
  y_pPID.SetOutputLimits(-0.55, 0.55);
  // Set motor power limits
  s_vPID.SetOutputLimits(-4096, 4096);
  e_vPID.SetOutputLimits(-4096, 4096);
  x_vPID.SetOutputLimits(-4096, 4096);
  y_vPID.SetOutputLimits(-4096, 4096);

  // Turn on PID
  s_pPID.SetMode(AUTOMATIC);
  e_pPID.SetMode(AUTOMATIC);
  x_pPID.SetMode(AUTOMATIC);
  y_pPID.SetMode(AUTOMATIC);
  s_vPID.SetMode(AUTOMATIC);
  e_vPID.SetMode(AUTOMATIC);
  x_vPID.SetMode(AUTOMATIC);
  y_vPID.SetMode(AUTOMATIC);

  timer = millis();

  Serial.clear();
}

void loop() {

  // Get new position commands from Serial and update setpoints -------------------------
  if (Serial.available() > 0) {
    char c = Serial.read();
    switch (c) {
      case 'p':
        // Toss the newline character out so it doesn't get confused
        c = Serial.read();
        // Fill sensor ring buffers
        for (int i = 0; i < BUF_SIZE; ++i) {
          // Read all sensor values sequentially
          s_raw_fb[i] = analogRead(S_FB);
          e_raw_fb[i] = analogRead(E_FB);
          x_raw_fb[i] = analogRead(X_FB);
          y_raw_fb[i] = analogRead(Y_FB);
        }
        // Low-pass the raw values and set as current position
        getFilteredPos();
        decodeAngles();
        break;
      case '\n':
        // parse the buffer
        // null character to terminate the string
        input[input_idx] = '\0';
        input_idx = 0;
        int filled;
        filled = sscanf(input, "%lf; %lf; %lf; %lf; %d; %d", &temp1, &temp2, &temp3, &temp4, &temp5, &temp6);
        if (filled == 6) {
          s_angle = temp1;
          e_angle = temp2;
          x_angle = temp3;
          y_angle = temp4;
          tt_ctrl = temp5;
          servo_ctrl = temp6;
        }

        // Constrain angle commands
        s_angle = clamp(s_angle, 0.0, 180.0);
        e_angle = clamp(e_angle, 20.0, 180.0);
        x_angle = clamp(x_angle, -90.0, 90.0);
        y_angle = clamp(y_angle, 90.0, 270.0);
        tt_ctrl = clamp(tt_ctrl, -4096, 4096);
        servo_ctrl = clamp(servo_ctrl, 600, 1500);

        // Convert angles to analog values
        s_p_set = (S_A * s_angle) + S_B;
        e_p_set = (E_A * e_angle) + E_B;
        x_p_set = (X_A * x_angle) + X_B;
        y_p_set = (Y_A * y_angle) - Y_B;

        s_vPID.resetInt();
        e_vPID.resetInt();
        x_vPID.resetInt();
        y_vPID.resetInt();

        break;
      default:
        // shove characters into buffer
        input[input_idx] = c;
        input_idx++;
        break;
    }
  }

  // Update current positions -----------------------------------------------------------
  s_raw_fb[buf_ptr] = analogRead(S_FB);
  e_raw_fb[buf_ptr] = analogRead(E_FB);
  x_raw_fb[buf_ptr] = analogRead(X_FB);
  y_raw_fb[buf_ptr] = analogRead(Y_FB);

  // Roll pointer back around if buffer is full
  if (buf_ptr < (BUF_SIZE - 1)) {
    ++buf_ptr;
  } else {
    buf_ptr = 0;
  }

  // Loop timer
  if ((millis() - timer) <= 10)
    return;
  timer = millis();

  // Low-pass the raw data and return positional values ---------------------------------
  getFilteredPos();

  // Compute PID ------------------------------------------------------------------------
  s_pPID.Compute();
  e_pPID.Compute();
  x_pPID.Compute();
  y_pPID.Compute();
  s_vPID.Compute();
  e_vPID.Compute();
  x_vPID.Compute();
  y_vPID.Compute();

  // Drive motors -----------------------------------------------------------------------
  drive(1, (int)s_out);
  drive(2, (int)e_out);
  drive(3, (int)x_out);
  drive(4, (int)y_out);
  drive(5, tt_ctrl);
  drive(6, servo_ctrl);
}

// Performs the low-pass filtering
void getFilteredPos() {
  // Temp Variables
  int s_temp = 0;
  int e_temp = 0;
  int x_temp = 0;
  int y_temp = 0;

  // Sum up all elements of array
  for (int i = 0; i < BUF_SIZE; ++i) {
    s_temp += s_raw_fb[i];
    e_temp += e_raw_fb[i];
    x_temp += x_raw_fb[i];
    y_temp += y_raw_fb[i];
  }

  s_p_prev = s_p_curr;
  e_p_prev = e_p_curr;
  x_p_prev = x_p_curr;
  y_p_prev = y_p_curr;

  // Divide by number of elements, and set as current values
  s_p_curr = ((double)s_temp) / ((double)BUF_SIZE);
  e_p_curr = ((double)e_temp) / ((double)BUF_SIZE);
  x_p_curr = ((double)x_temp) / ((double)BUF_SIZE);
  y_p_curr = ((double)y_temp) / ((double)BUF_SIZE);

  s_v_curr = (s_p_curr - s_p_prev) / 10;
  e_v_curr = (e_p_curr - e_p_prev) / 10;
  x_v_curr = (x_p_curr - x_p_prev) / 10;
  y_v_curr = (y_p_curr - y_p_prev) / 10;

  s_v_curr = (abs(s_v_curr) < MIN_V) ? 0 : s_v_curr;
  e_v_curr = (abs(e_v_curr) < MIN_V) ? 0 : e_v_curr;
  x_v_curr = (abs(x_v_curr) < MIN_V) ? 0 : x_v_curr;
  y_v_curr = (abs(y_v_curr) < MIN_V) ? 0 : y_v_curr;
}

// Drives the motors by sending commands to the associated H-bridges
void drive(int motor, int val) {
  // Identify the motor and look up its L_PWM and R_PWM pins
  switch (motor) {
    case 1:
      left = S_LPWM;
      right = S_RPWM;
      break;
    case 2:
      left = E_LPWM;
      right = E_RPWM;
      break;
    case 3:
      left = X_LPWM;
      right = X_RPWM;
      break;
    case 4:
      left = Y_LPWM;
      right = Y_RPWM;
      break;
    case 5:
      left = R_LPWM;
      right = R_RPWM;
      break;
    case 6:
      // Servo can be driven directly
      claw.writeMicroseconds(val);
      return;
    default:
      return;
      break;
  }

  // Check to make sure value isn't too small
  // Also sort the value based on its sign and drive motors accordingly
  // Look at Electrical: Power about how H-bridges work for more detail
  if (val > epsilon) {
    analogWrite(right, 0);
    analogWrite(left, val);
  } else if (val < -epsilon) {
    analogWrite(left, 0);
    analogWrite(right, abs(val));
  } else {
    analogWrite(right, 0);
    analogWrite(left, 0);
  }
}

// Converts potentiometer values to angle values
void decodeAngles() {
  double s_real, e_real, x_real, y_real;

  // Convert analog values to real angles
  s_real = (s_p_curr - S_B) / S_A;
  e_real = (e_p_curr - E_B) / E_A;
  x_real = (x_p_curr - X_B) / X_A;
  y_real = (y_p_curr + Y_B) / Y_A;

  // Send the angles!
  Serial.print(s_real);
  Serial.print(";");
  Serial.print(e_real);
  Serial.print(";");
  Serial.print(x_real);
  Serial.print(";");
  Serial.println(y_real);
}

double clamp(double input, double min, double max) {
  if (input < min) {
    return min;
  } else if (input > max) {
    return max;
  } else {
    return input;
  }
}

int clamp(int input, int min, int max) {
  if (input < min) {
    return min;
  } else if (input > max) {
    return max;
  } else {
    return input;
  }
}
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
#include <PID_v1.h>

// Ring buffer size
#define BUF_SIZE 20

// Ring buffer pointer
// All sensors get updated synchronously
int buf_ptr = 0;

// Shoulder Motor parameters
#define S_LPWM 2  // Teensy Pin controlling LPWM
#define S_RPWM 3  // Teensy Pin controlling RPWM
#define S_FB A6   // Teensy Analog Pin reading potentiometer
// Shoulder Motor PID parameters
double s_setpoint, s_curr, s_out;
double s_kp = 50.00;
double s_ki = 0.00;
double s_kd = 0.00;
PID s_PID(&s_curr, &s_out, &s_setpoint, s_kp, s_ki, s_kd, DIRECT);
// Target angle
double s_angle;
// Ring Buffer for potentiometer data
int s_raw_fb[BUF_SIZE];

// Elbow Motor parameters
#define E_LPWM 4
#define E_RPWM 5
#define E_FB A7
// Elbow Motor PID parameters
double e_setpoint, e_curr, e_out;
double e_kp = 40.00;
double e_ki = 1.00;
double e_kd = 0.00;
PID e_PID(&e_curr, &e_out, &e_setpoint, e_kp, e_ki, e_kd, REVERSE);
// Target angle
double e_angle;
// Ring Buffer for potentiometer data
int e_raw_fb[BUF_SIZE];

// Wrist Rotation Motor parameters
#define X_LPWM 6
#define X_RPWM 7
#define X_FB A8
// Wrist Rotation Motor PID parameters
double x_setpoint, x_curr, x_out;
double x_kp = 12.50;
double x_ki = 0.30;
double x_kd = 0.80;
PID x_PID(&x_curr, &x_out, &x_setpoint, x_kp, x_ki, x_kd, DIRECT);
// Target angle
double x_angle;
// Ring Buffer for potentiometer data
int x_raw_fb[BUF_SIZE];

// Wrist Bend Motor parameters
#define Y_LPWM 8
#define Y_RPWM 9
#define Y_FB A9
// Wrist Bend Motor PID parameters
double y_setpoint, y_curr, y_out;
double y_kp = 12.50;
double y_ki = 0.30;
double y_kd = 0.80;
PID y_PID(&y_curr, &y_out, &y_setpoint, y_kp, y_ki, y_kd, DIRECT);
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
  s_setpoint = s_curr;
  e_setpoint = e_curr;
  x_setpoint = x_curr;
  y_setpoint = y_curr;

  // Set motor power limits
  s_PID.SetOutputLimits(-4095, 4095);
  e_PID.SetOutputLimits(-4095, 4095);
  x_PID.SetOutputLimits(-4095, 4095);
  y_PID.SetOutputLimits(-2050, 2050);

  // Turn on PID
  s_PID.SetMode(AUTOMATIC);
  e_PID.SetMode(AUTOMATIC);
  x_PID.SetMode(AUTOMATIC);
  y_PID.SetMode(AUTOMATIC);

}

void loop() {
  // Get new position commands from Serial and update setpoints -------------------------
  if (Serial.available() > 0) {

    // Read input until newline character
    String data = Serial.readStringUntil('\n');

    // Drop data into temp buffer
    char input[64];
    data.toCharArray(input, data.length() + 1);

    // If the character p is read, the KR260 is polling current position
    // So return angles of all parts of the robot
    if (data == 'p') {
      decodeAngles();
    } else {
      // Temp variables for sscanf
      double temp1, temp2, temp3, temp4;
      int temp5, temp6;

      // Parse the data
      sscanf(input, "%lf; %lf; %lf; %lf; %d; %d", &temp1, &temp2, &temp3, &temp4, &temp5, &temp6);

      // Write data to control variables
      s_angle = temp1;
      e_angle = temp2;
      x_angle = temp3;
      y_angle = temp4;
      tt_ctrl = temp5;
      servo_ctrl = temp6;

      // Convert angles to analog values
      s_setpoint = (3.7391 * s_angle) + 274.16;
      e_setpoint = (3.7391 * e_angle) + 188.553;
      x_setpoint = (3.78889 * x_angle) + 478;
      y_setpoint = (3.74444 * y_angle) - 209.333;
    }
  }

  // Update current positions -----------------------------------------------------------
  s_raw_fb[buf_ptr] = analogRead(S_FB);
  e_raw_fb[buf_ptr] = analogRead(E_FB);
  x_raw_fb[buf_ptr] = analogRead(X_FB);
  y_raw_fb[buf_ptr] = analogRead(Y_FB);

  // Roll pointer back around if buffer is full
  if (buf_ptr < BUF_SIZE) {
    ++buf_ptr;
  } else {
    buf_ptr = 0;
  }

  // Low-pass the raw data and return positional values ---------------------------------
  getFilteredPos();

  // Compute PID ------------------------------------------------------------------------
  s_PID.Compute();
  e_PID.Compute();
  x_PID.Compute();
  y_PID.Compute();

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

  // Divide by number of elements, and set as current values
  s_curr = ((double)s_temp) / ((double)BUF_SIZE);
  e_curr = ((double)e_temp) / ((double)BUF_SIZE);
  x_curr = ((double)x_temp) / ((double)BUF_SIZE);
  y_curr = ((double)y_temp) / ((double)BUF_SIZE);
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
  s_real = (s_curr - 274.16) / 3.7391;
  e_real = (e_curr - 188.553) / 3.7391;
  x_real = (x_curr - 478) / 3.78889;
  y_real = (y_curr + 209.333) / 3.7444;

  // Send the angles!
  Serial.print(s_real);
  Serial.print(";");
  Serial.print(e_real);
  Serial.print(";");
  Serial.print(x_real);
  Serial.print(";");
  Serial.println(y_real);
}
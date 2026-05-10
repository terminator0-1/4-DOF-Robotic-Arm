#include <Encoder.h>
#include <Servo.h>

// ==========================================================
// NANO MOTOR 4 + ENCODER 4 + GRIPPER
// MEGA-COMPATIBLE VERSION
//
// Mega sends over Serial3:
//   ANGLE:3.14
//   HOME
//   STOP
//   RESET4
//   ENC4
//   ANGLE_NOW
//   OPEN_GRIPPER
//   CLOSE_GRIPPER
//
// Nano receives on Serial.
//
// FIXES:
// - Holds target during gripper close
// - Holds target for 5 seconds
// - Prints hold elapsed time
// - Prints when return home starts
// - Opens gripper before returning home
// - Keeps Timer0 normal so millis()/micros() work correctly
// ==========================================================

// -------------------- Encoder --------------------
Encoder enc4(3, 2);

// -------------------- Motor pins --------------------
const int M4_IN1 = 5;
const int M4_IN2 = 6;

// -------------------- Gripper Servo --------------------
Servo gripperServo;

const int GRIPPER_SERVO_PIN = 7;

const int GRIPPER_OPEN_ANGLE  = 0;
const int GRIPPER_CLOSE_ANGLE = 180;

const unsigned long SERVO_MOVE_TIME_MS = 3000;

bool gripperIsClosed = false;
bool servoAttached = false;
unsigned long servoCommandStart_ms = 0;

// Optional output flag pin
const int DONE_FLAG_PIN = 12;

// -------------------- Motor / encoder parameters --------------------
const double COUNTS_PER_REV4 = 48.0;
const double GEAR_RATIO4     = 73.82;
const double SUPPLY_VOLTAGE  = 12.0;

// -------------------- PID gains --------------------
double Kp4 = 4.5;
double Ki4 = 1.6;
double Kd4 = 0.9;

// -------------------- PWM behavior --------------------
const int MOVE_MIN_PWM4 = 45;
const int HOLD_MIN_PWM4 = 70;

const double POSITION_DEADBAND = 0.03;
const double HOLD_REGION_ERR   = 0.08;
const double TARGET_TOLERANCE  = 0.05;

// -------------------- Motion --------------------
double theta_final4 = 0.0;
double theta_des4   = 0.0;
double homeAngle4   = 0.0;

bool useRamp = false;
double rampRate4 = 0.9;

// -------------------- Controller state --------------------
double integral4 = 0.0;
double prevErr4  = 0.0;

// -------------------- Shutdown decline behavior --------------------
int shutdownPWM4 = 0;
const int PWM_DECLINE_STEP4 = 2;

// -------------------- Timing --------------------
const unsigned long START_DELAY_MS    = 700;
const unsigned long WAIT_AT_TARGET_MS = 5000;

unsigned long waitStart_ms = 0;
unsigned long moveStart_ms = 0;

// -------------------- Serial parsing --------------------
String incoming = "";

// -------------------- State machine --------------------
enum MotionState {
  IDLE,
  DELAY_BEFORE_TARGET,
  MOVING_TO_TARGET,
  WAITING_AT_TARGET,
  RETURNING_HOME,
  DECLINING_TO_ZERO,
  DONE
};

MotionState motionState = IDLE;

// ==========================================================
// Basic Helpers
// ==========================================================
int clampInt(int x, int lo, int hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

double clampDouble(double x, double lo, double hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

double applyDeadband(double x, double db) {
  if (fabs(x) < db) return 0.0;
  return x;
}

// ==========================================================
// Gripper Helpers
// ==========================================================
void commandServoNoDelay(int angleDeg) {
  angleDeg = clampInt(angleDeg, 0, 180);

  if (!servoAttached) {
    gripperServo.attach(GRIPPER_SERVO_PIN);
    servoAttached = true;
  }

  gripperServo.write(angleDeg);
  servoCommandStart_ms = millis();
}

void updateServoDetach() {
  if (servoAttached && millis() - servoCommandStart_ms >= SERVO_MOVE_TIME_MS) {
    gripperServo.detach();
    servoAttached = false;
    Serial.println("SERVO_DETACHED");
  }
}

void openGripper() {
  if (gripperIsClosed) {
    Serial.println("GRIPPER_OPEN");
  }

  commandServoNoDelay(GRIPPER_OPEN_ANGLE);
  gripperIsClosed = false;
}

void closeGripper() {
  if (!gripperIsClosed) {
    Serial.println("GRIPPER_CLOSED");
    commandServoNoDelay(GRIPPER_CLOSE_ANGLE);
    gripperIsClosed = true;
  }
}

// ==========================================================
// Encoder / Motor Helpers
// ==========================================================
long getEncoder4Count() {
  return enc4.read();
}

void resetEncoder4() {
  enc4.write(0);
}

double getAngle4() {
  long counts = enc4.read();
  return (counts * 2.0 * PI) / (COUNTS_PER_REV4 * GEAR_RATIO4);
}

void stopMotor4() {
  analogWrite(M4_IN1, 0);
  analogWrite(M4_IN2, 0);
}

void setMotor4Raw(int pwm) {
  pwm = clampInt(pwm, -255, 255);

  if (pwm > 0) {
    analogWrite(M4_IN1, pwm);
    analogWrite(M4_IN2, 0);
  }
  else if (pwm < 0) {
    analogWrite(M4_IN1, 0);
    analogWrite(M4_IN2, -pwm);
  }
  else {
    analogWrite(M4_IN1, 0);
    analogWrite(M4_IN2, 0);
  }
}

int mapSignedPWMContinuous(int pwmCmd, double err, int moveMin, int holdMin) {
  pwmCmd = clampInt(pwmCmd, -255, 255);

  if (pwmCmd == 0) return 0;

  int mag = abs(pwmCmd);

  if (fabs(err) > HOLD_REGION_ERR) {
    if (mag < 20) mag = 20;
    mag = map(mag, 20, 255, moveMin, 255);
  }
  else {
    if (mag < 8) mag = 8;
    mag = map(mag, 8, 255, holdMin, 255);
  }

  mag = clampInt(mag, 0, 255);

  if (pwmCmd < 0) return -mag;
  else return mag;
}

bool targetReached4(double theta4) {
  return fabs(theta_final4 - theta4) < TARGET_TOLERANCE;
}

bool homeReached4(double theta4) {
  return fabs(homeAngle4 - theta4) < TARGET_TOLERANCE;
}

// ==========================================================
// Input Validation
// ==========================================================
bool isValidNumberInput(String cmd) {
  cmd.trim();

  if (cmd.length() == 0) {
    return false;
  }

  bool hasDigit = false;
  bool hasDecimal = false;

  for (int i = 0; i < cmd.length(); i++) {
    char ch = cmd.charAt(i);

    if (isDigit(ch)) {
      hasDigit = true;
    }
    else if (ch == '.') {
      if (hasDecimal) return false;
      hasDecimal = true;
    }
    else if (ch == '-' || ch == '+') {
      if (i != 0) return false;
    }
    else {
      return false;
    }
  }

  return hasDigit;
}

// ==========================================================
// Motion Helpers
// ==========================================================
void echoReceivedTarget(double target) {
  Serial.print("RX_TARGET:");
  Serial.print(target, 6);
  Serial.print("\tRX_COUNT4:");
  Serial.println(getEncoder4Count());
}

void startMoveToTarget(double newTarget) {
  theta_final4 = newTarget;

  if (!useRamp) {
    theta_des4 = theta_final4;
  }
  else {
    theta_des4 = getAngle4();
  }

  integral4 = 0.0;
  prevErr4 = theta_final4 - getAngle4();

  digitalWrite(DONE_FLAG_PIN, LOW);

  if (gripperIsClosed) {
    openGripper();
  }

  moveStart_ms = millis();
  motionState = DELAY_BEFORE_TARGET;

  echoReceivedTarget(theta_final4);

  Serial.print("NEW_TARGET:");
  Serial.print(theta_final4, 6);
  Serial.print("\tCOUNT4:");
  Serial.println(getEncoder4Count());

  Serial.println("DELAYING_BEFORE_MOVE");
}

void startReturnHome() {
  theta_final4 = homeAngle4;

  if (!useRamp) {
    theta_des4 = theta_final4;
  }
  else {
    theta_des4 = getAngle4();
  }

  integral4 = 0.0;
  prevErr4 = theta_final4 - getAngle4();

  if (gripperIsClosed) {
    openGripper();
  }

  motionState = RETURNING_HOME;

  Serial.println("========== RETURN HOME STARTED ==========");
  Serial.print("homeAngle4:");
  Serial.println(homeAngle4, 6);
  Serial.print("currentAngle4:");
  Serial.println(getAngle4(), 6);
  Serial.print("theta_final4:");
  Serial.println(theta_final4, 6);
  Serial.print("errorToHome:");
  Serial.println(theta_final4 - getAngle4(), 6);
  Serial.println("=========================================");
}

void updateRampReference(double dt) {
  double maxStep4 = rampRate4 * dt;
  double diff4 = theta_final4 - theta_des4;

  if (diff4 > maxStep4) {
    theta_des4 += maxStep4;
  }
  else if (diff4 < -maxStep4) {
    theta_des4 -= maxStep4;
  }
  else {
    theta_des4 = theta_final4;
  }
}

// ==========================================================
// Serial Command Processing from Mega
// ==========================================================
void processCommand(String cmd) {
  cmd.trim();

  if (cmd.length() == 0) {
    return;
  }

  String upperCmd = cmd;
  upperCmd.toUpperCase();

  if (upperCmd.startsWith("ANGLE:")) {
    String val = cmd.substring(6);
    val.trim();

    if (!isValidNumberInput(val)) {
      Serial.print("IGNORED_INVALID_ANGLE:");
      Serial.println(val);
      return;
    }

    double target = val.toFloat();

    Serial.print("MEGA_CMD_ANGLE:");
    Serial.println(target, 6);

    startMoveToTarget(target);
  }
  else if (upperCmd == "HOME") {
    Serial.println("MEGA_CMD_HOME");
    startReturnHome();
  }
  else if (upperCmd == "STOP") {
    motionState = IDLE;
    stopMotor4();

    if (gripperIsClosed) {
      openGripper();
    }

    integral4 = 0.0;
    prevErr4 = 0.0;

    digitalWrite(DONE_FLAG_PIN, LOW);

    Serial.println("STOPPED");
  }
  else if (upperCmd == "RESET4") {
    resetEncoder4();

    theta_des4 = getAngle4();
    theta_final4 = theta_des4;
    homeAngle4 = theta_des4;

    integral4 = 0.0;
    prevErr4 = 0.0;

    motionState = IDLE;
    stopMotor4();

    if (gripperIsClosed) {
      openGripper();
    }

    Serial.println("ENCODER_RESET_OK");
    Serial.print("HOME_SET_TO:");
    Serial.println(homeAngle4, 6);
  }
  else if (upperCmd == "ENC4") {
    Serial.print("COUNT4:");
    Serial.println(getEncoder4Count());
  }
  else if (upperCmd == "ANGLE_NOW") {
    Serial.print("ANGLE_NOW:");
    Serial.print(getAngle4(), 6);
    Serial.print("\tCOUNT4:");
    Serial.println(getEncoder4Count());
  }
  else if (upperCmd == "OPEN_GRIPPER") {
    openGripper();
  }
  else if (upperCmd == "CLOSE_GRIPPER") {
    closeGripper();
  }
  else if (isValidNumberInput(cmd)) {
    double target = cmd.toFloat();

    Serial.print("USER_INPUT_ANGLE:");
    Serial.println(target, 6);

    startMoveToTarget(target);
  }
  else {
    Serial.print("UNKNOWN_CMD:");
    Serial.println(cmd);
  }
}

// ==========================================================
// Setup
// ==========================================================
void setup() {
  Serial.begin(115200);

  pinMode(M4_IN1, OUTPUT);
  pinMode(M4_IN2, OUTPUT);
  pinMode(DONE_FLAG_PIN, OUTPUT);

  digitalWrite(DONE_FLAG_PIN, LOW);

  stopMotor4();

  // Start gripper open, then detach later without blocking loop.
  commandServoNoDelay(GRIPPER_OPEN_ANGLE);
  gripperIsClosed = false;

  delay(500);
  updateServoDetach();

  theta_des4   = getAngle4();
  theta_final4 = theta_des4;
  homeAngle4   = theta_des4;
  prevErr4     = 0.0;

  motionState = IDLE;

  // Do NOT change Timer0 here.
  // Pins 5 and 6 use Timer0, but changing Timer0 breaks millis(), micros(), and delay().
  // TCCR0B = (TCCR0B & 0b11111000) | 0x02;

  Serial.println("NANO_MOTOR4_READY_FOR_MEGA");
  Serial.println("Waiting for Mega command: ANGLE:<value>");
}

// ==========================================================
// Loop
// ==========================================================
void loop() {
  updateServoDetach();

  // -------------------- Read commands from Mega --------------------
  while (Serial.available() > 0) {
    char c = Serial.read();

    if (c == '\n') {
      processCommand(incoming);
      incoming = "";
    }
    else if (c != '\r') {
      incoming += c;
    }
  }

  static unsigned long lastSample_us = 0;
  const unsigned long Ts_us = 1000;

  unsigned long now_us = micros();

  if (now_us - lastSample_us < Ts_us) {
    return;
  }

  lastSample_us = now_us;

  double dt = Ts_us / 1e6;
  double theta4 = getAngle4();
  long count4 = getEncoder4Count();

  if (motionState == IDLE) {
    stopMotor4();
    return;
  }

  if (motionState == DELAY_BEFORE_TARGET) {
    stopMotor4();

    if (millis() - moveStart_ms >= START_DELAY_MS) {
      motionState = MOVING_TO_TARGET;
      integral4 = 0.0;
      prevErr4 = theta_final4 - getAngle4();
      Serial.println("STARTING_MOVE");
    }

    return;
  }

  if (motionState == DECLINING_TO_ZERO) {
    if (shutdownPWM4 > 0) {
      shutdownPWM4 -= PWM_DECLINE_STEP4;
      if (shutdownPWM4 < 0) shutdownPWM4 = 0;
    }
    else if (shutdownPWM4 < 0) {
      shutdownPWM4 += PWM_DECLINE_STEP4;
      if (shutdownPWM4 > 0) shutdownPWM4 = 0;
    }

    if (shutdownPWM4 == 0) {
      stopMotor4();
      motionState = DONE;
      digitalWrite(DONE_FLAG_PIN, HIGH);

      Serial.print("HOME_REACHED");
      Serial.print("\tCOUNT4:");
      Serial.println(count4);
    }
    else {
      setMotor4Raw(shutdownPWM4);
    }

    return;
  }

  if (motionState == DONE) {
    stopMotor4();
    return;
  }

  if (motionState == MOVING_TO_TARGET || motionState == RETURNING_HOME) {
    if (useRamp) {
      updateRampReference(dt);
    }
    else {
      theta_des4 = theta_final4;
    }
  }
  else if (motionState == WAITING_AT_TARGET) {
    theta_des4 = theta_final4;
  }
  else {
    theta_des4 = theta_final4;
  }

  double err4  = applyDeadband(theta_des4 - theta4, POSITION_DEADBAND);
  double derr4 = (err4 - prevErr4) / dt;

  integral4 += err4 * dt;
  integral4 = clampDouble(integral4, -20.0, 20.0);

  double u4 = Kp4 * err4 + Ki4 * integral4 + Kd4 * derr4;

  int pwmCmd4 = clampInt((int)(u4 * 255.0 / SUPPLY_VOLTAGE), -255, 255);

  if (fabs(err4) < 0.01) {
    pwmCmd4 = 0;
  }

  int pwmOut4 = mapSignedPWMContinuous(
    pwmCmd4,
    err4,
    MOVE_MIN_PWM4,
    HOLD_MIN_PWM4
  );

  setMotor4Raw(pwmOut4);

  if (motionState == MOVING_TO_TARGET) {
    if (targetReached4(theta4)) {
      motionState = WAITING_AT_TARGET;
      waitStart_ms = millis();

      closeGripper();

      Serial.print("TARGET_REACHED");
      Serial.print("\tCOUNT4:");
      Serial.println(count4);
      Serial.println("HOLDING_TARGET_WITH_GRIPPER_CLOSED");
    }
  }
  else if (motionState == WAITING_AT_TARGET) {
    static int waitPrintDivider = 0;
    waitPrintDivider++;

    if (waitPrintDivider >= 200) {
      Serial.print("HOLDING...");
      Serial.print("\telapsed_ms:");
      Serial.print(millis() - waitStart_ms);
      Serial.print("\twaitTarget_ms:");
      Serial.print(WAIT_AT_TARGET_MS);
      Serial.print("\tTH4:");
      Serial.print(theta4, 6);
      Serial.print("\tFINAL4:");
      Serial.println(theta_final4, 6);
      waitPrintDivider = 0;
    }

    if (millis() - waitStart_ms >= WAIT_AT_TARGET_MS) {
      Serial.println("HOLD_DONE_STARTING_RETURN_HOME");
      startReturnHome();
    }
  }
  else if (motionState == RETURNING_HOME) {
    if (homeReached4(theta4)) {
      motionState = DECLINING_TO_ZERO;
      shutdownPWM4 = pwmOut4;

      Serial.print("HOME_POSITION_REACHED_START_DECLINE");
      Serial.print("\tCOUNT4:");
      Serial.print(count4);
      Serial.print("\tSTART_PWM:");
      Serial.println(shutdownPWM4);
    }
  }

  prevErr4 = err4;

  static int printDivider = 0;
  printDivider++;

  if (printDivider >= 20) {
    Serial.print("STATE:");

    if (motionState == IDLE) Serial.print("IDLE");
    else if (motionState == DELAY_BEFORE_TARGET) Serial.print("DELAY_BEFORE_TARGET");
    else if (motionState == MOVING_TO_TARGET) Serial.print("MOVING_TO_TARGET");
    else if (motionState == WAITING_AT_TARGET) Serial.print("WAITING_AT_TARGET");
    else if (motionState == RETURNING_HOME) Serial.print("RETURNING_HOME");
    else if (motionState == DECLINING_TO_ZERO) Serial.print("DECLINING_TO_ZERO");
    else if (motionState == DONE) Serial.print("DONE");

    Serial.print("\tCOUNT4:");
    Serial.print(count4);

    Serial.print("\tTH4:");
    Serial.print(theta4, 6);

    Serial.print("\tDES4:");
    Serial.print(theta_des4, 6);

    Serial.print("\tFINAL4:");
    Serial.print(theta_final4, 6);

    Serial.print("\tERR4:");
    Serial.print(err4, 6);

    Serial.print("\tPWM4:");
    Serial.print(pwmOut4);

    Serial.print("\tshutdownPWM4:");
    Serial.print(shutdownPWM4);

    Serial.print("\tgripperClosed:");
    Serial.print(gripperIsClosed);

    Serial.print("\tservoAttached:");
    Serial.println(servoAttached);

    printDivider = 0;
  }
}

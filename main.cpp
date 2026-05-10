#include <Encoder.h>

// ==========================================================
// MEGA CODE FOR MOTORS 1,2,3 + NANO MOTOR 4
//
// - Mega controls joints 1,2,3 locally
// - Mega sends joint 4 desired angle to Nano over Serial3
// - Nano handles all motor 4 control locally
// - Grabber is controlled by digital output pin 51
// - Nano raises a flag on pin 22 when it is time to grab
// - Fixed gravity bias only
// - Gravity bias does NOT change during return home
// - No braking
// ==========================================================

// -------------------- Pins --------------------
const int ENC_A1 = 20;
const int ENC_B1 = 21;
const int M1_IN1 = 12;
const int M1_IN2 = 5;

const int ENC_A2 = 18;
const int ENC_B2 = 19;
const int M2_IN1 = 10;
const int M2_IN2 = 11;

const int ENC_A3 = 2;
const int ENC_B3 = 3;
const int M3_IN1 = 45;
const int M3_IN2 = 44;

// -------------------- Nano flag + grab output --------------------
const int GRAB_FLAG_INPUT_PIN = 22;
const int GRAB_OUTPUT_PIN = 51;

bool grabTriggered = false;

// -------------------- Encoders --------------------
Encoder enc1(ENC_B1, ENC_A1);
Encoder enc2(ENC_B2, ENC_A2);
Encoder enc3(ENC_B3, ENC_A3);

// -------------------- Motor / Encoder parameters --------------------
const double COUNTS_PER_REV = 64.0;
const double COUNTS_PER_REV3 = 48.0;
const double GEAR_RATIO = 410.32;
const double GEAR_RATIO3 = 240.2;
const double SUPPLY_VOLTAGE = 12.0;

// -------------------- PID gains --------------------
double Kp1 = 4.0, Ki1 = 2.5, Kd1 = 0.9;
double Kp2 = 5.4, Ki2 = 1.5, Kd2 = 0.9;
double Kp3 = 3.0, Ki3 = 1.5, Kd3 = 0.4;

// -------------------- Fixed Gravity Bias --------------------
// These values stay the same in every state.
// No special return-home gravity compensation.
double gravityBias1 = 2.5;
double gravityBias2 = 3.0;
double gravityBias3 = 0.0;

// -------------------- Motion command --------------------
double theta_final1 = 0.0;
double theta_final2 = 0.0;
double theta_final3 = 0.0;
double theta_final4 = 0.0;

double theta_des1 = 0.0;
double theta_des2 = 0.0;
double theta_des3 = 0.0;

// -------------------- Home positions --------------------
const double HOME_ANGLE1 = 0.1;
const double HOME_ANGLE2 = 0.0;
const double HOME_ANGLE3 = 0.0;

double target_out1 = 0.0;
double target_out2 = 0.0;
double target_out3 = 0.0;

bool useRamp = true;
double rampRate1 = 3.0;
double rampRate2 = 2.5;
double rampRate3 = 2.0;

// -------------------- Controller state --------------------
double integral1 = 0.0, integral2 = 0.0, integral3 = 0.0;
double prevErr1 = 0.0, prevErr2 = 0.0, prevErr3 = 0.0;

// -------------------- PWM behavior --------------------
const int MOVE_MIN_PWM1 = 100;
const int HOLD_MIN_PWM1 = 85;

const int MOVE_MIN_PWM2 = 90;
const int HOLD_MIN_PWM2 = 85;

const int MOVE_MIN_PWM3 = 120;
const int HOLD_MIN_PWM3 = 100;

const double HOLD_REGION_ERR = 0.1;
const double POSITION_DEADBAND = 0.002;

// Separate tolerances
const double TARGET_TOLERANCE1 = 0.05;
const double TARGET_TOLERANCE2 = 0.05;
const double TARGET_TOLERANCE3 = 0.05;

// -------------------- Motion state machine --------------------
enum MotionState {
  MOVING_TO_TARGET,
  WAITING_AT_TARGET,
  RETURNING_HOME,
  DONE
};

MotionState motionState = MOVING_TO_TARGET;
unsigned long waitStart_ms = 0;
const unsigned long WAIT_BEFORE_HOME_MS = 5000;

// -------------------- Nano serial state --------------------
String nanoIncoming = "";
bool nanoTargetReached = false;
bool nanoHomeReached = false;

// ==========================================================
// Helpers
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

double getAngle1() {
  long counts = enc1.read();
  return (counts * 2.0 * PI) / (COUNTS_PER_REV * GEAR_RATIO);
}

double getAngle2() {
  long counts = enc2.read();
  return (counts * 2.0 * PI) / (COUNTS_PER_REV * GEAR_RATIO);
}

double getAngle3() {
  long counts = enc3.read();
  return (counts * 2.0 * PI) / (COUNTS_PER_REV3 * GEAR_RATIO3);
}

double readNumberFromSerial(const char* prompt) {
  Serial.println(prompt);

  while (true) {
    while (Serial.available() == 0) {}

    String s = Serial.readStringUntil('\n');
    s.trim();

    if (s.length() > 0) {
      return s.toFloat();
    }
  }
}

void waitForEnterToStart() {
  Serial.println("Move joints to zero position, then press Enter.");

  while (Serial.available()) Serial.read();
  while (Serial.available() == 0) {}
  while (Serial.available()) Serial.read();
}

void closeGrabber() {
  digitalWrite(GRAB_OUTPUT_PIN, HIGH);
}

void openGrabber() {
  digitalWrite(GRAB_OUTPUT_PIN, LOW);
}

// ==========================================================
// Ramp Reference Update
// ==========================================================
void updateRampReference(double dt) {
  double maxStep1 = rampRate1 * dt;
  double maxStep2 = rampRate2 * dt;
  double maxStep3 = rampRate3 * dt;

  double diff1 = theta_final1 - theta_des1;
  double diff2 = theta_final2 - theta_des2;
  double diff3 = theta_final3 - theta_des3;

  if (diff1 > maxStep1) theta_des1 += maxStep1;
  else if (diff1 < -maxStep1) theta_des1 -= maxStep1;
  else theta_des1 = theta_final1;

  if (diff2 > maxStep2) theta_des2 += maxStep2;
  else if (diff2 < -maxStep2) theta_des2 -= maxStep2;
  else theta_des2 = theta_final2;

  if (diff3 > maxStep3) theta_des3 += maxStep3;
  else if (diff3 < -maxStep3) theta_des3 -= maxStep3;
  else theta_des3 = theta_final3;
}

// ==========================================================
// Motor Output
// ==========================================================
void setMotorRaw(int pwm1, int pwm2, int pwm3) {
  pwm1 = clampInt(pwm1, -255, 255);
  pwm2 = clampInt(pwm2, -255, 255);
  pwm3 = clampInt(pwm3, -255, 255);

  if (pwm1 > 0) {
    analogWrite(M1_IN1, pwm1);
    analogWrite(M1_IN2, 0);
  } else if (pwm1 < 0) {
    analogWrite(M1_IN1, 0);
    analogWrite(M1_IN2, -pwm1);
  } else {
    analogWrite(M1_IN1, 0);
    analogWrite(M1_IN2, 0);
  }

  if (pwm2 > 0) {
    analogWrite(M2_IN1, pwm2);
    analogWrite(M2_IN2, 0);
  } else if (pwm2 < 0) {
    analogWrite(M2_IN1, 0);
    analogWrite(M2_IN2, -pwm2);
  } else {
    analogWrite(M2_IN1, 0);
    analogWrite(M2_IN2, 0);
  }

  if (pwm3 > 0) {
    analogWrite(M3_IN1, pwm3);
    analogWrite(M3_IN2, 0);
  } else if (pwm3 < 0) {
    analogWrite(M3_IN1, 0);
    analogWrite(M3_IN2, -pwm3);
  } else {
    analogWrite(M3_IN1, 0);
    analogWrite(M3_IN2, 0);
  }
}

int mapSignedPWMContinuous(int pwmCmd, double err, int moveMin, int holdMin) {
  pwmCmd = clampInt(pwmCmd, -255, 255);

  if (pwmCmd == 0) return 0;

  int mag = abs(pwmCmd);

  if (fabs(err) > HOLD_REGION_ERR) {
    if (mag < 20) mag = 20;
    mag = map(mag, 20, 255, moveMin, 255);
  } else {
    if (mag < 8) mag = 8;
    mag = map(mag, 8, 255, holdMin, 255);
  }

  mag = clampInt(mag, 0, 255);

  return (pwmCmd < 0) ? -mag : mag;
}

// ==========================================================
// Nano Commands
// ==========================================================
void sendAngle4ToNano(double angle4) {
  nanoTargetReached = false;
  nanoHomeReached = false;

  Serial3.print("ANGLE:");
  Serial3.println(angle4, 6);
}

void sendHome4ToNano() {
  nanoHomeReached = false;
  Serial3.println("HOME");
}

void sendStop4ToNano() {
  Serial3.println("STOP");
}

void requestAngle4Now() {
  Serial3.println("ANGLE_NOW");
}

void processNanoMessage(String msg) {
  msg.trim();

  if (msg.length() == 0) return;

  Serial.print("Nano: ");
  Serial.println(msg);

  if (msg.startsWith("TARGET_REACHED")) {
    nanoTargetReached = true;
  } else if (msg.startsWith("HOME_REACHED")) {
    nanoHomeReached = true;
  }
}

void readNanoSerial() {
  while (Serial3.available() > 0) {
    char c = Serial3.read();

    if (c == '\n') {
      processNanoMessage(nanoIncoming);
      nanoIncoming = "";
    } else if (c != '\r') {
      nanoIncoming += c;
    }
  }
}

// ==========================================================
// State Helpers
// ==========================================================
bool targetReached(double t1, double t2, double t3) {
  return (
    fabs(theta_final1 - t1) < TARGET_TOLERANCE1 &&
    fabs(theta_final2 - t2) < TARGET_TOLERANCE2 &&
    fabs(theta_final3 - t3) < TARGET_TOLERANCE3
  );
}

void startReturnHome() {
  theta_final1 = HOME_ANGLE1;
  theta_final2 = HOME_ANGLE2;
  theta_final3 = HOME_ANGLE3;

  if (!useRamp) {
    theta_des1 = theta_final1;
    theta_des2 = theta_final2;
    theta_des3 = theta_final3;
  }

  integral1 = 0.0;
  integral2 = 0.0;
  integral3 = 0.0;

  prevErr1 = theta_final1 - getAngle1();
  prevErr2 = theta_final2 - getAngle2();
  prevErr3 = theta_final3 - getAngle3();

  sendHome4ToNano();

  motionState = RETURNING_HOME;

  Serial.print("Returning home: ");
  Serial.print(theta_final1, 4);
  Serial.print(", ");
  Serial.print(theta_final2, 4);
  Serial.print(", ");
  Serial.println(theta_final3, 4);
}

// ==========================================================
// Setup
// ==========================================================
void setup() {
  Serial.begin(115200);
  Serial3.begin(115200);

  pinMode(M1_IN1, OUTPUT);
  pinMode(M1_IN2, OUTPUT);
  pinMode(M2_IN1, OUTPUT);
  pinMode(M2_IN2, OUTPUT);
  pinMode(M3_IN1, OUTPUT);
  pinMode(M3_IN2, OUTPUT);

  pinMode(GRAB_FLAG_INPUT_PIN, INPUT);
  pinMode(GRAB_OUTPUT_PIN, OUTPUT);
  openGrabber();

  setMotorRaw(0, 0, 0);

  target_out1 = readNumberFromSerial("Enter desired joint 1 angle in radians:");
  target_out2 = readNumberFromSerial("Enter desired joint 2 angle in radians:");
  target_out3 = readNumberFromSerial("Enter desired joint 3 angle in radians:");
  theta_final4 = readNumberFromSerial("Enter desired joint 4 angle in radians:");
  useRamp = (readNumberFromSerial("Use ramp? Enter 1 for yes, 0 for step:") > 0.5);

  theta_final1 = target_out1;
  theta_final2 = target_out2;
  theta_final3 = target_out3;

  waitForEnterToStart();

  enc1.write(0);
  enc2.write(0);
  enc3.write(0);

  double theta1_0 = getAngle1();
  double theta2_0 = getAngle2();
  double theta3_0 = getAngle3();

  if (useRamp) {
    theta_des1 = theta1_0;
    theta_des2 = theta2_0;
    theta_des3 = theta3_0;
  } else {
    theta_des1 = theta_final1;
    theta_des2 = theta_final2;
    theta_des3 = theta_final3;
  }

  integral1 = 0.0;
  integral2 = 0.0;
  integral3 = 0.0;

  prevErr1 = theta_des1 - theta1_0;
  prevErr2 = theta_des2 - theta2_0;
  prevErr3 = theta_des3 - theta3_0;

  delay(1500);

  sendAngle4ToNano(theta_final4);
  delay(50);
  sendAngle4ToNano(theta_final4);
  delay(50);
  sendAngle4ToNano(theta_final4);

  Serial.println("Mega started. Angle 4 sent to Nano.");

  TCCR2B = (TCCR2B & 0b11111000) | 0x01;
  TCCR1B = (TCCR1B & 0b11111000) | 0x01;
  TCCR4B = (TCCR4B & 0b11111000) | 0x01;
  TCCR5B = (TCCR5B & 0b11111000) | 0x01;

  motionState = MOVING_TO_TARGET;
  closeGrabber();
}

// ==========================================================
// Loop
// ==========================================================
void loop() {
  readNanoSerial();

  if (motionState == DONE) {
    setMotorRaw(0, 0, 0);
    return;
  }

  static unsigned long lastSample_us = 0;
  const unsigned long Ts_us = 1000;

  unsigned long now_us = micros();

  if (now_us - lastSample_us < Ts_us) {
    return;
  }

  lastSample_us = now_us;

  double dt = Ts_us / 1e6;

  double theta1 = getAngle1();
  double theta2 = getAngle2();
  double theta3 = getAngle3();

  if (motionState == MOVING_TO_TARGET || motionState == RETURNING_HOME) {
    if (useRamp) {
      updateRampReference(dt);
    } else {
      theta_des1 = theta_final1;
      theta_des2 = theta_final2;
      theta_des3 = theta_final3;
    }
  } else {
    theta_des1 = theta_final1;
    theta_des2 = theta_final2;
    theta_des3 = theta_final3;
  }

  double err1 = applyDeadband(theta_des1 - theta1, POSITION_DEADBAND);
  double err2 = applyDeadband(theta_des2 - theta2, POSITION_DEADBAND);
  double err3 = applyDeadband(theta_des3 - theta3, POSITION_DEADBAND);

  double derr1 = (err1 - prevErr1) / dt;
  double derr2 = (err2 - prevErr2) / dt;
  double derr3 = (err3 - prevErr3) / dt;

  integral1 += err1 * dt;
  integral2 += err2 * dt;
  integral3 += err3 * dt;

  integral1 = clampDouble(integral1, -30.0, 30.0);
  integral2 = clampDouble(integral2, -30.0, 30.0);
  integral3 = clampDouble(integral3, -30.0, 30.0);

  double u1 = Kp1 * err1 + Ki1 * integral1 + Kd1 * derr1;
  double u2 = Kp2 * err2 + Ki2 * integral2 + Kd2 * derr2;
  double u3 = Kp3 * err3 + Ki3 * integral3 + Kd3 * derr3;

  // -------------------- Fixed Gravity Bias --------------------
  // Same gravity bias is used in all states.
  // It does not change during return home.
  double motionDir1 = theta_des1 - theta1;
  double motionDir2 = theta_des2 - theta2;
  double motionDir3 = theta_des3 - theta3;

  if (motionDir1 > POSITION_DEADBAND) {
    u1 += gravityBias1;
  } else if (motionDir1 < -POSITION_DEADBAND) {
    u1 -= gravityBias1;
  }

  if (motionDir2 > POSITION_DEADBAND) {
    u2 += gravityBias2;
  } else if (motionDir2 < -POSITION_DEADBAND) {
    u2 -= gravityBias2;
  }

  if (motionDir3 > POSITION_DEADBAND) {
    u3 += gravityBias3;
  } else if (motionDir3 < -POSITION_DEADBAND) {
    u3 -= gravityBias3;
  }

  int pwmCmd1 = clampInt((int)(u1 * 255.0 / SUPPLY_VOLTAGE), -255, 255);
  int pwmCmd2 = clampInt((int)(u2 * 255.0 / SUPPLY_VOLTAGE), -255, 255);
  int pwmCmd3 = clampInt((int)(u3 * 255.0 / SUPPLY_VOLTAGE), -255, 255);

  if (fabs(err1) < 0.01) {
    pwmCmd1 = 0;
  }

  if (fabs(err2) < 0.01) {
    pwmCmd2 = 0;
  }

  if (fabs(err3) < 0.01) {
    pwmCmd3 = 0;
  }

  int pwmOut1 = mapSignedPWMContinuous(pwmCmd1, err1, MOVE_MIN_PWM1, HOLD_MIN_PWM1);
  int pwmOut2 = mapSignedPWMContinuous(pwmCmd2, err2, MOVE_MIN_PWM2, HOLD_MIN_PWM2);
  int pwmOut3 = mapSignedPWMContinuous(pwmCmd3, err3, MOVE_MIN_PWM3, HOLD_MIN_PWM3);

  setMotorRaw(pwmOut1, pwmOut2, pwmOut3);

  // Grabber logic from Nano hardware flag
  if (!grabTriggered && digitalRead(GRAB_FLAG_INPUT_PIN) == HIGH) {
    closeGrabber();
    grabTriggered = true;
    Serial.println("Grab flag received. Pin 51 set HIGH.");
  }

  if (motionState == MOVING_TO_TARGET) {
    if (targetReached(theta1, theta2, theta3)) {
      motionState = WAITING_AT_TARGET;
      waitStart_ms = millis();

      Serial.println("Target reached. Waiting 5 seconds...");
    }
  } 
  else if (motionState == WAITING_AT_TARGET) {
    if (millis() - waitStart_ms >= WAIT_BEFORE_HOME_MS) {
      startReturnHome();
    }
  } 
  else if (motionState == RETURNING_HOME) {
    if (targetReached(theta1, theta2, theta3)) {
      setMotorRaw(0, 0, 0);
      sendStop4ToNano();

      motionState = DONE;

      Serial.println("Mega joints home reached. Motors shut off. No braking.");
    }
  }

  prevErr1 = err1;
  prevErr2 = err2;
  prevErr3 = err3;

  static int printDecim = 0;
  printDecim++;

  if (printDecim >= 10) {
    Serial.print("state=");

    if (motionState == MOVING_TO_TARGET) {
      Serial.print("MOVING_TO_TARGET");
    } else if (motionState == WAITING_AT_TARGET) {
      Serial.print("WAITING_AT_TARGET");
    } else if (motionState == RETURNING_HOME) {
      Serial.print("RETURNING_HOME");
    } else if (motionState == DONE) {
      Serial.print("DONE");
    }

    Serial.print("\tth1=");
    Serial.print(theta1, 6);

    Serial.print("\tdes1=");
    Serial.print(theta_des1, 6);

    Serial.print("\tfinal1=");
    Serial.print(theta_final1, 6);

    Serial.print("\terr1=");
    Serial.print(err1, 6);

    Serial.print("\tu1=");
    Serial.print(u1, 6);

    Serial.print("\tpwm1=");
    Serial.print(pwmOut1);

    Serial.print("\tgrav1=");
    Serial.print(gravityBias1, 3);

    Serial.print("\tcounts2=");
    Serial.print(enc2.read());

    Serial.print("\tth2=");
    Serial.print(theta2, 6);

    Serial.print("\tdes2=");
    Serial.print(theta_des2, 6);

    Serial.print("\tfinal2=");
    Serial.print(theta_final2, 6);

    Serial.print("\terr2=");
    Serial.print(err2, 6);

    Serial.print("\tderr2=");
    Serial.print(derr2, 6);

    Serial.print("\tint2=");
    Serial.print(integral2, 6);

    Serial.print("\tu2=");
    Serial.print(u2, 6);

    Serial.print("\tpwmCmd2=");
    Serial.print(pwmCmd2);

    Serial.print("\tpwm2=");
    Serial.print(pwmOut2);

    Serial.print("\tgrav2=");
    Serial.print(gravityBias2, 3);

    Serial.print("\tth3=");
    Serial.print(theta3, 6);

    Serial.print("\tdes3=");
    Serial.print(theta_des3, 6);

    Serial.print("\tfinal3=");
    Serial.print(theta_final3, 6);

    Serial.print("\terr3=");
    Serial.print(err3, 6);

    Serial.print("\tu3=");
    Serial.print(u3, 6);

    Serial.print("\tpwm3=");
    Serial.print(pwmOut3);

    Serial.print("\tgrav3=");
    Serial.print(gravityBias3, 3);

    Serial.print("\tramp=");
    Serial.print(useRamp);

    Serial.print("\ttime_ms=");
    Serial.println(millis());

    printDecim = 0;
  }
}

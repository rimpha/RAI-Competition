
#include <Wire.h>
#include <math.h>


// =====================================================
// 1. PIN CONFIGURATION
// =====================================================

// L298N
#define ENA 5
#define IN1 7
#define IN2 8

#define ENB 6
#define IN3 9
#define IN4 10

// Front Ultrasonic
#define FRONT_TRIG 2
#define FRONT_ECHO 3

// Left Ultrasonic
#define LEFT_TRIG 4
#define LEFT_ECHO 11

// Right Ultrasonic
#define RIGHT_TRIG 12
#define RIGHT_ECHO 13

// IR Sensor
#define IR_PIN A0

// MPU6050
#define MPU_ADDR 0x68


// =====================================================
// 2. CONFIGURATION
// =====================================================

// Motor PWM
const int BASE_SPEED = 75;
const int TURN_SPEED = 65;

const int MIN_MOTOR_SPEED = 60;
const int MAX_MOTOR_SPEED = 110;

// Motor adjustment
int LEFT_OFFSET = 0;
int RIGHT_OFFSET = 0;

// Ultrasonic thresholds
const float FRONT_LIMIT = 12.0;
const float SIDE_OPEN_LIMIT = 25.0;

// Sensor timeout
const unsigned long ECHO_TIMEOUT = 18000;

// Gyroscope
const int YAW_SIGN = 1;

// Turning
const float TURN_TOLERANCE = 2.0;
const float TURN_RATE_LIMIT = 5.0;

const unsigned long TURN_TIMEOUT = 7000;

// Reverse recovery
const unsigned long BACKTRACK_MS = 700;

// Navigation
const unsigned long DECISION_DELAY = 150;

// Finish detection
const int IR_CONFIRM_COUNT = 5;


// =====================================================
// 3. PID PARAMETERS
// =====================================================

// Heading PID
float headingKp = 2.0;
float headingKi = 0.0;
float headingKd = 0.8;

// Turning PID
float turnKp = 1.8;
float turnKi = 0.0;
float turnKd = 0.35;

// PID integral limits
const float HEADING_INTEGRAL_LIMIT = 20.0;
const float TURN_INTEGRAL_LIMIT = 30.0;

// Correction limits
const int MAX_HEADING_CORRECTION = 25;
const int MAX_TURN_OUTPUT = 100;


// =====================================================
// 4. GLOBAL VARIABLES
// =====================================================

float gyroBiasZ = 0.0;
float yawAngle = 0.0;

float targetHeading = 0.0;

float gyroRateZ = 0.0;

unsigned long lastGyroTime = 0;

bool finished = false;
bool recoveryStopped = false;

// PID variables
float headingIntegral = 0.0;
float turnIntegral = 0.0;

// Navigation state
enum RobotState {
  RUNNING,
  FINISHED,
  RECOVERY_STOP
};

RobotState robotState = RUNNING;


// =====================================================
// 5. MOTOR CONTROL
// =====================================================

void setMotor(int leftSpeed, int rightSpeed) {

  leftSpeed = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

  // Left motor
  if (leftSpeed >= 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  }
  else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }

  // Right motor
  if (rightSpeed >= 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  }
  else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }

  analogWrite(ENA, abs(leftSpeed));
  analogWrite(ENB, abs(rightSpeed));
}


void stopMotor() {

  analogWrite(ENA, 0);
  analogWrite(ENB, 0);

}


void emergencyStop() {

  stopMotor();

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);

}


void moveForwardRaw(int speedValue) {

  setMotor(
    speedValue + LEFT_OFFSET,
    speedValue + RIGHT_OFFSET
  );

}


void moveBackwardRaw(int speedValue) {

  setMotor(-speedValue, -speedValue);

}


// =====================================================
// 6. ULTRASONIC SENSOR
// =====================================================

float readUltrasonicRaw(int trigPin, int echoPin) {

  digitalWrite(trigPin, LOW);
  delayMicroseconds(3);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);

  digitalWrite(trigPin, LOW);

  unsigned long duration =
    pulseIn(echoPin, HIGH, ECHO_TIMEOUT);

  if (duration == 0) {
    return 400.0;
  }

  return duration * 0.0343 / 2.0;

}


// Median filter: 3 measurements

float readUltrasonic(int trigPin, int echoPin) {

  float a = readUltrasonicRaw(trigPin, echoPin);
  delay(25);

  float b = readUltrasonicRaw(trigPin, echoPin);
  delay(25);

  float c = readUltrasonicRaw(trigPin, echoPin);

  // Sort values

  if (a > b) {
    float t = a;
    a = b;
    b = t;
  }

  if (b > c) {
    float t = b;
    b = c;
    c = t;
  }

  if (a > b) {
    float t = a;
    a = b;
    b = t;
  }

  return b;

}


float getFrontDistance() {
  return readUltrasonic(FRONT_TRIG, FRONT_ECHO);
}


float getLeftDistance() {
  return readUltrasonic(LEFT_TRIG, LEFT_ECHO);
}


float getRightDistance() {
  return readUltrasonic(RIGHT_TRIG, RIGHT_ECHO);
}


// =====================================================
// 7. IR FINISH DETECTION
// =====================================================

bool checkFinish() {

  int blackCount = 0;

  for (int i = 0; i < IR_CONFIRM_COUNT; i++) {

    if (digitalRead(IR_PIN) == HIGH) {
      blackCount++;
    }

    delay(10);

  }

  return blackCount == IR_CONFIRM_COUNT;

}


// =====================================================
// 8. MPU6050
// =====================================================

bool readMPURegister(uint8_t reg, int16_t &value) {

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);

  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  Wire.requestFrom(MPU_ADDR, (uint8_t)2, (uint8_t)true);

  if (Wire.available() < 2) {
    return false;
  }

  uint8_t highByte = Wire.read();
  uint8_t lowByte = Wire.read();

  value = (int16_t)((highByte << 8) | lowByte);

  return true;

}


void calibrateGyro() {

  Serial.println("GYRO CALIBRATION");
  Serial.println("KEEP ROBOT STILL!");

  long total = 0;

  const int samples = 1500;

  for (int i = 0; i < samples; i++) {

    int16_t rawZ = 0;

    if (readMPURegister(0x47, rawZ)) {
      total += rawZ;
    }

    delay(2);

  }

  gyroBiasZ = (float)total / samples;

  yawAngle = 0;

  targetHeading = 0;

  lastGyroTime = micros();

  Serial.print("Gyro Bias: ");
  Serial.println(gyroBiasZ);

  Serial.println("CALIBRATION COMPLETE");

}


void updateGyro() {

  unsigned long now = micros();

  float dt = (now - lastGyroTime) / 1000000.0;

  lastGyroTime = now;

  int16_t rawZ = 0;

  if (!readMPURegister(0x47, rawZ)) {
    gyroRateZ = 0;
    return;
  }

  // MPU6050 default range +/-250 deg/s
  gyroRateZ = (rawZ - gyroBiasZ) / 131.0;

  yawAngle += gyroRateZ * YAW_SIGN * dt;

}


// =====================================================
// 9. PID HEADING CONTROL
// =====================================================

void resetHeadingPID() {

  headingIntegral = 0;

}


void headingPID(float target) {

  updateGyro();

  float error = target - yawAngle;

  float dt = 0.01;

  headingIntegral += error * dt;

  headingIntegral = constrain(
    headingIntegral,
    -HEADING_INTEGRAL_LIMIT,
    HEADING_INTEGRAL_LIMIT
  );

  float derivative = -gyroRateZ * YAW_SIGN;

  float output =
    headingKp * error +
    headingKi * headingIntegral +
    headingKd * derivative;

  output = constrain(
    output,
    -MAX_HEADING_CORRECTION,
    MAX_HEADING_CORRECTION
  );

  int leftSpeed = BASE_SPEED + (int)output;
  int rightSpeed = BASE_SPEED - (int)output;

  leftSpeed = constrain(
    leftSpeed,
    MIN_MOTOR_SPEED,
    MAX_MOTOR_SPEED
  );

  rightSpeed = constrain(
    rightSpeed,
    MIN_MOTOR_SPEED,
    MAX_MOTOR_SPEED
  );

  setMotor(leftSpeed, rightSpeed);

}


// =====================================================
// 10. PID TURN CONTROL
// =====================================================

bool turnToAngle(float angleChange) {

  stopMotor();
  delay(150);

  float turnTarget = yawAngle + angleChange;

  float previousError = turnTarget - yawAngle;

  turnIntegral = 0;

  unsigned long startTime = millis();

  unsigned long previousTime = micros();

  Serial.print("TURN TARGET: ");
  Serial.println(turnTarget);

  while (millis() - startTime < TURN_TIMEOUT) {

    updateGyro();

    unsigned long now = micros();

    float dt = (now - previousTime) / 1000000.0;

    previousTime = now;

    if (dt <= 0 || dt > 0.1) {
      dt = 0.01;
    }

    float error = turnTarget - yawAngle;

    turnIntegral += error * dt;

    turnIntegral = constrain(
      turnIntegral,
      -TURN_INTEGRAL_LIMIT,
      TURN_INTEGRAL_LIMIT
    );

    float derivative = -gyroRateZ * YAW_SIGN;

    float output =
      turnKp * error +
      turnKi * turnIntegral +
      turnKd * derivative;

    output = constrain(
      output,
      -MAX_TURN_OUTPUT,
      MAX_TURN_OUTPUT
    );

    if (
      abs(error) <= TURN_TOLERANCE &&
      abs(gyroRateZ) <= TURN_RATE_LIMIT
    ) {

      stopMotor();
      delay(200);

      targetHeading = turnTarget;

      resetHeadingPID();

      Serial.println("TURN COMPLETE");

      return true;

    }

    int motorSpeed = abs((int)output);

    // Minimum PWM is needed to overcome motor friction.
    motorSpeed = constrain(
      motorSpeed,
      MIN_MOTOR_SPEED,
      TURN_SPEED
    );

    if (output > 0) {

      // Right rotation
      setMotor(motorSpeed, -motorSpeed);

    }
    else {

      // Left rotation
      setMotor(-motorSpeed, motorSpeed);

    }

    previousError = error;

  }

  stopMotor();

  Serial.println("TURN TIMEOUT - CHECK GYRO");

  return false;

}


void turnLeft() {
  turnToAngle(-90.0);
}


void turnRight() {
  turnToAngle(90.0);
}


// =====================================================
// 11. REVERSE RECOVERY
// =====================================================

void reverseRecovery() {

  Serial.println("========================");
  Serial.println("DEAD END DETECTED");
  Serial.println("REVERSE - NO TURN");
  Serial.println("========================");

  stopMotor();
  delay(200);

  // Reverse in the same direction of orientation.
  // Do not rotate the robot.

  moveBackwardRaw(65);

  delay(BACKTRACK_MS);

  stopMotor();

  delay(300);

  // No rear sensor / encoder:
  // Exact return position cannot be confirmed.

  recoveryStopped = true;

  robotState = RECOVERY_STOP;

  Serial.println("REVERSE COMPLETE");
  Serial.println("ROBOT STOPPED FOR SAFETY");

}


// =====================================================
// 12. SENSOR READING
// =====================================================

struct SensorData {

  float front;
  float left;
  float right;

};


SensorData readAllSensors() {

  SensorData data;

  data.front = getFrontDistance();

  delay(40);

  data.left = getLeftDistance();

  delay(40);

  data.right = getRightDistance();

  return data;

}


void printSensorData(SensorData data) {

  Serial.print("F: ");
  Serial.print(data.front);

  Serial.print(" | L: ");
  Serial.print(data.left);

  Serial.print(" | R: ");
  Serial.print(data.right);

  Serial.print(" | Yaw: ");
  Serial.println(yawAngle);

}


// =====================================================
// 13. NAVIGATION
// =====================================================

void mazeSolver() {

  if (robotState != RUNNING) {
    return;
  }

  // Check finish first

  if (checkFinish()) {

    stopMotor();

    finished = true;

    robotState = FINISHED;

    Serial.println("========================");
    Serial.println("MAZE FINISHED!");
    Serial.println("========================");

    return;

  }

  // Read sensors

  stopMotor();

  SensorData data = readAllSensors();

  printSensorData(data);

  // Determine wall conditions

  bool frontBlocked =
    data.front < FRONT_LIMIT;

  bool leftOpen =
    data.left > SIDE_OPEN_LIMIT;

  bool rightOpen =
    data.right > SIDE_OPEN_LIMIT;

  // ---------------------------------------------------
  // LEFT HAND PRIORITY
  // ---------------------------------------------------

  // 1. Left path available

  if (leftOpen) {

    Serial.println("LEFT PATH AVAILABLE");

    turnLeft();

    return;

  }

  // 2. Front path available

  if (!frontBlocked) {

    Serial.println("MOVE FORWARD");

    resetHeadingPID();

    return;

  }

  // 3. Right path available

  if (rightOpen) {

    Serial.println("RIGHT PATH AVAILABLE");

    turnRight();

    return;

  }

  // 4. Dead end: reverse instead of U-turn

  Serial.println("ALL PATHS BLOCKED");

  reverseRecovery();

}


// =====================================================
// 14. SETUP
// =====================================================

void setup() {

  Serial.begin(9600);

  // Motor pins

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Ultrasonic

  pinMode(FRONT_TRIG, OUTPUT);
  pinMode(FRONT_ECHO, INPUT);

  pinMode(LEFT_TRIG, OUTPUT);
  pinMode(LEFT_ECHO, INPUT);

  pinMode(RIGHT_TRIG, OUTPUT);
  pinMode(RIGHT_ECHO, INPUT);

  // IR

  pinMode(IR_PIN, INPUT);

  stopMotor();

  // Initialize I2C

  Wire.begin();

  Wire.setClock(100000);

  delay(200);

  // Wake MPU6050

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  delay(100);

  // Gyro full scale +/-250 deg/s

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B);
  Wire.write(0x00);
  Wire.endTransmission();

  // DLPF configuration

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1A);
  Wire.write(0x03);
  Wire.endTransmission();

  delay(500);

  // Gyro calibration

  calibrateGyro();

  Serial.println("========================");
  Serial.println("MAZE SOLVER V2.0");
  Serial.println("PID CONTROL ACTIVE");
  Serial.println("SLOW MODE ACTIVE");
  Serial.println("========================");

  delay(2000);

}


// =====================================================
// 15. MAIN LOOP
// =====================================================

void loop() {

  // Finished

  if (robotState == FINISHED) {

    emergencyStop();

    return;

  }

  // Recovery stop

  if (robotState == RECOVERY_STOP) {

    emergencyStop();

    return;

  }

  // Check finish while moving

  if (checkFinish()) {

    emergencyStop();

    robotState = FINISHED;

    Serial.println("FINISH DETECTED");

    return;

  }

  // Read front distance for emergency stop

  float front = getFrontDistance();

  if (front < FRONT_LIMIT) {

    stopMotor();

    delay(DECISION_DELAY);

    mazeSolver();

    return;

  }

  // PID forward movement

  headingPID(targetHeading);

}

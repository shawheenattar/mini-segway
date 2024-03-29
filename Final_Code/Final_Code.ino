/* -----------------------------
 * ------- PREPROCESSOR --------
 * ----------------------------- */
#include <Wire.h>
#include <QueueList.h>
#include <CommunicationUtils.h>
#include <DebugUtils.h>
//libraries needed for using SparkFun IMU
//#include <FIMU_ADXL345.h>
//#include <FIMU_ITG3200.h>
//#include <FreeSixIMU.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_Simple_AHRS.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_9DOF.h>
#include "PIDController.h"
#include "Motor.h"
#include <PS3BT.h>
#include <usbhub.h>
#include <SPI.h>
#include <L3G.h>


L3G gyro;

// motor
#define R_MOTOR 0
#define L_MOTOR 1

// encoder speed direction
#define ENC_FORWARD -1;
#define ENC_BACKWARD 1;

// turning
#define TURN_RIGHT -1
#define TURN_NONE 0
#define TURN_LEFT 1

// PS3 controller
// Satisfy IDE, which only needs to see the include statment in the ino.
#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif

/* -----------------------------
 * ----------- IMU -------------
 * ----------------------------- */
//FreeSixIMU IMU = FreeSixIMU();
//
//// arrays passed to IMU object to be filled up with raw values
//int16_t rawAcc[3] = {0, 0, 0};
//float rawGyro[3] = {0.0, 0.0, 0.0}; 
float zeroIMUAngle = 0.22;

// sensor scale factors
  const float GYRO_SCALE = 1; // already taken care of in FreeSixIMU library
  const float ACC_SCALE = 0.1; // taken from TKJ Electronics' code

const float RADIAN_TO_DEGREE = float(180 / 3.14); // for use in accelerometer angle calculation
//
double pitch = 0; // stores filtered pitch of robot

/*-----------------------------------*/

/* Assign a unique ID to the sensors */
Adafruit_9DOF                dof   = Adafruit_9DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);

/* Update this with the correct SLP for accurate altitude measurements */
float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;

void initSensors()
{
  if(!accel.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println(F("Ooops, no LSM303 detected ... Check your wiring!"));
    while(1);
  }
  if(!mag.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while(1);
  }
}


/* -----------------------------
 * ---------- MOTOR ------------
 * ----------------------------- */
// motor PWM
const int L_MOTOR_PWM = 5;
const int R_MOTOR_PWM = 4;

// change direction of motors
const int L_MOTOR_IN_A = 7;
const int L_MOTOR_IN_B = 6;
const int R_MOTOR_IN_A = 2;
const int R_MOTOR_IN_B = 3;

// deadband pitch
const float DEADBAND_PITCH = 1.25;

Motor motor_R(R_MOTOR_PWM, R_MOTOR_IN_A, R_MOTOR_IN_B);
Motor motor_L(L_MOTOR_PWM, L_MOTOR_IN_A, L_MOTOR_IN_B);

/* -----------------------------
 * --------- ENCODERS ----------
 * ----------------------------- */
// encoder Arduino ports
const int L_ENCODER_A = 24;
const int L_ENCODER_B = 25;
const int R_ENCODER_A = 52;
const int R_ENCODER_B = 53;

// dt calculation
bool leftEncTimeState, rightEncTimeState = true;
long leftEnc_t1, rightEnc_t1 = 0;
long leftEnc_t2, rightEnc_t2 = 0;
long leftEnc_dt, rightEnc_dt = 0;

// encoder direction (-1 or 1 coefficient)
int leftEncDirection, rightEncDirection = ENC_FORWARD;

// speed calculation constants
const int THRESHOLD_DT = 50; // number of milliseconds of not moving for speed to count as 0
const float MILLIS_TO_SEC = 1000;

// digital low pass filter stuff (digitalSmooth function)
float smoothedRightSpeed, smoothedLeftSpeed = 0;

/* -----------------------------
 * ---- SPEED TO ANGLE PID -----
 * ----------------------------- */
const float kP_speed = -0.008;
const float kI_speed = -0.0001;
const float kD_speed = 0.00005 / 0.0035;

PIDController speedToAnglePIDController(kP_speed, kI_speed, kD_speed);

float speedSetpoint = 0.0;

unsigned long nowTime = 0;

/* -----------------------------
 * ---- ANGLE TO MOTOR PID -----
 * ----------------------------- */
// gains
//float kP_angle = 70.0;
//const float kI_angle = 0.4;
//const float kD_angle = 0.5;

float kP_angle = 85.0;
const float kI_angle = 0.7;
const float kD_angle = 0.2 / 0.0035;

PIDController angleToMotorPIDController(kP_angle, kI_angle, kD_angle, true, false, true);

float angleSetpoint = 0.0;
float commandedSpeedSetpoint;

/* -----------------------------
 *  COMMANDED SPEED TO PWM PID -
 * ----------------------------- */
const float kP_speedToPWM = -0.06;
const float kI_speedToPWM = -0.001;
const float kD_speedToPWM = 0.0;
const float kFF_speedToPWM = 0.5;

PIDController speedToPWMPIDController_R(kP_speedToPWM, kI_speedToPWM, kD_speedToPWM);
PIDController speedToPWMPIDController_L(kP_speedToPWM, kI_speedToPWM, kD_speedToPWM);

float motorPWMCommand_R, motorPWMCommand_L = 0;

/* -----------------------------
 * --- COMPLEMENTARY FILTER ----
 * ----------------------------- */
unsigned long loopTime = 0;
const float COMPLEMENTARY_GAIN = 0.995;
float lastPitch = 0;
unsigned long lastStartTime = 0;

/* -----------------------------
 * ------ PS3 CONTROLLER -------
 * ----------------------------- */
USB usb;
BTD btd(&usb); // bluetooth dongle
PS3BT PS3(&btd); // PS3 controller bluetooth
const int JOYSTICK_DEADBAND = 28;

int turningDirection = TURN_NONE;
float turningOffset = 0.0;

/* ====================================
 ================ SETUP ===============
 ====================================== */
void setup() {
  Serial.begin(9600);
  
  if (usb.Init() == -1) {
    Serial.print(F("\r\nOSC did not start"));
    while (1); //halt
  }
  Serial.print(F("\r\nPS3 Bluetooth Library Started"));

  // Arduino pins to motor driver
  motor_R.setup();
  motor_L.setup();
  
  // Arduino pins to encoder
  pinMode(L_ENCODER_A, INPUT);
  pinMode(L_ENCODER_B, INPUT);
  pinMode(R_ENCODER_A, INPUT);
  pinMode(R_ENCODER_B, INPUT);

  // encoder read interrupts
  attachInterrupt(L_ENCODER_A, leftEncoder, RISING); // pin 2, low to high
  attachInterrupt(R_ENCODER_A, rightEncoder, RISING); // pin 3, low to high

  // IMU initialization
//  Wire.begin(); // IMU connection
//  delay(5);
//  IMU.init(); // begin the IMU
//  delay(5);


/* Initialise the sensors */
  initSensors();
  Serial.begin(9600);
  Wire.begin();

  if (!gyro.init())
  {
    Serial.println("Failed to autodetect gyro type!");
    while (1);
  }

  gyro.enableDefault();


  Serial.begin(115200);
  Serial.println(F("Adafruit 9 DOF Pitch/Roll/Heading Example")); Serial.println("");
  

}

/* ====================================
 ================ LOOP ================
 ====================================== */
void loop() {
  
//---------------------------------------
//---------------read IMU----------------
//---------------------------------------
  sensors_event_t accel_event;
  sensors_event_t mag_event;
  sensors_vec_t   orientation;

  /* Calculate pitch and roll from the raw accelerometer data and heading from magnetometer */
  accel.getEvent(&accel_event);
  dof.accelGetOrientation(&accel_event, &orientation);
  mag.getEvent(&mag_event);
  dof.magGetOrientation(SENSOR_AXIS_Z, &mag_event, &orientation);

  float rawAccX = orientation.roll;
  float rawAccY = orientation.pitch;
  float rawAccZ = orientation.heading;

//  Serial.println(rawAccX);

  /*reads values from gyroscope*/
  gyro.read();

  int rawGyroX = (int)gyro.g.x;
  int rawGyroY = (int)gyro.g.y;
  int rawGyroZ = (int)gyro.g.z;

//  Serial.println(rawGyroX);
//  Serial.println(rawGyroY);
//  Serial.println(rawGyroZ);

 delay(10);

//-------------------------------------------
//--------------read IMU end-----------------
//-------------------------------------------

  float accXval = rawAccX / ACC_SCALE;
  accXval--; //-1g when lying down
  float accYval = rawAccY / ACC_SCALE;
  float accZval = rawAccZ / ACC_SCALE;

  float R = sqrt(pow(accXval, 2) + pow(accYval, 2) + pow(accZval, 2)); // Calculate the length of force vector
  float angleX = acos(accXval / R) * RADIAN_TO_DEGREE;
  
  nowTime = millis();
  loopTime = nowTime - lastStartTime;
  
  usb.Task();
  
  if (PS3.PS3Connected || PS3.PS3NavigationConnected) {
    readJoystick();
  }
  
  // get raw acc and gyro readings
//  updateIMU();

  // complementary filter
  pitch = COMPLEMENTARY_GAIN * (lastPitch + rawGyroX * loopTime / 1000) + (1 - COMPLEMENTARY_GAIN) * (angleX - zeroIMUAngle);
  lastPitch = pitch;

//  Serial.println(pitch);

  // PID controllers
  speedToAnglePID();
  angleToMotorPID();
  speedToPWMPID();

  lastStartTime = nowTime;

  Serial.println(leftEnc_t2);
  Serial.println(rightEnc_t2);
  Serial.println(rightEncDirection);
  Serial.println(getSmoothedRightSpeed());
  
  
}

/* ====================================
 ============ READ JOYSTICK ===========
 ====================================== */
void readJoystick() {
  // left/right
  if (PS3.getAnalogHat(RightHatX) > 150) { // right
    turningOffset = 5.0;
    turningDirection = TURN_RIGHT;
  } else if (PS3.getAnalogHat(RightHatX) < 120) { // left
    turningOffset = 5.0;
    turningDirection = TURN_LEFT;
  } else { // stay still
    turningOffset = 0.0;
    turningDirection = TURN_NONE;
  }
  
  // forward/backward
  speedSetpoint = mapJoystickValue(PS3.getAnalogHat(LeftHatY), 140.0);

  
}

float mapJoystickValue(int joyValue, float outputMax) {
  if (joyValue >= 127.5 + JOYSTICK_DEADBAND) { // backwards
    return scale(joyValue, 150.0, 255, 0, -1.0 * outputMax);
  } else if (joyValue <= 127.5 - JOYSTICK_DEADBAND) { // forwards
    return scale(joyValue, 105.0, 0, 0, outputMax);
  } else { // don't move
    return 0.0;
  }
}

float scale(int input, float inputMin, int inputMax, int outputMin, float outputMax) {
  float output = 0;
  output = abs(input - inputMin) / abs(inputMax - inputMin) * (outputMax - outputMin);
  constrain(output, outputMin, outputMax);
  return int(output);
}


/* ====================================
 === SPEED TO ANGLE PID + GET SPEED ===
 ====================================== */
void speedToAnglePID() {
  angleSetpoint = speedToAnglePIDController.compute(getAverageFilteredSpeed(), speedSetpoint);
  Serial.println(getAverageFilteredSpeed());
}

float getAverageFilteredSpeed() {
  // WHEN ALL THREE LOOPS ARE IN, ADD GLOBAL VARS BACK IN AND DELETE GET SMOOTHED METHODS
  return (getSmoothedRightSpeed() + getSmoothedLeftSpeed()) / 2;
}

float getSmoothedRightSpeed() {
  smoothedRightSpeed = smooth(getRawRightSpeed(), 0.95, smoothedRightSpeed);
  return smoothedRightSpeed;
}

float getSmoothedLeftSpeed() {
  smoothedLeftSpeed = smooth(getRawLeftSpeed(), 0.95, smoothedLeftSpeed);
  return smoothedLeftSpeed;
}

float getRawLeftSpeed() {
  if ((millis() - leftEnc_t2) < THRESHOLD_DT) {
    return leftEncDirection * (MILLIS_TO_SEC / leftEnc_dt);
  } else {
    return 0;
  }
}

float getRawRightSpeed() {
  if ((millis() - rightEnc_t2) < THRESHOLD_DT) {
    return rightEncDirection * (MILLIS_TO_SEC / rightEnc_dt);
  } else {
    return 0;
  }

}

/* ====================================
 ====== ANGLE TO MOTOR SPEED PID ======
 ====================================== */
void angleToMotorPID() {
  commandedSpeedSetpoint = angleToMotorPIDController.compute(pitch, angleSetpoint);
//  Serial.println(angleToMotorPIDController.getDerivative());
}

/* ====================================
  MOTOR SPEED TO PWM PID + MOVE MOTORS 
 ====================================== */
void speedToPWMPID() {
  motorPWMCommand_R = speedToPWMPIDController_R.feedforwardCompute(getSmoothedRightSpeed(), commandedSpeedSetpoint, kFF_speedToPWM);
  motorPWMCommand_L = speedToPWMPIDController_L.feedforwardCompute(getSmoothedLeftSpeed(), commandedSpeedSetpoint, kFF_speedToPWM);
                
  switch(turningDirection) {
    case TURN_RIGHT:
      motor_R.sendPWMCommand(-1 * (motorPWMCommand_R + turningOffset));
      motor_L.sendPWMCommand(motorPWMCommand_L + turningOffset);
      break;
    case TURN_LEFT:
      motor_R.sendPWMCommand(motorPWMCommand_R + turningOffset);
      motor_L.sendPWMCommand(-1 * (motorPWMCommand_L + turningOffset));
      break;
    case TURN_NONE:
      motor_R.sendPWMCommand(motorPWMCommand_R);
      motor_L.sendPWMCommand(motorPWMCommand_L);
      break;

      
  }
//      Serial.println(motorPWMCommand_R);
//      Serial.println(motorPWMCommand_L);
}

/* ====================================
 = ENCODER INTERRUPT SERVICE ROUTINES =
 ====================================== */
void leftEncoder() {
  // determine direction
  if (digitalRead(L_ENCODER_B) == LOW) {
    leftEncDirection = ENC_FORWARD;
  } else {
    leftEncDirection = ENC_BACKWARD;
  }

  // record dt time difference in milliseconds
  if (leftEncTimeState) {
    leftEnc_t1 = millis();
    leftEncTimeState = false;
  } else {
    leftEnc_t2 = millis();
    leftEnc_dt = leftEnc_t2 - leftEnc_t1;
    leftEncTimeState = true;
  }
}

void rightEncoder() {
  // determine direction
  if (digitalRead(R_ENCODER_B) == LOW) {
    rightEncDirection = ENC_FORWARD;
  } else {
    rightEncDirection = ENC_BACKWARD;
  }

  // record dt time difference in milliseconds
  if (rightEncTimeState) {
    rightEnc_t1 = millis();
    rightEncTimeState = false;
  } else {
    rightEnc_t2 = millis();
    rightEnc_dt = rightEnc_t1 - rightEnc_t2;
    rightEncTimeState = true;
  }
}

/* ====================================
 === ENCODER SPEED LOW PASS FILTER ====
 ====================================== */
/*
  from http://playground.arduino.cc/Main/Smooth
  written by Paul Badger
  int sensVal - the sensor variable - raw material to be smoothed
  float  filterVal - The filter value is a float and must be between 0 and .9999 say. 0 is off (no smoothing) and .9999 is maximum smoothing.
    The actual performance of the filter is going to be dependent on fast you are sampling your sensor (the total loop time), so
    some trial and error will probably be neccessary to get the desired response.
  smoothedVal - Use this for the output of the sensor and also feed it back into the loop. Each sensor needs its own value.
    Don't use this variable for any other purpose.
*/
int smooth(float data, float filterVal, float smoothedVal) {
  if (filterVal > 1) {     // check to make sure param's are within range
    filterVal = .99;
  }
  else if (filterVal <= 0) {
    filterVal = 0;
  }

  smoothedVal = (data * (1 - filterVal)) + (smoothedVal  *  filterVal);

  return (int)smoothedVal;
}

/* ====================================
 ====== GET PROCESSED IMU VALUES ======
 ====================================== */
//void updateIMU() {
// 
//}

//float getGyroXRate() {
//  return (int)gyro.g.x;
//}
//
//// taken from TKJ Electronics' code
//float getAccX() {
//
//  sensors_event_t accel_event;
//  sensors_event_t mag_event;
//  sensors_vec_t   orientation;
//
//  /* Calculate pitch and roll from the raw accelerometer data */
//  accel.getEvent(&accel_event);
//  dof.accelGetOrientation(&accel_event, &orientation);
//  
//  float accXval = orientation.roll / ACC_SCALE;
//  float accYval = orientation.pitch / ACC_SCALE;
//  accYval--; //-1g when lying down
//  float accZval = orientation.heading / ACC_SCALE;
//
//  float R = sqrt(pow(accXval, 2) + pow(accYval, 2) + pow(accZval, 2)); // Calculate the length of force vector
//  float angleX = acos(accXval / R) * RADIAN_TO_DEGREE;
//  return angleX;
//  
//}
//


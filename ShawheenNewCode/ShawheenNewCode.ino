#include <Wire.h>
#include <QueueList.h>
#include <FIMU_ADXL345.h>
#include <FIMU_ITG3200.h>
#include <FreeSixIMU.h>
#include <Motor.h>
#include <SimplePID.h>
#include <PS3BT.h>
#include <usbhub.h>
#include <SPI.h>

// PS3 controller
// Satisfy IDE, which only needs to see the include statment in the ino.
#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif

// encoder speed direction
#define ENC_FORWARD -1;
#define ENC_BACKWARD 1;

// Motor setup
const int motorLpwm = 5;
const int motorRpwm = 4;
const int motorLdirectionA = 7;
const int motorLdirectionB = 6;
const int motorRdirectionA = 2;
const int motorRdirectionB = 3;

Motor motorL(motorLpwm , motorLdirectionA, motorLdirectionB);
Motor motorR(motorRpwm , motorRdirectionA, motorRdirectionB);

// Encoder setup
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

// IMU Setup
FreeSixIMU IMU = FreeSixIMU();

// arrays passed to IMU object to be filled up with raw values
int16_t rawAcc[3] = {0, 0, 0};
float rawGyro[3] = {0.0, 0.0, 0.0};

float zeroIMUAngle = 7.8;

// sensor scale factors
const float GYRO_SCALE = 1; // already tMaken care of in FreeSixIMU library
const float ACC_SCALE = 0.1; // taken from TKJ Electronics' code
const float RADIAN_TO_DEGREE = float(180 / 3.14); // for use in accelerometer angle calculation

double pitch = 0; // stores filtered pitch of robot

float lastPitch = 0;

unsigned long loopTime = 0;
unsigned long lastStartTime = 0;
const float complementaryGain = .995;
unsigned long nowTime = 0;

void motor(int motorPower);

/*working variables*/
unsigned long lastTime;
double Input, Output, Setpoint;
double errSum, lastErr;
double kp, ki, kd;

float angleSetpoint = 0.0;
float speedSetpoint = 0.0;
float commandedSpeedSetpoint;

/* -----------------------------
 * ------ PS3 CONTROLLER -------
 * ----------------------------- */
USB usb;
BTD btd(&usb); // bluetooth dongle
PS3BT PS3(&btd); // PS3 controller bluetooth
const int JOYSTICK_DEADBAND = 28;

float turningOffset = 0.0;

////////////////////////
/// PID Controlllers ///
////////////////////////

SimplePID angleToMotor(29, 0.0, .15); // (kP, kI, kD) 33, 0 , 0.4

/////////////////////////
///////  SETUP  /////////
/////////////////////////

//Balanced offset:
float offset = 0.0;

void setup() {

  Serial.begin(9600);
  Serial2.begin(9600);

  if (usb.Init() == -1) {
    Serial2.print(F("\r\nOSC did not start"));
    while (1); //halt
  }
  Serial2.print(F("\r\nPS3 Bluetooth Library Started"));
  
  motorL.setup();
  motorR.setup();
  
  pinMode(motorLpwm, OUTPUT);
  pinMode(motorRpwm, OUTPUT);
  pinMode(motorLdirectionA, OUTPUT);
  pinMode(motorLdirectionB, OUTPUT);
  pinMode(motorRdirectionA, OUTPUT);
  pinMode(motorRdirectionB, OUTPUT);

  Wire.begin(); // IMU connection
  delay(5);
  IMU.init(); // begin the IMU
  delay(5);
  
//  while(analogRead(A8) < 900) {
//    
//  }
  updateIMU();
  offset = getAccX();
}

//////////////////////////
////////  LOOP  //////////
//////////////////////////

void loop() {
    
  nowTime = millis();
  loopTime = nowTime - lastStartTime;

  usb.Task();
  if (PS3.PS3Connected) {
    readJoystick();
  }
  
  updateIMU();

  pitch = (complementaryGain * (lastPitch + getGyroXRate() * loopTime / 1000)) + ((1 - complementaryGain) * (getAccX() - offset));
  lastPitch = pitch;

  //speedToAnglePID();

  angleToMotor.setpoint(speedSetpoint); //angleSetpoint
  angleToMotor.input(pitch);
  angleToMotor.compute();
  angleToMotor.output();

Serial2.print(getAccX());
Serial2.print(" ");
Serial2.print(offset);
Serial2.print(" ");
Serial2.println(pitch);
  
  motor(map(angleToMotor.output(), -360, 360, 255, 0)); // Clean this up!
//  Serial2.print((pitch));
//  Serial2.print(" ");
//  Serial2.print(map(Output, -360, 360, 255, 0));
//  Serial2.print(" ");
//  Serial2.print(getAccX());
//  Serial2.println(getAccX() - offset);
  
  lastStartTime = nowTime;
  
}

/* ====================================
 ============ READ JOYSTICK ===========
 ====================================== */
void readJoystick() {
  // left/right
  if (PS3.getAnalogHat(RightHatY) > 150) { // right
    speedSetpoint = -3.0;
  } else if (PS3.getAnalogHat(RightHatY) < 120) { // left
    speedSetpoint = 3.0;
  } else { // stay still
    speedSetpoint = 0.0;
  }
  
}

/////////////////////////
///////  MOTORS  ////////
/////////////////////////

void motor(int motorPower) {
  //forward
  if(motorPower > 127.5) {
    digitalWrite(motorLdirectionA, LOW);
    digitalWrite(motorLdirectionB, HIGH);
    digitalWrite(motorRdirectionA, LOW);
    digitalWrite(motorRdirectionB, HIGH);
    analogWrite(motorLpwm, map(motorPower, 127.5, 255, 0, 255));
    analogWrite(motorRpwm, map(motorPower, 127.5, 255, 0, 255));
  }
  else if(motorPower < 127.5) {
    digitalWrite(motorLdirectionA, HIGH);
    digitalWrite(motorLdirectionB, LOW);
    digitalWrite(motorRdirectionA, HIGH);
    digitalWrite(motorRdirectionB, LOW);
    analogWrite(motorLpwm, map(motorPower, 127.5, 0, 0, 255));
    analogWrite(motorRpwm, map(motorPower, 127.5, 0, 0, 255));
  }
}

/////////////////////////
/////  UPDATE IMU  //////
/////////////////////////

void updateIMU() {
  IMU.acc.readAccel(&rawAcc[0], &rawAcc[1], &rawAcc[2]);
  IMU.gyro.readGyro(&rawGyro[0], &rawGyro[1], &rawGyro[2]);
}

//////////////////////////
////////  GYRO  //////////
//////////////////////////

float getGyroXRate() {
  return rawGyro[1];
}

/////////////////////////
////  ACCELEROMETER  ////
/////////////////////////

// taken from TKJ Electronics' code
float getAccX() {
  float accXval = rawAcc[0] / ACC_SCALE;
  float accYval = rawAcc[1] / ACC_SCALE;
  //accXval--; //-1g when lying down
  float accZval = rawAcc[2] / ACC_SCALE;

  float R = sqrt(pow(accXval, 2) + pow(accYval, 2) + pow(accZval, 2)); // Calculate the length of force vector
  float angleX = ((acos(accXval / R) * RADIAN_TO_DEGREE) - 90) * -1;
  return angleX;
}

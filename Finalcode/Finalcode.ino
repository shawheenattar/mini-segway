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


/* ====================================
 ================ SETUP ===============
 ====================================== */
void setup() {
  Serial.begin(9600);

  // Arduino pins to motor driver
  motor_R.setup();
  motor_L.setup();
  
  
  // IMU initialization
//  Wire.begin(); // IMU connection
//  delay(5);
//  IMU.init(); // begin the IMU
//  delay(5);

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
  
  /* Initialise the sensors */
  initSensors();

  // Arduino pins to motor driver
  pinMode(R_MOTOR_PWM, OUTPUT);
  pinMode(R_MOTOR_IN_A, OUTPUT);
  pinMode(R_MOTOR_IN_B, OUTPUT);
  pinMode(L_MOTOR_PWM, OUTPUT);
  pinMode(L_MOTOR_IN_A, OUTPUT);
  pinMode(L_MOTOR_IN_B, OUTPUT);

}

/* ====================================
 ================ LOOP ================
 ====================================== */
void loop() {

//=========== Accelerometer values ===========//
  sensors_event_t accel_event;
  sensors_event_t mag_event;
  sensors_vec_t   orientation;

  /* Calculate pitch and roll from the raw accelerometer data */
  accel.getEvent(&accel_event);
  if (dof.accelGetOrientation(&accel_event, &orientation))
  {
    /* 'orientation' should have valid .roll and .pitch fields */
    Serial.print(F("Roll: "));
    Serial.print(orientation.roll);
    Serial.print(F("; "));
    Serial.print(F("Pitch: "));
    Serial.print(orientation.pitch);
    Serial.print(F("; "));
  }
  
  /* Calculate the heading using the magnetometer */
  mag.getEvent(&mag_event);
  if (dof.magGetOrientation(SENSOR_AXIS_Z, &mag_event, &orientation))
  {
    /* 'orientation' should have valid .heading data now */
    Serial.print(F("Heading: "));
    Serial.print(orientation.heading);
    Serial.print(F("; "));
  }

  Serial.println(F(""));

//=========== Gyroscope values ==========//
  
  gyro.read();

  Serial.print("G ");
  Serial.print("X: ");
  Serial.print((int)gyro.g.x);
  Serial.print(" Y: ");
  Serial.print((int)gyro.g.y);
  Serial.print(" Z: ");
  Serial.println((int)gyro.g.z);

  delay(100);

  motorDirection((orientation.roll*1.5)+127);
  
  
}

void motorDirection(byte motorPower) {

if(motorPower < 126) {
  // backwards
  digitalWrite(R_MOTOR_IN_A, HIGH);
  digitalWrite(R_MOTOR_IN_B, LOW);
  digitalWrite(L_MOTOR_IN_A, HIGH);
  digitalWrite(L_MOTOR_IN_B, LOW);
  analogWrite(R_MOTOR_PWM, map(motorPower, 127, 0, 30, 255));
  analogWrite(L_MOTOR_PWM, map(motorPower, 127, 0, 30, 255));
  Serial.println(map(motorPower, 127, 0, 30, 255));
}

else if(motorPower > 128) {
  // forward
  digitalWrite(R_MOTOR_IN_A, LOW);
  digitalWrite(R_MOTOR_IN_B, HIGH);
  digitalWrite(L_MOTOR_IN_A, LOW);
  digitalWrite(L_MOTOR_IN_B, HIGH);
  analogWrite(R_MOTOR_PWM, map(motorPower, 127, 255, 30, 255));
  analogWrite(L_MOTOR_PWM, map(motorPower, 127, 255, 30, 255));
  Serial.println(map(motorPower, 127, 255, 30, 255));
}

  
  
}


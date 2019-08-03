/* -----------------------------
 * --------- ENCODERS ----------
 * ----------------------------- */
// encoder Arduino ports
const int L_ENCODER_A = 24;
const int L_ENCODER_B = 25;
const int R_ENCODER_A = 52;
const int R_ENCODER_B = 53;

// encoder speed direction
#define ENC_FORWARD -1;
#define ENC_BACKWARD 1;

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

float angleSetpoint = 0.0;
float commandedSpeedSetpoint;


void setup() {

  Serial.begin(9600);
  Serial2.begin(9600);
  // put your setup code here, to run once:

 // Arduino pins to encoder
  pinMode(L_ENCODER_A, INPUT);
  pinMode(L_ENCODER_B, INPUT);
  pinMode(R_ENCODER_A, INPUT);
  pinMode(R_ENCODER_B, INPUT);

  // encoder read interrupts
  attachInterrupt(L_ENCODER_A, leftEncoder, RISING); // pin 2, low to high
  attachInterrupt(R_ENCODER_A, rightEncoder, RISING); // pin 3, low to high
  
}

void loop() {
  // put your main code here, to run repeatedly:

  Serial2.print(getRawLeftSpeed());
  Serial2.print("  ");
  Serial2.println(getSmoothedLeftSpeed());
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

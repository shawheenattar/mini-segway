
#include "IOpins.h"
#include "Constants.h"



// define global variables here
volatile unsigned int lpulse=100000;                      // width of left and right encoder pulses in uS
volatile unsigned int rpulse=100000;                      // width of left and right encoder pulses in uS
volatile unsigned int lcount,rcount;                      // left and right pulse counters to measure distance
volatile byte lflag;                                      // flag indicates when left  encoder changes states
volatile byte rflag;                                      // flag indicates when right encoder changes states

byte lpwm,rpwm;                                           // left and right motor speeds generated from the processor
int lspeed,rspeed;                                        // left and right motor speeds requested by the user
unsigned long bestspeed=maxspeed;                         // encoder pulses per second for speed=100% (less than actual to allow for load)
unsigned int batvolt;                                     // battery voltage to 2 decimal places e.g. 751 = 7.51V
unsigned long mtime;                                      // "no go" timer used to re-start motors when both motors stopped
unsigned long btime;                                      // timer used to check battery voltage

volatile unsigned long ltime;                             // remembers time of left  encoders last state change in uS
volatile unsigned long rtime;                             // remembers time of right encoders last state change in uS

void setup()
{
  pinMode(lmdirpin,OUTPUT);
  pinMode(rmdirpin,OUTPUT);
  
  digitalWrite(lmencpin,1);                               // enable pullup resistor for left  encoder sensor
  digitalWrite(rmencpin,1);                               // enable pullup resistor for right encoder sensor
  attachInterrupt(0,Lencoder,CHANGE);                     // trigger left  encoder ISR on state change
  attachInterrupt(1,Rencoder,CHANGE);                     // trigger right encoder ISR on state change
  lspeed=0;rspeed=0;
}



void loop()
{ 
  //======================================================== Motor Speed Control =========================================================
  
  unsigned int temp;                                      // temporary variable for calculationa
  if(micros()-mtime>999)                                  // Call motor control function every mS
  {
    mtime=micros();                                       // reset motor timer
    MotorControl();                                       // update motor speeds    
    
    batvolt=analogRead(powerpin);                         // read battery voltage  
    temp=50*batvolt/maxvolt*batvolt/maxvolt*(maxspeed/50);// calculate best possible speed with available battery voltage as an integer
    bestspeed=long(temp);                                 // convert to long                                
  }
  
  
  //======================================================== Your Code ====================================================================
  
  // The tutorial for this code can be found here: http://letsmakerobots.com/node/38636
  
  // Adjust the maxvolt and maxspeed values in the "Constants.h" tab to suit your battery and gearbox as instructed in the tutorial
  // Adjust the IO pin definitions in the "IOpins.h" tab to suit your controller and wiring scheme
  
  // Use the variable lspeed to control the left  motor speed. -100 to +100.  Negative values indicate reverse direction
  // Use the variable rspeed to control the right motor speed. -100 to +100.  Negative values indicate reverse direction
  
  // use the variables lcount and rcount to measure distance travelled. These counters increment every time the encoders output changes state
  // for best accuracy, make sure the robot comes to a complete stop before resetting the counters
  
  
  
  
  
}













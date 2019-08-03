
//======================================================= Control speed and direction of left and right motors ==========================================
void MotorControl()
{
  unsigned long actual;                                // stores temporary calculation of actual left and right motor speeds in uS between encoder pulses
  if(lspeed<-100) lspeed=-100;                         // limit left and right motor speeds to allowed values
  if(rspeed<-100) rspeed=-100;
  if(lspeed>100) lspeed=100;
  if(rspeed>100) rspeed=100;
  
  if(micros()-ltime>20000 && lspeed!=0) lpwm+=2;       // jumpstart stalled motor
  if(micros()-rtime>20000 && rspeed!=0) rpwm+=2;       // jumpstart stalled motor
   
  digitalWrite(lmdirpin,lspeed>0);                     // set direction of left  motor
  actual=1000000/(abs(lspeed)*bestspeed/100);          // calculate desired time in uS between encoder pulses
  if(actual>lpulse && lpwm>0) lpwm--;                  // if motor is running too fast then decrease PWM
  if(actual<lpulse && lpwm<254) lpwm++;                // if motor is running too slow then increase PWM
  analogWrite(lmpwmpin,lpwm);                          // update speed for left  motor
      
  digitalWrite(rmdirpin,rspeed>0);                     // set direction of right motor
  actual=1000000/(abs(rspeed)*bestspeed/100);          // calculate desired time in uS between encoder pulses  
  if(actual>rpulse && rpwm>0) rpwm--;                  // if motor is running too fast then decrease PWM
  if(actual<rpulse && rpwm<254) rpwm++;                // if motor is running too slow then increase PWM
  analogWrite(rmpwmpin,rpwm);                          // update speed for right motor
}

/*
Power    SC/sec     μS/SC
100%  =   1900  =   526μS
 90%  =   1710  =   585μS
 80%  =   1520  =   658μS
 70%  =   1330  =   751μS
 60%  =   1140  =   877μS
 50%  =    950  =  1053μS
 40%  =    760  =  1316μS
 30%  =    570  =  1754μS
 20%  =    380  =  2631μS
 10%  =    190  =  5263μS
  1%  =     19  = 52632μS 
 */


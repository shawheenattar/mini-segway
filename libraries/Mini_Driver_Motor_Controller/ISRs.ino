
//======================================================= ISR for left encoder =======================================================
void Lencoder()
{
  lpulse=micros()-ltime;                               // time between last state change and this state change
  ltime=micros();                                      // update ltime with time of most recent state change
  lcount++;                                            // increment left motor distance counter
}


//======================================================= ISR for right encoder ======================================================
void Rencoder()
{
  rpulse=micros()-rtime;                               // time between last state change and this state change
  rtime=micros();                                      // update ltime with time of most recent state change
  rcount++;                                            // increment right motor distance counter
}


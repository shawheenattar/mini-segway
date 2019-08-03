 #include <RotaryEncoder.h>;
 RotaryEncoder encoder(52,53,5,6,1000);
 void setup()
 {  
   Serial.begin(9600);
   Serial2.begin(9600);
 }
 void loop()
 {
   int enc = encoder.readEncoder();
   if(enc != 0) {
     Serial2.println(enc);
   } 
   delayMicroseconds(5);
 }

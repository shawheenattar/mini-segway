int led = 7;
int wait = 5;
void setup() {
  pinMode(led, OUTPUT);
}
void loop() {
  for(int level=0;level<256;level+=1) {
  analogWrite(led, level);
  delay(wait);
}
  for(int level=255;level>0;level-=1) {
  analogWrite(led, level);
  delay(wait);
}
}

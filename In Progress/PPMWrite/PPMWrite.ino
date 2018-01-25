
#include "SBUS.h"
#include <Servo.h>

SBUS x8r(Serial2);
int out=0;
int maxthrottle=1811;
int minthrottle=172;
int dacmax=4096;
int dacmin=0;
Servo myservo; 
uint16_t channels[16];
uint8_t failSafe;
uint16_t lostFrames = 0;

void setup() {
  Serial.begin(115200);
  x8r.begin();
  myservo.attach(23,1000,2000);
  myservo.write(90);
  
}

int count=0;

void loop() {
  if(x8r.read(&channels[0], &failSafe, &lostFrames)){
    Serial.print("sensor0 = ");
    Serial.println(channels[2]);
    out=90+map(channels[2],minthrottle,maxthrottle,0,90);
    Serial.println(out);
    myservo.write(out); 
  }
}

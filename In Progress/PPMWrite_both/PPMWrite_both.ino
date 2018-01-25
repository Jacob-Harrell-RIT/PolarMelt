
#include "SBUS.h"
#include <Servo.h>

SBUS x8r(Serial2);
int out=0;
int out1=0;
int maxthrottle=1811;
int minthrottle=172;
int dacmax=4096;
int dacmin=0;
Servo myservo; 
Servo myservo1;
uint16_t channels[16];
uint8_t failSafe;
uint16_t lostFrames = 0;

void setup() {
  Serial.begin(115200);
  x8r.begin();
  myservo.attach(23,1000,2000);
  myservo1.attach(22,1000,2000);
  myservo.writeMicroseconds(1430);
  myservo1.writeMicroseconds(1430);
  
}

int count=0;

void loop() {
  if(x8r.read(&channels[0], &failSafe, &lostFrames)){
    Serial.print("sensor0 = ");
    Serial.println(channels[2]);
    //out=1000+map(channels[2],minthrottle,maxthrottle,430,1000);
    //note that the motors spin in opposite directions in this example
    out=map(channels[2],minthrottle,maxthrottle,1000,2000);
    //out1=1000+map(channels[2],minthrottle,maxthrottle,430,1000);
    out1=map(channels[2],minthrottle,maxthrottle,1000,2000);
    Serial.println(out);
    myservo.writeMicroseconds(out); 
    myservo1.writeMicroseconds(out1); 
    
  }
}

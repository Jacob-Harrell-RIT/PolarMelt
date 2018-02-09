
#include "SBUS.h"
#include <Adafruit_DotStar.h>
#include <Servo.h>
#include <SPI.h>  
#define NUMPIXELS 19
#define DATAPIN    4//17//5//17
#define CLOCKPIN   5//16//4//16
#define DATAPIN1  17
#define CLOCKPIN1 16
SBUS x8r(Serial2);
Adafruit_DotStar strip = Adafruit_DotStar(
  NUMPIXELS, DATAPIN, CLOCKPIN, DOTSTAR_BRG);
Adafruit_DotStar  strip2=Adafruit_DotStar(
  NUMPIXELS, DATAPIN1, CLOCKPIN1, DOTSTAR_BRG);
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
  strip.begin(); // Initialize pins for output
  strip2.begin();
  strip2.show();
  strip.show();  // Turn all LEDs off ASAP
}

int count=0;
int failsafe=0;
int      head  = 0, tail = -10; // Index of first 'on' and 'off' pixels
uint32_t color = 0xFF0000;   
void loop() {
  if(x8r.read(&channels[0], &failSafe, &lostFrames)){
     failsafe=0;
    //Serial.print("sensor0 = ");
    //Serial.println(channels[2]);
    //out=1000+map(channels[2],minthrottle,maxthrottle,430,1000);
    //note that the motors spin in opposite directions in this example
    out=map(channels[2],minthrottle,maxthrottle,1430,2000);
    //out1=1000+map(channels[2],minthrottle,maxthrottle,430,1000);
    out1=map(channels[2],minthrottle,maxthrottle,1430,1000);
    Serial.print("1:");
    Serial.println(out);
    Serial.print("2:");
    Serial.println(out1);
    myservo.writeMicroseconds(out); 
    myservo1.writeMicroseconds(out1); 
      strip.setPixelColor(head, color); // 'On' pixel at head
  strip.setPixelColor(tail, 0);     // 'Off' pixel at tail
  strip.show();                     // Refresh strip
  strip2.setPixelColor(head, color); // 'On' pixel at head
  strip2.setPixelColor(tail, 0);     // 'Off' pixel at tail
  strip2.show();                     // Refresh 
    if(++head >= NUMPIXELS) {         // Increment head index.  Off end of strip?
    head = 0;                       //  Yes, reset head index to start
    if((color >>= 8) == 0)          //  Next color (R->G->B) ... past blue now?
      color = 0xFF0000;             //   Yes, reset to red
  }
  if(++tail >= NUMPIXELS) tail = 0;
  }
  else if(failsafe>=2000){
  myservo.writeMicroseconds(1430);
  myservo1.writeMicroseconds(1430);
  failsafe=0;
  }
  else{
    failsafe++;
  }
}

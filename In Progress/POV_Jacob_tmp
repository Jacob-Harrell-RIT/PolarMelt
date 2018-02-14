#include "SBUS.h"
#include <Adafruit_DotStar.h>
#include <Servo.h>
#include <SPI.h>  

// Dotstar Constants
const int NUMPIXELS = 19;
const int DATAPIN = 4;
const int CLOCKPIN = 5;
const int DATAPIN1 = 17;
const int CLOCKPIN1 = 16;
// Color Constants
const int LED_OFF = 0x000000;
const int LED_ON = 0x000099;
const int LED_ERR = 0x009900;
const int LED_STRT = 0x660066;
const int LED_NEUT = 0x990000;
// Math Constants
const float ACCEL_RADIUS = 2.5; //cm
const float ADC_TO_THETA = 7.13377835; // Magic Number to convert from unitless ADC to unitless rotation
const int DT = 1.25; //ms
// Translation Profile


// Reciever
SBUS x8r(Serial2);

// Dotstar Strips
Adafruit_DotStar strip = Adafruit_DotStar(
  NUMPIXELS, DATAPIN, CLOCKPIN, DOTSTAR_BRG);
Adafruit_DotStar  strip2 =Adafruit_DotStar(
  NUMPIXELS, DATAPIN1, CLOCKPIN1, DOTSTAR_BRG);

int forward_flg = 0;

volatile int heading;

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

int count=0;
int failsafe=0;

IntervalTimer accTimer;

void setup() {
  // Begin the serial output
  Serial.begin(115200);
  // Begin the reciever
  x8r.begin();
  // Initialize the motors
  myservo.attach(23,1000,2000);
  myservo1.attach(22,1000,2000);
  myservo.writeMicroseconds(1430);
  myservo1.writeMicroseconds(1430);
  // Initialize the analog read granularity
  analogReadResolution(12);
  // Setup the heading
  heading = 1024;
  // Initialize pins for output
  strip.begin();
  strip2.begin();
  // Turn all LEDs off ASAP
  strip.show();
  strip2.show();
  // Then turn them into the LED_NEUT color
  lightLEDs(LED_STRT);
  // Start the timer
  accTimer.begin(readAccel,1250);
}

void lightLEDs(int color) {
  // For each pixel
  for (int i = 0; i < NUMPIXELS; i++) {
    // set the color
    strip.setPixelColor(i,color);
    strip2.setPixelColor(i,color);
  } // for each pixel
  // Show the results
  strip.show();
  strip2.show();
} // lightLEDs()

void readAccel() {
  // Average the readings
  int acc0 = analogRead(A7);
  int acc1 = analogRead(A6);
  float acc_avg = (acc0 + acc1) / 2;
  // Shift the average
  acc_avg -= 2047;
  // Calculate the Rotational Velocity
  float theta_dot = sqrt(acc_avg/ACCEL_RADIUS);
  // Add the change in heading
  heading += (theta_dot * DT * ADC_TO_THETA);
  // Loop the heading
  heading = heading % 4096;
}

void loop() { 
  if(x8r.read(&channels[0], &failSafe, &lostFrames)){
    Serial.println(failsafe);
    if (failSafe){
      //out1=1000+map(channels[2],minthrottle,maxthrottle,430,1000);
      out1=1430;
      out=1430;
      myservo1.writeMicroseconds(out1);
      myservo.writeMicroseconds(out);
      Serial.print("1err:");
      Serial.println(out);
      Serial.print("2err:");
      Serial.println(out1);
      lightLEDs(LED_ERR);
     }else{
      //Serial.print("sensor0 = ");
      //Serial.println(channels[2]);
      //out=1000+map(channels[2],minthrottle,maxthrottle,430,1000);
      // Check if the throttle is on or not
      if(channels[2] < minthrottle) {
        // If not, light the LEDs to a neutral color (to show that a connection exists, but no movement is happening
        lightLEDs(LED_NEUT);
      } // throttle on?
      //note that the motors spin in opposite directions in this example
      out=map(channels[2],minthrottle,maxthrottle,1430,2000);
      //out1=1000+map(channels[2],minthrottle,maxthrottle,430,1000);
      out1=map(channels[2],minthrottle,maxthrottle,1430,2000);//1430,998);
      Serial.print("1:");
      Serial.println(out);
      Serial.print("2:");
      Serial.println(out1);
      myservo.writeMicroseconds(out); 
      myservo1.writeMicroseconds(out1);
      // Check the heading to light the LEDs
      if(heading < 1360 && heading > 683) {
        Serial.print("Heading is Forward at: ");
        Serial.println(heading);
        if (!forward_flg) {
          lightLEDs(LED_ON);
          forward_flg = 1;
        } // forward_flg
      } else {
        Serial.print("Heading is NOT Forward at: ");
        Serial.println(heading);
        if (forward_flg) {
          lightLEDs(LED_OFF);
          forward_flg = 0;
        } // forward_flg
      } // forward?
    } // failsafe?
  } // Signal read?
} // loop()

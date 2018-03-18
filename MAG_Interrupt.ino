#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303.h>

Adafruit_LSM303 lsm;

volatile int magx, magy, magz;
volatile int accx, accy, accz;
int gravity=0;//0 neutral +100 up -100 down
volatile bool magflg = 0;

void setup() 
{
  Serial.begin(115200);
  
  // Try to initialise and warn if we couldn't detect the chip
  if (!lsm.begin())
  {
    Serial.println("Oops ... unable to initialize the LSM303. Check your wiring!");
    while (1);
  }

  pinMode(12, INPUT);
  attachInterrupt(12, readMag, FALLING);

  pinMode(13, OUTPUT);
}

void loop() 
{
  if(magflg) {
    lsm.read();
    //magx = (int)lsm.magData.x;
    magy = (int)lsm.magData.y;
    magz = (int)lsm.magData.z;
    accx = (int)lsm.accelData.x;
    //gravity threshold
    if(accx>1500){
      gravity=100;
    }
    else if (accx<-1500){
      gravity=-100;
    }
    else{
      gravity=0;
    }
    
    //accy = (int)lsm.accelData.y;
    //accz = (int)lsm.accelData.z;
    //Serial.print("Mag X: "); Serial.print(magx);     Serial.print(" ");
    Serial.print("Y: "); Serial.print(magy);         Serial.print(" ");
    Serial.print("Z: "); Serial.print(magz);       Serial.print(" ");
    Serial.print("X: "); Serial.println(gravity);     //Serial.print(" ");
    //Serial.print("Y: "); Serial.print(accy);         Serial.print(" ");
    //Serial.print("Z: "); Serial.println(accz);       Serial.print(" ");
    digitalWrite(13,LOW);
    magflg = 0;
  } else {
  }
}

void readMag() {
  magflg = 1;
  digitalWrite(13,HIGH);
}

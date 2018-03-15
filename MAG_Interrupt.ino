#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303.h>

Adafruit_LSM303 lsm;

volatile int magx, magy, magz;
volatile int accx, accy, accz;
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
    magx = (int)lsm.magData.x;
    magy = (int)lsm.magData.y;
    magz = (int)lsm.magData.z;
    accx = (int)lsm.accelData.x;
    accy = (int)lsm.accelData.y;
    accz = (int)lsm.accelData.z;
    Serial.print("Mag X: "); Serial.print(magx);     Serial.print(" ");
    Serial.print("Y: "); Serial.print(magy);         Serial.print(" ");
    Serial.print("Z: "); Serial.println(magz);       Serial.print(" ");
    Serial.print("Accel X: "); Serial.print(accx);     Serial.print(" ");
    Serial.print("Y: "); Serial.print(accy);         Serial.print(" ");
    Serial.print("Z: "); Serial.println(accz);       Serial.print(" ");
    digitalWrite(13,LOW);
    magflg = 0;
  } else {
  }
}

void readMag() {
  magflg = 1;
  digitalWrite(13,HIGH);
}

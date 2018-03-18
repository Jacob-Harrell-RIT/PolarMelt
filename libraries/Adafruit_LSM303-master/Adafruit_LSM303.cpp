/***************************************************************************
  This is a library for the LSM303 Accelerometer and magnentometer/compass

  Designed specifically to work with the Adafruit LSM303DLHC Breakout

  These displays use I2C to communicate, 2 pins are required to interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!

  Written by Kevin Townsend for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ***************************************************************************/
#include <Adafruit_LSM303.h>

/***************************************************************************
 CONSTRUCTOR
 ***************************************************************************/
bool Adafruit_LSM303::begin()
{
  Wire.begin();

  // Enable the accelerometer
  write8(LSM303_ADDRESS_ACCEL, LSM303_REGISTER_ACCEL_CTRL_REG1_A, 0x27);
  
  // Set Data Output Rate for Magnometer
  write8(LSM303_ADDRESS_MAG, LSM303_REGISTER_MAG_CRA_REG_M, 0x1C);
  write8(LSM303_ADDRESS_MAG, LSM303_REGISTER_MAG_CRB_REG_M, (byte)LSM303_MAGGAIN_8_1 );
  // Enable the magnetometer
  write8(LSM303_ADDRESS_MAG, LSM303_REGISTER_MAG_MR_REG_M, 0x00);
  

  return true;
}

/***************************************************************************
 PUBLIC FUNCTIONS
 ***************************************************************************/
void Adafruit_LSM303::read()
{
  // Read the accelerometer
  Wire.beginTransmission((byte)LSM303_ADDRESS_ACCEL);
  Wire.write(LSM303_REGISTER_ACCEL_OUT_X_L_A | 0x80);
  Wire.endTransmission();
  Wire.requestFrom((byte)LSM303_ADDRESS_ACCEL, (byte)2);

  // Wait around until enough data is available
  while (Wire.available() < 2);

  uint8_t xlo = Wire.read();
  uint8_t xhi = Wire.read();
  //uint8_t ylo = Wire.read();
  //uint8_t yhi = Wire.read();
  //uint8_t zlo = Wire.read();
  //uint8_t zhi = Wire.read();

  // Shift values to create properly formed integer (low byte first)
  // KTOWN: 12-bit values are left-aligned, no shift needed
  // accelData.x = (xlo | (xhi << 8)) >> 4;
  // accelData.y = (ylo | (yhi << 8)) >> 4;
  // accelData.z = (zlo | (zhi << 8)) >> 4;
  accelData.x = (int16_t)((xhi << 8) | xlo);
  //accelData.y = (int16_t)((yhi << 8) | ylo);
  //accelData.z = (int16_t)((zhi << 8) | zlo);
  
  // Read the magnetometer
  Wire.beginTransmission((byte)LSM303_ADDRESS_MAG);
  Wire.write(LSM303_REGISTER_MAG_OUT_Z_H_M);
  Wire.endTransmission();
  Wire.requestFrom((byte)LSM303_ADDRESS_MAG, (byte)6);//TODO fix issue where MCU Crashes if only asks for 4 bytes
  
  // Wait around until enough data is available
  while (Wire.available() < 4);

  // Note high before low (different than accel)  
  //xhi = Wire.read();
  //xlo = Wire.read();
  uint8_t zhi = Wire.read();
  uint8_t zlo = Wire.read();
  uint8_t yhi = Wire.read();
  uint8_t ylo = Wire.read();
  
  // Shift values to create properly formed integer (low byte first)
  //magData.x = (xlo | (xhi << 8));
  magData.y = (ylo | (yhi << 8));
  magData.z = (zlo | (zhi << 8));  
  
  // ToDo: Calculate orientation
  magData.orientation = 0.0;
}

void Adafruit_LSM303::setMagGain(lsm303MagGain gain)
{
  write8(LSM303_ADDRESS_MAG, LSM303_REGISTER_MAG_CRB_REG_M, (byte)gain);
}

/***************************************************************************
 PRIVATE FUNCTIONS
 ***************************************************************************/
void Adafruit_LSM303::write8(byte address, byte reg, byte value)
{
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

byte Adafruit_LSM303::read8(byte address, byte reg)
{
  byte value;

  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(address, (byte)1);
  value = Wire.read();
  Wire.endTransmission();

  return value;
}

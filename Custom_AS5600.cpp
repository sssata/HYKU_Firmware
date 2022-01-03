#include <Wire.h>
#include "Custom_AS5600.h"


void as5600Setup()
{
  //        0123456789ABCDEF
  // 8,9 are filter settings
  setConf(0b0000000010000000);

  Wire.beginTransmission(0x36);
  Wire.write(0x0C);
  uint8_t err = Wire.endTransmission();
  if (err == 0) {
    Serial.println(F("AS5600 ERROR"));
  }
}

void setConf(word _conf)
{
  uint8_t err = writeOneByte(0x07, highByte(_conf));
  delay(2);
  writeOneByte(0x08, lowByte(_conf));
  delay(2);
}

/*******************************************************
  Method: getRawAngle
  In: none
  Out: value of raw angle register
  Description: gets raw value of magnet position.
  start, end, and max angle settings do not apply
*******************************************************/
word getRawAngleFast()
{
  return readTwoBytesFast();
}

/*******************************************************
  Method: writeOneByte
  In: address and data to write
  Out: none
  Description: writes one byte to a i2c register
*******************************************************/
uint8_t writeOneByte(int adr_in, int dat_in)
{
  Wire.beginTransmission(0x36);
  Wire.write(adr_in);
  Wire.write(dat_in);
  uint8_t err = Wire.endTransmission();
  return err;
}

/*******************************************************
  Method: readTwoBytes
  In: two registers to read
  Out: data read from i2c as a word
  Description: reads two bytes register from i2c
*******************************************************/
word readTwoBytesFast()
{
  word retVal = -1;

  // Automatic pointer reload for rawAngle
  //register, don't need to write reg addr
  //Wire.beginTransmission(0x36);
  //Wire.write(0x0c);
  //Wire.endTransmission(false);

  Wire.requestFrom(0x36, 2);
  while (Wire.available() < 2)
    ;

  word high = Wire.read();
  int low = Wire.read();

  high = high << 8;
  retVal = high | low;

  return retVal;
}
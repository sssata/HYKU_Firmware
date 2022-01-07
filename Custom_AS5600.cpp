#include <Wire.h>
#include "Custom_AS5600.h"

CustomAS5600::CustomAS5600()
{
    ///empty
}

uint8_t CustomAS5600::as5600Setup()
{
    uint8_t err;
    // Set Filter Speed
    // bits 8,9 are filter settings
    //                0123456789ABCDEF
    err = setConfRegister(0b0000000010000000);

    if (err != 0){
        return err;
    }
    Wire.beginTransmission(AS5600_ADDRESS);
    Wire.write(AS5600_RAW_ANGLE_ADDRESS_HIGH);
    err = Wire.endTransmission();
    if (err != 0)
    {
        Serial.println(F("AS5600 ERROR"));
    }

    return 0;
}

/*******************************************************
  Method: setConfRegister
  In: none
  Out: value of raw angle register
  Description: gets raw value of magnet position.
  start, end, and max angle settings do not apply
*******************************************************/
uint8_t CustomAS5600::setConfRegister(word _conf)
{
    uint8_t err;
    err = writeOneByte(AS5600_CONF_ADDRESS_HIGH, highByte(_conf));
    if (err != 0){
        return err;
    }
    delay(2);
    err = writeOneByte(AS5600_CONF_ADDRESS_LOW, lowByte(_conf));
    if (err != 0){
        return err;
    }
    delay(2);
    return 0;
}

/*******************************************************
  Method: getRawAngle
  In: none
  Out: value of raw angle register
  Description: gets raw value of magnet position.
  start, end, and max angle settings do not apply
*******************************************************/
word CustomAS5600::getRawAngleFast()
{
    word retVal = -1;

    if (!rawAngleAddressWritten)
    {
        Wire.beginTransmission(AS5600_ADDRESS);
        Wire.write(AS5600_RAW_ANGLE_ADDRESS_HIGH);
        Wire.endTransmission(false);
    }
    // Automatic pointer reload for rawAngle
    //register, don't need to write reg addr every time

    Wire.requestFrom(AS5600_ADDRESS, 2);
    while (Wire.available() < 2)
        ;

    word high = Wire.read();
    int low = Wire.read();

    high = high << 8;
    retVal = high | low;

    return retVal;
}

/*******************************************************
  Method: writeOneByte
  In: address and data to write
  Out: none
  Description: writes one byte to a i2c register
*******************************************************/
uint8_t CustomAS5600::writeOneByte(int adr_in, int dat_in)
{
    Wire.beginTransmission(AS5600_ADDRESS);
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
word CustomAS5600::readTwoBytesFast()
{
    word retVal = -1;

    // Automatic pointer reload for rawAngle
    //register, don't need to write reg addr
    //Wire.beginTransmission(0x36);
    //Wire.write(0x0c);
    //Wire.endTransmission(false);

    Wire.requestFrom(AS5600_ADDRESS, 2);
    while (Wire.available() < 2)
        ;

    word high = Wire.read();
    int low = Wire.read();

    high = high << 8;
    retVal = high | low;

    return retVal;
}
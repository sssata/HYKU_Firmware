#include <Wire.h>
#include "Custom_AS5600.h"

CustomAS5600::CustomAS5600()
{
    ///empty
}

/*******************************************************
  Method: as5600Setup
  In: void
  Out: Error code
    0 : Success
    1 : Data too long
    2 : NACK on transmit of address
    3 : NACK on transmit of data
    4 : Other error
  Description: Initializes the AS5600 by setting conf register bits and
*******************************************************/
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
    err = Wire.endTransmission(true);
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
  In: buffer to write to
  Out: Error Code:
    0 : Success
    1 : Data too long
    2 : NACK on transmit of address
    3 : NACK on transmit of data
    4 : Other error
    5 : Timeout on read
  Description: gets raw value of magnet position.
  start, end, and max angle settings do not apply
*******************************************************/
uint8_t CustomAS5600::getRawAngleFast(uint16_t *buffer)
{
    uint16_t retVal = -1;
    int status = 0;

    // Check if raw angle address is already written to sensor
    // Datasheet reference: Automatic Increment of the Address Pointer for ANGLE, RAW ANGLE and MAGNITUDE Registers
    if (!rawAngleAddressWritten)
    {
        // Write raw angle register if not already set
        Wire.beginTransmission(AS5600_ADDRESS);
        Wire.write(AS5600_RAW_ANGLE_ADDRESS_HIGH);
        status = Wire.endTransmission(false);
        rawAngleAddressWritten = true;
    }

    // Check for error
    if (status != 0){
        return status;
    }
    // Automatic pointer reload for rawAngle
    //register, don't need to write reg addr every time

    unsigned long request_from_start_time = micros();

    Wire.requestFrom(AS5600_ADDRESS, 2, true);


    // wait until available
    while (Wire.available() < 2){
        
        // Check for timeout
        if ((micros() - request_from_start_time) > 1000){
            return 5;
        }
    }

    uint16_t high = Wire.read();
    uint16_t low = Wire.read();

    high = high << 8;
    retVal = high | low;

    *buffer = retVal;

    return 0;
}

/*******************************************************
  Method: writeOneByte
  In: address and data to write
  Out: Error Code
  Description: writes one byte to a i2c register
*******************************************************/
uint8_t CustomAS5600::writeOneByte(int adr_in, int dat_in)
{
    Wire.beginTransmission(AS5600_ADDRESS);
    Wire.write(adr_in);
    Wire.write(dat_in);
    uint8_t err = Wire.endTransmission(true);
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
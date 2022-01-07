#pragma once

#include <Arduino.h>

#define AS5600_ADDRESS 0x36
#define AS5600_RAW_ANGLE_ADDRESS_HIGH 0x0C
#define AS5600_RAW_ANGLE_ADDRESS_LOW 0x0D
#define AS5600_CONF_ADDRESS_HIGH 0x07
#define AS5600_CONF_ADDRESS_LOW 0x08


class CustomAS5600 {
    private:
        bool rawAngleAddressWritten;

    public:
        CustomAS5600();

        uint8_t as5600Setup();

        uint8_t setConfRegister(word _conf);

        word getRawAngleFast();

        uint8_t writeOneByte(int adr_in, int dat_in);

        word readTwoBytesFast();
};
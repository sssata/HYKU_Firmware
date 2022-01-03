#pragma once

#include <Arduino.h>

void as5600Setup();

void setConf(word _conf);

word getRawAngleFast();

uint8_t writeOneByte(int adr_in, int dat_in);

word readTwoBytesFast();
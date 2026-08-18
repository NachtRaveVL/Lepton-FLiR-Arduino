#pragma once
#include "Arduino.h"

#define I2C_FASTMODE 1

boolean i2c_init(void);
bool i2c_start(uint8_t addr);
void LEPFLIR_i2c_stop(void);
bool LEPFLIR_i2c_write(uint8_t value);
uint8_t i2c_read(bool last);

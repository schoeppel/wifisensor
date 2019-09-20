#pragma once

#include <inttypes.h>

void i2c_start();
void i2c_stop();
uint8_t i2c_write(uint8_t data);
uint8_t i2c_read(uint8_t ack);

void i2c_init();

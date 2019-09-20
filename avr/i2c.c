#include "i2c.h"

#include <avr/io.h>

#define SDA (1<<5)
#define SCL (1<<4)
#define I2C_VPORT VPORTA
#define I2C_PORT PORTA

void i2c_init() {
	I2C_VPORT.DIR &= ~(SDA | SCL);
	I2C_VPORT.OUT &= ~(SDA | SCL);
}

__attribute__((always_inline)) static inline void i2c_wait() {
	__asm__("nop");
	__asm__("nop");
}

__attribute__((always_inline)) static inline void sda_lo() {
	I2C_VPORT.DIR |= SDA;
}

__attribute__((always_inline)) static inline void sda_hi() {
	I2C_VPORT.DIR &= ~SDA;
}

__attribute__((always_inline)) static inline void scl_lo() {
	I2C_VPORT.DIR |= SCL;
	i2c_wait();
}

__attribute__((always_inline)) static inline void scl_hi() {
	I2C_VPORT.DIR &= ~SCL;
	i2c_wait();
}

void i2c_start() {
	sda_hi();
	scl_hi();
	sda_lo();
	scl_lo();
}

void i2c_stop() {
	sda_lo();
	scl_hi();
	sda_hi();
}

uint8_t i2c_write(uint8_t data) {
	uint8_t ack;
	
	for (uint8_t i = 0; i < 8; i++) {
		if (data & 0x80)
			sda_hi();
		else
			sda_lo();

		scl_hi();
		scl_lo();
		data <<= 1;
	}

	sda_hi();
	scl_hi();
	ack = !(I2C_VPORT.IN & SDA);
	scl_lo();
	
	return ack;
}

uint8_t i2c_read(uint8_t ack) {
	uint8_t data = 0;
	
	sda_hi();
	for (uint8_t i = 0; i < 8; i++) {
		data <<= 1;
		scl_hi();
		if (I2C_VPORT.IN & SDA) data++;
		scl_lo();
	}
	
    if (ack)
		sda_lo();
	scl_hi();
	scl_lo();
	return data;
}

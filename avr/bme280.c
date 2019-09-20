#include "bme280.h"

#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>

#include "i2c.h"

#define ADDR 0x76

#define REG_CONFIG 0xf5
#define REG_CTRL_MEAS 0xf4
#define REG_STATUS 0xf3
#define REG_STATUS_MEASURING (1<<3)
#define REG_CTRL_HUM 0xf2
#define REG_ID 0xd0

//#define BME_DEBUG 1

struct bme_cal_data {
	// 0x88
	uint16_t dig_T1;
	int16_t dig_T2;
	int16_t dig_T3;
	
	// 0x8E
	uint16_t dig_P1;
	int16_t dig_P2;
	int16_t dig_P3;
	int16_t dig_P4;
	int16_t dig_P5;
	int16_t dig_P6;
	int16_t dig_P7;
	int16_t dig_P8;
	int16_t dig_P9;

	// 0xA1
	uint8_t dig_H1;
	// 0xE1
	int16_t dig_H2;
	// 0xE3
	uint8_t dig_H3;
	
	// 0xE4
	int16_t dig_H4;
	int16_t dig_H5;
	int8_t dig_H6;	
} __attribute__((packed));

struct bme_data {
	// 0xF7
	uint8_t press_msb;
	uint8_t press_lsb;
	uint8_t press_xlsb;
	// 0xFA
	uint8_t temp_msb;
	uint8_t temp_lsb;
	uint8_t temp_xlsb;
	// 0xFD
	uint8_t hum_msb;
	uint8_t hum_lsb;
} __attribute__((packed));

static struct bme_cal_data cal;
static int32_t t_fine;

uint8_t bme280_write(uint8_t reg, uint8_t data) {
	i2c_start();
	uint8_t ack = i2c_write(ADDR << 1);
	if (! ack) goto err;
	ack = i2c_write(reg);
	if (! ack) goto err;
	ack = i2c_write(data);
	if (! ack) goto err;
	i2c_stop();
	return 0;
	
	err:
	i2c_stop();
	return 1;
}

uint8_t bme280_read(uint8_t reg, void *data, uint8_t len) {
	i2c_start();
	uint8_t ack = i2c_write(ADDR << 1);
	if (! ack) goto err;
	ack = i2c_write(reg);
	if (! ack) goto err;
	
	i2c_start();
	ack = i2c_write((ADDR << 1) | 1); // read
	if (! ack) goto err;
	
	while (len > 0) {
		*((uint8_t*)data) = i2c_read(len > 1);
		len--;
		data++;
	}
	i2c_stop();
	return 0;
	
	err:
	i2c_stop();
	return 1;
}



uint8_t bme280_init() {
	uint8_t ret = 0;
	uint8_t id = 0;
	ret |= bme280_read(REG_ID, &id, 1);
	ret |= bme280_read(0x88, &cal.dig_T1, 24);
	ret |= bme280_read(0xA1, &cal.dig_H1, 1);
	ret |= bme280_read(0xE1, &cal.dig_H2, 3);
	uint8_t buf[4];
	ret |= bme280_read(0xE4, &buf, 4);
	cal.dig_H4 = buf[0]<<4 | (buf[1] & 0xf);
	cal.dig_H5 = buf[1]>>4 | buf[2]<<4;
	cal.dig_H6 = buf[3];

#ifdef BME_DEBUG
	printf("ID = %x\n", id);
	
	printf("T1 = %u\n", cal.dig_T1);
	printf("T2 = %i\n", cal.dig_T2);
	printf("T3 = %i\n", cal.dig_T3);
	
	printf("P1 = %u\n", cal.dig_P1);
	printf("P2 = %i\n", cal.dig_P2);
	printf("P3 = %i\n", cal.dig_P3);
	printf("P4 = %i\n", cal.dig_P4);
	printf("P5 = %i\n", cal.dig_P5);
	printf("P6 = %i\n", cal.dig_P6);
	printf("P7 = %i\n", cal.dig_P7);
	printf("P8 = %i\n", cal.dig_P8);
	printf("P9 = %i\n", cal.dig_P9);
	
	printf("H1 = %i\n", cal.dig_H1);
	printf("H2 = %i\n", cal.dig_H2);
	printf("H3 = %i\n", cal.dig_H3);
	printf("H4 = %i\n", cal.dig_H4);
	printf("H5 = %i\n", cal.dig_H5);
	printf("H6 = %i\n", cal.dig_H6);
#endif
	
	ret |= bme280_write(REG_CTRL_MEAS, 0xAC);
	ret |= bme280_write(REG_CTRL_HUM, 3); // H: 4x
	return ret;
}

static int32_t bme280_get_temp(int32_t adc_T) {
	int32_t var1, var2, T;
	var1 = ((((adc_T>>3) - ((int32_t)cal.dig_T1<<1))) * ((int32_t)cal.dig_T2)) >> 11;
	var2 = (((((adc_T>>4) - ((int32_t)cal.dig_T1)) * ((adc_T>>4) - ((int32_t)cal.dig_T1))) >> 12) * ((int32_t)cal.dig_T3)) >> 14;
	t_fine = var1 + var2; 
	T = (t_fine * 50 + 128) >> 8;
	return T;
}

static uint32_t bme280_get_pressure(int32_t adc_P) {
	int64_t var1, var2, p;
	var1 = ((int64_t)t_fine) - 128000;
	var2 = var1 * var1 * (int64_t)cal.dig_P6;
	var2 = var2 + ((var1*(int64_t)cal.dig_P5)<<17);
	var2 = var2 + (((int64_t)cal.dig_P4)<<35);
	var1 = ((var1 * var1 * (int64_t)cal.dig_P3)>>8) + ((var1 * (int64_t)cal.dig_P2)<<12);
	var1 = (((((int64_t)1)<<47)+var1))*((int64_t)cal.dig_P1)>>33;
	if (var1 == 0) return 0;
	p = 1048576-adc_P;
	p = (((p<<31)-var2)*3125)/var1;
	var1 = (((int64_t)cal.dig_P9) * (p>>13) * (p>>13)) >> 25;
	var2 = (((int64_t)cal.dig_P8) * p) >> 19;
	p = ((p + var1 + var2) >> 8) + (((int64_t)cal.dig_P7)<<4);
	p *= 1000;
	p >>= 8;
	return (uint32_t)p;
}

static uint32_t bme280_get_humidity(int32_t adc_H) {
	int32_t v_x1_u32r;
	v_x1_u32r = (t_fine - ((int32_t)76800));
	v_x1_u32r = (((((adc_H << 14) - (((int32_t)cal.dig_H4) << 20) - (((int32_t)cal.dig_H5) * v_x1_u32r)) +
		((int32_t)16384)) >> 15) * (((((((v_x1_u32r * ((int32_t)cal.dig_H6)) >> 10) * (((v_x1_u32r * 
		((int32_t)cal.dig_H3)) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) *
		((int32_t)cal.dig_H2) + 8192) >> 14));
	v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32_t)cal.dig_H1)) >> 4));
	v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
	v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
	uint32_t res = (uint32_t)(v_x1_u32r>>12);
	res *= 1000;
	res /= 1024;
	return res;
}

uint8_t bme280_meas(struct bme_result *res) {
	uint8_t ret = 0;
	ret = bme280_write(REG_CTRL_MEAS, 0xAD); // T: 16x P: 4x, forced mode
	if (ret) return 1;
		
	uint8_t mclkb = CLKCTRL.MCLKCTRLB;
	CCP = CCP_IOREG_gc;
	CLKCTRL.MCLKCTRLB = CLKCTRL_PDIV_64X_gc | CLKCTRL_PEN_bm; // 312.5KHz
	
	uint8_t regval;
	do {
		bme280_read(REG_STATUS, &regval, 1);
		_delay_ms(1);
	} while (regval & REG_STATUS_MEASURING);
	
	CCP = CCP_IOREG_gc;
	CLKCTRL.MCLKCTRLB = mclkb;
	
	struct bme_data data;
	ret = bme280_read(0xF7, &data, sizeof(data));
	if (ret) return 1;
	ret = bme280_write(REG_CTRL_MEAS, 0xAC);
	if (ret) return 1;
	
	unsigned long adc_P = (uint32_t)data.press_msb<<12 | data.press_lsb<<4 | data.press_xlsb >> 4;
	unsigned long adc_T = (uint32_t)data.temp_msb<<12 | data.temp_lsb<<4 | data.temp_xlsb >> 4;
	uint16_t adc_H = data.hum_msb<<8 | data.hum_lsb;
	
#ifdef BME_DEBUG
	printf("P = %lx\n", adc_P);
	printf("T = %lx\n", adc_T);
	printf("H = %x\n", adc_H);
#endif
	
	res->temp = bme280_get_temp(adc_T);
	res->press = bme280_get_pressure(adc_P);
	res->hum = bme280_get_humidity(adc_H);
	
	return 0;
}


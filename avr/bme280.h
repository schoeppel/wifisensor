#pragma once

#include <inttypes.h>

struct bme_result {
	int32_t temp;
	uint32_t press;
	uint32_t hum;
};

uint8_t bme280_init();
uint8_t bme280_meas(struct bme_result *res);

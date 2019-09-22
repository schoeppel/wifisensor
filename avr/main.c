#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>

#include <util/delay.h>
#include <stdio.h>
#include <string.h>

#include "bme280.h"
#include "i2c.h"
#include "i2c_slave.h"

#define LEDA_K (1<<7)
#define LEDA_A (1<<6)
#define BOOST_EN (1<<1)
#define ESP_RST (1<<2)

#define abs(x) ((x)>0?(x):-(x))

#define DEBUG_BAUDRATE 9600

int usart_putchar(char c, FILE *stream);
FILE mystdout = FDEV_SETUP_STREAM(usart_putchar, NULL, _FDEV_SETUP_WRITE);

ISR(RTC_PIT_vect) {
	RTC.PITINTFLAGS = RTC_PI_bm;
}

static void usart_init() {
	USART0.BAUD = (F_CPU * 64UL) / (16UL * DEBUG_BAUDRATE);
	USART0.CTRLB = USART_TXEN_bm;
	USART0.CTRLC = USART_CHSIZE_8BIT_gc;
	
	VPORTB.DIR |= PIN2_bm;
	stdout = &mystdout;
}

int usart_putchar(char c, FILE *stream) {
	while ((USART0.STATUS & USART_DREIF_bm) == 0);
	USART0.TXDATAL = c;
	return 0;
}

static void rtc_init() {
	while (RTC.STATUS != 0) {}
	RTC.CTRLA = RTC_RTCEN_bm;
	
	while (RTC.PITSTATUS != 0) {}
	RTC.PITCTRLA = RTC_PERIOD_CYC32768_gc | RTC_PITEN_bm;
	RTC.PITINTCTRL = RTC_PI_bm;
	RTC.CLKSEL = RTC_CLKSEL_INT1K_gc;
}

static uint16_t get_battery_voltage() {
	ADC0.CTRLB = 0;
	ADC0.CTRLC = ADC_PRESC_DIV16_gc | ADC_REFSEL_VDDREF_gc;
	ADC0.CTRLD = ADC_INITDLY_DLY16_gc;
	ADC0.MUXPOS = ADC_MUXPOS_INTREF_gc;
	ADC0.CTRLA = ADC_ENABLE_bm;
	
	ADC0.COMMAND = ADC_STCONV_bm;
	while (ADC0.COMMAND) {
	}
	uint16_t digits = ADC0.RES;
	ADC0.CTRLA = 0;
	return (550ULL * 1024ULL) / digits;
}

static void rtc_sleep() {
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);
	sleep_mode();
}

static void esp_power(uint8_t enable) {
	if (enable) {
		VPORTA.OUT |= BOOST_EN;
		PORTB.PIN0CTRL = PORT_PULLUPEN_bm;
		PORTB.PIN1CTRL = PORT_PULLUPEN_bm;		

		
	} else {
		PORTB.PIN0CTRL = PORT_ISC_INPUT_DISABLE_gc;
		PORTB.PIN1CTRL = PORT_ISC_INPUT_DISABLE_gc;
		VPORTA.OUT &= ~BOOST_EN;
	}
}

uint16_t get_brightness() {
	VPORTA.OUT |= LEDA_K;
	_delay_us(50);
	
	uint8_t mclkb = CLKCTRL.MCLKCTRLB;
	CCP = CCP_IOREG_gc;
	CLKCTRL.MCLKCTRLB = CLKCTRL_PDIV_64X_gc | CLKCTRL_PEN_bm; // 312.5KHz
	
	VPORTA.DIR &= ~LEDA_K;
	
	TCB0.CTRLA = TCB_ENABLE_bm;
	uint16_t ts_start = TCB0.CNT;
	uint16_t period = 0;
	
	do {
		period = TCB0.CNT - ts_start;
	} while ((VPORTA.IN & LEDA_K) && period < 50000);
	
	CCP = CCP_IOREG_gc;
	CLKCTRL.MCLKCTRLB = mclkb;
	
	VPORTA.OUT &= ~LEDA_K;
	VPORTA.DIR |= LEDA_K;
	
	return period;
}


int main(void) {
	CCP = CCP_IOREG_gc;
	CLKCTRL.MCLKCTRLB = CLKCTRL_PDIV_4X_gc | CLKCTRL_PEN_bm;  // 5MHz
	
	PORTA.PIN0CTRL |= PORT_PULLUPEN_bm;
	
	// ADC
	PORTA.PIN3CTRL |= PORT_ISC_INPUT_DISABLE_gc;
	for (uint8_t i = 4; i < 6; i++) {
		*(&PORTA.PIN0CTRL + i) |= PORT_PULLUPEN_bm;
	}
	
	for (uint8_t i = 2; i < 4; i++) {
		*(&PORTB.PIN0CTRL + i) |= PORT_PULLUPEN_bm;
	}
	
	VPORTA.OUT = 0;
	VPORTB.OUT = 0;
	VPORTA.DIR = BOOST_EN | ESP_RST | LEDA_K | LEDA_A; // BOOST and RST as OUT
	
	esp_power(0);
	
	usart_init();
	rtc_init();
	
	i2c_init();
	i2c_slave_init();
	sei();
	
	int ret = bme280_init();
	if (ret) {
		printf("bme280_init: %i\n", ret);
	}
	
	printf("init\n");
		
	uint32_t uptime = 0;
	uint32_t wakeup_count  = 0;
	
	_delay_ms(100);
	
	uint8_t sleep_count = 255;
	struct bme_result res_last_wakeup; 
	
	while (1) {
		uint8_t debug_mode = ! (VPORTB.IN & (1<<3));
		
		if (debug_mode) {
				printf("enabled debug_mode\n");
				_delay_ms(100);
		}
		
		struct bme_result res;
		uint8_t ret = bme280_meas(&res);	// 1mA
		if (ret) {
			printf("BME error\n");
		}
		
		if (debug_mode) {
			printf("T= %li\n", res.temp);
			printf("P = %lu\n", res.press);
			printf("H = %lu\n", res.hum);
			_delay_ms(100);
		}
	
		uint8_t do_wakeup = 0;
		if (sleep_count > 6) do_wakeup = 1;
		int32_t temp_diff = res.temp - res_last_wakeup.temp;
		int32_t hum_diff = res.hum - res_last_wakeup.hum;
		if (temp_diff > 200 || temp_diff < -200) do_wakeup = 1;
		if (hum_diff > 2000 || temp_diff < -2000) do_wakeup = 1;
		
		if (do_wakeup) {
			// Wakeup esp8266
			wakeup_count++;
			uint16_t battery_voltage = get_battery_voltage();
			uint16_t brightness_raw = get_brightness();
			if (brightness_raw == 0) brightness_raw = 1;
			uint32_t brightness = ((uint32_t)battery_voltage * 256ULL) / brightness_raw;
			if (brightness_raw > 50000) brightness = 0;
			if (brightness > 65535) brightness = 65535;
			uint16_t brightness_for_esp = brightness;
			
			if (battery_voltage < 2200) {
				printf("low battery\nLOWBAT\n");
				rtc_sleep();
				continue;
			}
			
			if (debug_mode) printf("W = %lu\n", wakeup_count);
			memcpy((uint8_t*)i2c_slave_buf, &res.temp, 4);
			memcpy((uint8_t*)i2c_slave_buf + 4, &res.press, 4);
			memcpy((uint8_t*)i2c_slave_buf + 8, &res.hum, 4);
			memcpy((uint8_t*)i2c_slave_buf + 12, &uptime, 4);
			memcpy((uint8_t*)i2c_slave_buf + 16, &wakeup_count, 4);
			memcpy((uint8_t*)i2c_slave_buf + 20, &battery_voltage, 2);
			memcpy((uint8_t*)i2c_slave_buf + 22, &brightness_for_esp, 2);
			
			uint16_t ts_on = RTC.CNT;
			//printf("%u esp ON\n", ts_on);
			i2c_slave_buf[255] = 0;
			if (debug_mode) printf("ESP on\n");
			esp_power(1);
			
			uint8_t error = 0;
			while (i2c_slave_buf[255] == 0) {
				uint16_t on_period = RTC.CNT - ts_on;
				if (on_period > 10000 && ! debug_mode) {
					printf("ESP timeout\n");
					error = 1;
					break;
				}			
			}
			
			if (! error) {
				sleep_count = 0;
				res_last_wakeup = res;
			}
		}
		
		if (! debug_mode) {
			esp_power(0);
			// wakeup every 5 minutes
			for (uint8_t i = 0; i < 9; i++) {
				rtc_sleep();
				_delay_us(5);
				uptime += 33;
			}
			
			sleep_count++;
		} else {
			_delay_ms(19000);
			printf("ESP off\n");
			esp_power(0);
			_delay_ms(1000);
			uptime += 20;
		}
	}
}


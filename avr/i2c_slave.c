#include <avr/io.h>
#include <avr/interrupt.h>
volatile uint8_t i2c_slave_buf[256];

ISR(TWI0_TWIS_vect) {
	static uint8_t address_received;
	static uint8_t i2c_slave_buf_addr;

	
	uint8_t status = TWI0.SSTATUS;
	
	if (status & TWI_APIF_bm) { // Address match or STOP
		
		/* Handle Collision flag */
		if (status & TWI_COLL_bm) {
			TWI0.SSTATUS |= TWI_COLL_bm;
			TWI0.SCTRLB = TWI_SCMD_COMPTRANS_gc;
			return;
		}
		
		if (status & TWI_AP_bm) {
			TWI0.SCTRLB = TWI_SCMD_RESPONSE_gc; // Address match: send ACK
		} else {
			address_received = 0;
			i2c_slave_buf_addr = 0;
			TWI0.SCTRLB = TWI_SCMD_COMPTRANS_gc; // STOP
		}
	}
	
	if (status & TWI_DIF_bm) { // Data
		if (status & TWI_DIR_bm) { // Master read
			TWI0.SDATA = i2c_slave_buf[i2c_slave_buf_addr++];
			TWI0.SCTRLB = TWI_SCMD_RESPONSE_gc;
		} else { // Master write
			TWI0.SCTRLB = TWI_SCMD_RESPONSE_gc;
			if (address_received) {
				i2c_slave_buf[i2c_slave_buf_addr++] = TWI0.SDATA;
			} else {
				i2c_slave_buf_addr = TWI0.SDATA;
				address_received = 1;
			}
		}
	}
}

void i2c_slave_init() {
		TWI0.SADDR = 0x50<<1;
		TWI0.SCTRLA = TWI_DIEN_bm | TWI_APIEN_bm | TWI_ENABLE_bm | TWI_SMEN_bm | TWI_PIEN_bm;
}

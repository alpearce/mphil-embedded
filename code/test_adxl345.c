#include <avr/io.h>
#include <avr/interrupt.h>

#include "usart2.h"
#include "spi2.h"
#include "config.h"

#include <util/delay.h>
#include <util/delay_basic.h>



/* Fuses  H 0xDF  (default)
          L 0xE2  (8MHz internal oscillator, no CLKOUT)


  Simple Demo for adxl345 with SPI communication using spi2.c (Need to download new version of spi2)
  
  SPI is 4 sire mode (SCK, MOSI, MISO, CS)
  CS is on PB4, if you change it then do a make clean

  Serial is 19200 baud 8N1
  
*/




void port_direction_init(void) {

/* define outputs for PORTB */
	DDRB = 0xBF ;									// Note that I had to set MOSI, SCK and SS all to outputs.
													// Setting SPE in SPCR doesn't do the output setting it would seem.
/* define outputs for PORTC */
	DDRC = 0 ;
/* define outputs for PORTD */
	DDRD = 0;
	
/* Initial conditions for PORTB */		
	PORTB = 0xBF ;	// ADXL345 inactive
/* Initial conditions for PORTC */		
	PORTC = 0 ;											// no pullups
/* Initial conditions for PORTD */		
	PORTD = 0 ;											// no pullups						
}



void write_adxl_reg(uint8_t reg,uint8_t data) {

	PORTB &= ~(1<<ADXL345_CS);
	
	xmit_spi(reg & 0x3F);							// single byte and read both set to 0
	xmit_spi(data);
	
	PORTB |= (1<<ADXL345_CS);
	_delay_us(2);

}



uint8_t read_adxl_reg(uint8_t reg) {

	uint8_t returnval ;
	
	PORTB &= ~(1<<ADXL345_CS);
	
	xmit_spi((reg & 0xBF) | 0x80);				// clear Multi byte, set R
	returnval = recv_spi();
	
	PORTB |= (1<<ADXL345_CS);
	_delay_us(2);
	
	return returnval;

}


uint8_t burst_read_adxl_reg(uint8_t *dest, uint8_t reg, uint8_t bytes) {

	uint8_t i ;
	
	PORTB &= ~(1<<ADXL345_CS);
	xmit_spi(reg | 0xC0);				// set Multi byte, set R
	for (i=0;i<bytes;i++) {
		*dest++ = recv_spi();
	}
	PORTB |= (1<<ADXL345_CS);
	_delay_us(2);
}



void init_adxl345() {

	write_adxl_reg(0x31,0x08);		// SPI 4 bit, full resolution

	_delay_us(20);					// Probably not needed

	write_adxl_reg(0x2D,0x03);		// standby
	write_adxl_reg(0x2D,0x0B);		// measure
	write_adxl_reg(0x2E,0);			// no interrupts
	write_adxl_reg(0x38,0x80);		// stream mode

}




int main(void) {
		
	uint8_t readings[16] ;
	uint8_t i ;
	uint16_t x;
	uint16_t y;
	uint16_t z;
	uint16_t xsign;
	uint16_t ysign;
	uint16_t zsign;
	
	port_direction_init();
	USART0_init(25);							// 19200 baud. no interrupts
	
	init_spi(3,3);								// mode3 and slow as possible
												// **** Download latest spi2.c ****
	
	init_adxl345();
	
//	SREG |= 0x80;								// not using interrupts
		
	while (1) {

		USART0_transmit_uint8(read_adxl_reg(0));
		USART0_transmit('\r');

		burst_read_adxl_reg(readings,0x32,6);
		x = (readings[1]<<8) + readings[0];
		y = (readings[3]<<8) + readings[2];
		z = (readings[5]<<8) + readings[4];
		
		xsign = ' ';
		if (x > 32768) {
			xsign = '-';
			x = -x ;
		}
		USART0_transmit(xsign);
		USART0_transmit_uint16(x);
		USART0_transmit(' ');
		ysign = ' ';
		if (y > 32768) {
			ysign = '-';
			y = -y ;
		}
		USART0_transmit(ysign);
		USART0_transmit_uint16(y);
		USART0_transmit(' ');
		zsign = ' ';
		if (z > 32768) {
			zsign = '-';
			z = -z ;
		}
		USART0_transmit(zsign);
		USART0_transmit_uint16(z);
		USART0_transmit('\r');
		_delay_ms(1000);
	}
}



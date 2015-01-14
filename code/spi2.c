/* 25/11/2013  svn 282  */


// this library only requires the definitions of registers in the Atmel.

/******************		
******************
	IMPORTANT
		The spi function built in to the Microcontroller needs to
		be initialised before the device is used. 
		Note that you MUST set the mode, and the rate, though setting rate=3 should always work.
		See the init_spi() funtion in spi2.h for details
******************		
******************/		

#include <avr/io.h>
#include "spi2.h"



inline void init_spi(uint8_t mode,uint8_t rate) {
	SPCR = (1<<SPE) | (1<<MSTR) | ((mode & 0x03) << 2) | (rate & 0x03) ;
}


inline void xmit_spi(uint8_t dat) {
	SPDR=dat;
	while(!(SPSR & (1<<SPIF)))
		;
}


inline uint8_t recv_spi(void) {
	SPDR = 0xFF;
	while(!(SPSR & (1<<SPIF)))
		;
	return SPDR;
}	


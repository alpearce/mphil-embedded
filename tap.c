#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include "usart2.h"
#include "spi2.h"
#include "config.h"
#include "adxl345.h"

#define F_CPU 8E6 // 8MHz

#include <util/delay.h>
#include <util/delay_basic.h>

#define THRESH_TAP 70 
#define DUR 16 // max duration for a tap
#define LATENT 0 //disable double taps
#define WINDOW 0 //disable double taps 
#define TIME_INACT 0
#define TAP_AXES 0x07 // use any direction for tap detection 
#define INT_ENABLE 0x40
#define INT_MAP 0x40 //single tap is INT1
#define BW_RATE 0x1A // low power mode, default rate

#define MAX_KNOCKS 32 // maximum length of the pattern
#define MATCH_THRESH 20 // even 10 should be reasonably ok 
#define TIMEOUT 300 // tens of ms before tap processing starts


/* Fuses  H 0xDF  (default)
uint8_t state;
          L 0xE2  (8MHz internal oscillator, no CLKOUT)


  Simple Demo for adxl345 with SPI communication using spi2.c (Need to download new version of spi2)
  
  SPI is 4 sire mode (SCK, MOSI, MISO, CS)
  CS is on PB4, if you change it then do a make clean

  Serial is 19200 baud 8N1
  4p
*/

int taps[MAX_KNOCKS];
int code[MAX_KNOCKS];
uint8_t tap_idx;
uint8_t code_idx;
volatile uint16_t counter;
// TODO make this a state with an enum or something. This is dumb.
uint8_t idle; // is the device asleep/idle
uint8_t recording; // are we in recording or verify mode


/*------------- Function Declarations ----------------*/
void record_tap();
void write_adxl_reg(uint8_t reg,uint8_t data);
void finish_tapping();
void flashred();
void flashgreen();


/*--------------------- Init -------------------------*/
void port_direction_init(void) {

/* define outputs for PORTB */
	DDRB = 0xBF ;									// Note that I had to set MxI, SCK and SS all to outputs.
													// Setting SPE in SPCR doesn't do the output setting it would seem.
/* define outputs for PORTC */
	DDRC = 0 ;
	DDRD = 0;
	PORTB = 0xBF; //10111111
	PORTC = 0; 
	PORTD = 0;

//  Pull up inputs
  PORTA = 0xff;
//	PORTB = 0xff; 
 PORTB = ~(1<<PB2); //don't pull up PB2
	PORTC = 0xff;
	PORTD = 0xff;
// Power register
	PRR = 0xf1;

  // init pb2 for button
  DDRB &= ~(1 << PB2); //set PB2 as input
  PORTB &= ~(1 << PB2); //don't pull up PB2
}

void init_adxl345() {

	write_adxl_reg(0x31,0x08);		// SPI 4 bit, full resolution
	_delay_us(20);					// Probably not needed

	write_adxl_reg(THRESH_TAP_ADDRESS, THRESH_TAP); 				// tap threshold
	write_adxl_reg(DUR_ADDRESS, DUR); 								// tap duration

	write_adxl_reg(WINDOW_ADDRESS, WINDOW); 						// tap window
//	write_adxl_reg(TIME_INACT_ADDRESS, TIME_INACT); 				// time to inactivity 
	write_adxl_reg(TAP_AXES_ADDRESS, TAP_AXES); 					// tap axes 
//	write_adxl_reg(POWER_CTL_ADDRESS,0x03);								// standby
	write_adxl_reg(POWER_CTL_ADDRESS,0x08);								// measure
  write_adxl_reg(INT_ENABLE_ADDRESS, INT_ENABLE);					// tap interrupts
	write_adxl_reg(INT_MAP_ADDRESS, INT_MAP);						// map interrupts to pins 
	write_adxl_reg(FIFO_CTL_ADDRESS,0x80);								// stream mode
	write_adxl_reg(BW_RATE_ADDRESS,BW_RATE);								
}

void init_timer() {
	OCR1A = 0x270F;
	TCCR1A = 0;
	TCCR1B = 0x0a; // divide by 8 for 1 MHz 
	TCCR1C = 0;
	TIMSK1 = 0x02;
	TCNT1 = 0;
}

void main_init() {
	port_direction_init();
	USART0_init(25);							// 19200 baud. no interrupts
	init_spi(3,3);								// mode3 and slow as possible
	init_adxl345();
	EICRA |= 0x0A;
	EIMSK |= 0x03;
	init_timer();
  recording = 1;	
	idle = 1;
	tap_idx = 0;
	code_idx = 0;
 
	finally:
	SREG |= 0x80;								//  using interrupts
}

/*------------------ ADXL stuff -----------------------*/
void write_adxl_reg(uint8_t reg, uint8_t data) {

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



/* ----------------- Interrupt vectors -------------------*/
ISR(INT0_vect) {
//	PORTB |= (1<<PB0);
  //USART0_transmit('0');
}


//single tap interrupt
ISR(INT1_vect) { 
	PORTB |= (1<<PB1); // green led on

	//stop the timer by selecting no clock source
	TCCR1B &= 0xf8; 
	if (idle) {
		idle = 0;
		PORTB &= ~(1<<PB0); // red LED off
	} else {
		//don't record first one because timer value will be meaningless
		if (recording) { 
			record_tap(code, &code_idx);
		} else { 
			record_tap(taps, &tap_idx);
		}
	}

	//clear and start the timer
	TCNT1 = 0;
	counter = 0;
	TCCR1B = 0x0a;
}

ISR(TIMER1_COMPA_vect) {
	counter++; //counts 10s of milliseconds
	if (counter > TIMEOUT && !idle) {
		finish_tapping();
		//go to sleep? 
	}
}

//default empty handler
ISR(__vector_default){
}

/* ----------------- Knock pattern sensing -------------------*/
void clear_knocks(int* arr, int* idx) {
	int i = 0;
	for (; i < MAX_KNOCKS; i++) {
		arr[i] = -1;
	}
	*idx = 0;
}

void replay_code() {
	int i = 0;
	int j;
	PORTB |= (1<<PB0);
	_delay_ms(100); 
	for (; i < code_idx; i++) {
		j = 0;
		PORTB &= ~(1<<PB0); // red LED off
		for (; j < code[i]; j++) {
			_delay_ms(10); // have to give this function a constant
		}
		PORTB |= (1<<PB0);
		_delay_ms(100);
	}
	PORTB &= ~(1<<PB0); 
}

int check() {
	int i = 0;
	if (tap_idx != code_idx) { 
		return 0;
	}
	i = 0;
	uint8_t correct = 1;
	for (; i < tap_idx; i++) {
		if ((taps[i] > code[i] + MATCH_THRESH) || (taps[i] < code[i] - MATCH_THRESH)) {
			correct = 0;	
			break;
		}
	}
	return correct;
}


void usart_print_arr(int* arr, uint8_t* idx) {
	int i = 0;
	for (; i < *idx; i++) {
		int16_t casted = (int16_t) arr[i];
		USART0_transmit_int16(casted);
		USART0_transmit(' ');
	}
	USART0_transmit('\n');
}

void finish_tapping() {
	if (recording) {
		replay_code();
		recording = 0;
	} else {
		int correct = check();	
		if (!correct) {
			flashred();
		} else {
			flashgreen();
		}
		clear_knocks(taps, &tap_idx); // clear the attempt
	}
	idle = 1;
}


void record_tap(int* tap_arr, int* arr_idx) {
	tap_arr[*arr_idx] = counter;
	(*arr_idx)++; 
}

/* ----------------- Helper functions -------------------*/

void flashred() {
	PORTB &= ~(1<<PB0);
	_delay_ms(500);
	PORTB |= (1<<PB0);
	_delay_ms(500);
	PORTB &= ~(1<<PB0);
}

void flashgreen() {
	PORTB &= ~(1<<PB1);
	_delay_ms(500);
	PORTB |= (1<<PB1);
	_delay_ms(500);
	PORTB &= ~(1<<PB1);
}

void flash_fast_pattern() {
	PORTB &= ~(1<<PB0);
	_delay_ms(100);
	PORTB |= (1<<PB0);
	_delay_ms(100);
	PORTB &= ~(1<<PB0);

	PORTB &= ~(1<<PB1);
	_delay_ms(100);
	PORTB |= (1<<PB1);
	_delay_ms(100);
	PORTB &= ~(1<<PB1);
	
	PORTB &= ~(1<<PB0);
	_delay_ms(100);
	PORTB |= (1<<PB0);
	_delay_ms(100);
	PORTB &= ~(1<<PB0);
}


void changeState() {
	if (recording) { //switch to verify mode
		flashred();
	} else {//switch to recording mode
		flash_fast_pattern();	
		clear_knocks(code, &code_idx); // clear the old code
	}
	recording = !recording;
}

/*----------------------- MAIN ----------------------*/
int main(void) {
	uint8_t readings[16] ;
	uint8_t i ;
	uint16_t x;
	uint16_t y;
	uint16_t z;
	uint16_t xsign;
	uint16_t ysign;
	uint16_t zsign;
	
	main_init();
  uint8_t int_source = 0;

	while (1) {

		int_source = read_adxl_reg(INT_SOURCE_ADDRESS); // necessary for generating interrupts
		_delay_ms(10);
		if (PINB & (1<<PB2)) {
			changeState();
		}
		PORTB &= ~(1<<PB1);
		if (code_idx > 0)
			PORTB &= ~(1<<PB0); // red LED off if we have a password
		set_sleep_mode(0x1);  // ADC noise reduction - best power savings possible wiht interrupt still working
		sleep_mode();
	}
}



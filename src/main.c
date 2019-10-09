/*
 * main.c
 *
 *  Created on: 9. 7. 2015
 *      Author: LSL<ludek.slouf@gmail.com>
 *
 *      Porty:
 *
 *      B0, B1, B3 : IN -slave adresa
 *      B2 : OUT ventilator
 *      B4 : IN-OUT termistor
 *      B5, B7: SDA, SCL - i2c slave
 *      D0 - D6: OUT pwm
 *
 *      TODO: provoz bez kontroleru:
 *      po zapnuti napajeni stoupa jas po dobu jedne hodiny do maxima
 *      - nastaveneho propojkou
 *      - delka sviceni je  urcena propojkou (10,11,12 hodin)
 *      - po uplynuti doby klesa jas na nulu po dobu jedne hodiny
 *      - dalsi hodinu funguje nocni sviceni
 *      - externi spinaci hodiny je potreba nastavit tak, aby se vyply
 *      - po ukonceni celeho cyklu
 *
 *      - popojky A0 A1 A2
 *        --0 12 hod
 *        --1 10 hod max
 *        00- 100%
 *        01- 75%
 *        10- 50%
 *        11- 25%
 */

//TODO:  if overheat lower brightness
//VERSION = cislo hlavni verze
#define VERSION          200
#define VERSION_SUB      206

#define F_CPU         16000000L
// includes
#include <stddef.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/pgmspace.h>
#include <util/atomic.h>
#include <avr/wdt.h>
#include "usiTwiSlave.h"

#include "twi_registry.h"

#ifdef DEBUG
#include "dbg_putchar.h"
#endif

#define MASTER    0xFF
#define DEMO      0xde
#define EFFECT    0xef

#define LOW_BYTE(x)    		(x & 0xff)
#define HIGH_BYTE(x)       	((x >> 8) & 0xff)

#define BYTELOW(v)   (*(((unsigned char *) (&v))))
#define BYTEHIGH(v)  (*((unsigned char *) (&v)+1))

#define	bis(ADDRESS,BIT)	(ADDRESS & (1<<BIT))
#define sbi(port,bit)       (port) |= (1 << (bit))
#define cbi(port,bit)       (port) &= ~(1 << (bit))

#define TWIADDR 0b00100000
//#define TWIADDR 0b00110000
uint8_t twiaddr = TWIADDR;

/* Thermometer */
#define THERM_ERROR_ERR 0x00
#define THERM_ERROR_OK  0x01
#define THERM_TEMP_ERR  0x80

/* Thermometer Connections (At your choice) */
#define THERM_PORT 	PORTB
#define THERM_DDR 	DDRB
#define THERM_PIN 	PINB
#define THERM_DQ 	PB4
#define TEMPERATURE_DELAY    2000
#define TEMPERATURE_TRESHOLD 28       //fan on
#define TEMPERATURE_TRESHOLD_STOP 40  //fan off, when led is off
#define TEMPERATURE_MAX      40  //fan max
#define FAN_MIN              80  //minimum pwm fan 30%
#define FAN_MAX              255 //minimum pwm fan 30%

/* Utils */
#define THERM_INPUT_MODE() 		THERM_DDR&=~(1<<THERM_DQ)
#define THERM_OUTPUT_MODE()		THERM_DDR|=(1<<THERM_DQ)
#define THERM_LOW() 				THERM_PORT&=~(1<<THERM_DQ)
#define THERM_HIGH() 			THERM_PORT|=(1<<THERM_DQ)

#define THERM_CMD_CONVERTTEMP 0x44
#define THERM_CMD_RSCRATCHPAD 0xbe
#define THERM_CMD_WSCRATCHPAD 0x4e
#define THERM_CMD_CPYSCRATCHPAD 0x48
#define THERM_CMD_RECEEPROM 0xb8
#define THERM_CMD_RPWRSUPPLY 0xb4
#define THERM_CMD_SEARCHROM 0xf0
#define THERM_CMD_READROM 0x33
#define THERM_CMD_MATCHROM 0x55
#define THERM_CMD_SKIPROM 0xcc
#define THERM_CMD_ALARMSEARCH 0xec

// Scratchpad locations
#define TEMP_LSB        0
#define TEMP_MSB        1
#define HIGH_ALARM_TEMP 2
#define LOW_ALARM_TEMP  3
#define CONFIGURATION   4
#define INTERNAL_BYTE   5
#define COUNT_REMAIN    6
#define COUNT_PER_C     7
#define SCRATCHPAD_CRC  8

// rizeni chodu
unsigned long tempTicks = 0;
unsigned long pwmTicks = 0;
unsigned long dayTime = 0;
unsigned long setDayTime = 0;
unsigned long nightTime = 0;
unsigned long setNightTime = 0;
unsigned long timeTicks = 0;
unsigned long milis_time = 0;
unsigned long i_timeTicks = 0;

#ifdef DEBUG
#define SEC  1000L
#else
#define SEC  1000L
#endif

#define HOUR  3600L
#define DAY  (24L*HOUR)

#ifdef DEBUG
#define RAMPUP  (256L)
#define RAMPDOWN  (256L)
#else
#define RAMPUP  (4L*HOUR)
#define RAMPDOWN  (4L*HOUR)
#endif

const uint8_t version PROGMEM = VERSION;
const uint8_t version_sub PROGMEM = VERSION_SUB;

const uint8_t pwmtable1[170] PROGMEM = { 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2,
		2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 3,
		3, 4, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6, 6, 7, 7, 7, 7,
		7, 8, 8, 8, 9, 9, 9, 9, 10, 10, 10, 11, 11, 11, 12, 12, 13, 13, 13, 14,
		14, 15, 15, 16, 16, 17, 17, 18, 19, 19, 20, 21, 21, 22, 23, 23, 24, 25,
		26, 27, 27, 28, 29, 30, 31, 32, 33, 35, 36, 37, 38, 39, 41, 42, 43, 45,
		46, 48, 49, 51, 53, 54, 56, 58, 60, 62, 64, 66, 68, 71, 73, 75, 78, 80,
		83, 86, 89, 91, 95, 98, 101, 104, 108, 111, 115, 119, 123, 127, 131,
		135, 140, 144, 149, 154, 159, 164, 170, 175, 181, 187, 193, 200, 206,
		213, 220, 227, 235, 242, 250 };

const uint16_t pwmtable2[86] PROGMEM = { 259, 267, 276, 285, 295, 304, 314, 325,
		336, 347, 358, 370, 382, 395, 408, 421, 435, 450, 464, 480, 496, 512,
		529, 546, 564, 583, 602, 622, 643, 664, 686, 708, 732, 756, 781, 807,
		833, 861, 889, 919, 949, 980, 1013, 1046, 1081, 1116, 1153, 1191, 1231,
		1271, 1313, 1357, 1402, 1448, 1496, 1545, 1596, 1649, 1703, 1759, 1818,
		1878, 1940, 2004, 2070, 2138, 2209, 2282, 2357, 2435, 2515, 2598, 2684,
		2773, 2864, 2959, 3057, 3158, 3262, 3370, 3481, 3596, 3715, 3837, 3964,
		4000 };
	

//teplomer
int8_t therm_ok = 0;
uint8_t scratchpad[9];
int8_t rawTemperature = 25;
uint16_t crc = 0xFFFF;

// pwm
int16_t val, nval = 0;
#define PWM_FREQ      240   //480
#define LED_PORT      PORTD              // Port for PWM
#define LED_DIR       DDRD               // Register for PWM
#define PWM_CHANNELS  7                  // count PWM channels
#define PWMNIGHT      19  //5% max hodnoty
#define MAXPWM        256 //63
#define PWM_BITS   12
#define LEDS       7

#define NIGHT_LED1  0
#define NIGHT_LED2  8

volatile uint8_t loop = 0;
volatile uint8_t bitmask = 0;

/* poradi a casovani bitu:
 *  - mame 10x preruseni na 4096 bitu
 *  - cely cyklus trva neco pres  4 mSec, takze mame frekvenci cca 240Hz
 * 1 tick = 1/FCPU * 16 * 1000000 uS =  1uS
 * 0, 1, 2, 3, 4, 5, 1/4 7,1/2 6,1/2 7,1/2 6,1/4 7, 1/2 8,1/2 9,1/2 8,1/4 11,1/2 10,1/2 11, 1/2 10,1/4 11,1/2 9
 *  0 bit: 1 uS
 *  1 bit: 2 uS
 *  2 bit: 4 uS
 *  3 bit: 8 uS
 *  4 bit: 16 uS
 *  5 bit: 32 uS
 *  1/2 6bitu: 32us (6 bit: 64uS)
 *  1/4 7 bitu 32uS
 *  atd ....
 *  nejdelsi okno mezi prerusenimi je cca 1000 uS (1/2 11 bitu
 *  ostatni okna jsou cca 100, 200, 500 uS
 *
 *  pro reset teplomeru potrebujeme okno o delce cca 520uS
 *  takze cekame na nastaveni OCR1B na 45045, pak mame cca 1000uS na reset teplomeru
 *
 *  vyznamnou chybu muzou zpusobit dalsi preruseni
 *  	-  funkce milis(), ktera trva ...4,5 uS, takze by neme byt problem = max chyba je cca 3.5%
 *  			a pouze u vyssich bitu
 *  	-  TODO: overit dobu trvani preruseni pri I2C komunikaci
 */

#if (PWM_FREQ == 480)
#define LOOP_COUNT 9
#define MAX_LOOP   LOOP_COUNT-1
const uint8_t tbl_loop_bitmask[LOOP_COUNT] = { 8, 9, 8, 11, 10, 11, 10, 11, 9 };
const uint16_t tbl_loop_len[LOOP_COUNT] = {3069,5117,6141,10237,14333,22525,26621,30717,32765};
#define WAIT_0 8
#define WAIT_1 16
#define WAIT_2 25
#define WAIT_3 57
#define WAIT_4 121
#define WAIT_5 249
#define WAIT_6a 249
#define WAIT_6b 249
#define WAIT_7a 249
#define WAIT_7b 505
#define WAIT_7c 249
#else
#define LOOP_COUNT 9
#define MAX_LOOP   LOOP_COUNT-1
const uint8_t tbl_loop_bitmask[LOOP_COUNT] = { 8, 9, 8, 11, 10, 11, 10, 11, 9 };
const uint16_t tbl_loop_len[LOOP_COUNT] = { 6133, 10229, 12277, 20469, 28661, 45045, 53237, 61429, 65523 };
#define WAIT_0   16
#define WAIT_1   32
#define WAIT_2   57
#define WAIT_3   121
#define WAIT_4   249
#define WAIT_5   505
#define WAIT_6a  505
#define WAIT_6b  505
#define WAIT_7a  505
#define WAIT_7b  1017
#define WAIT_7c  505
#endif

uint16_t incLedValues[LEDS + 1] = { 0 };
uint16_t ledValues[LEDS + 1] = { 0 };
uint16_t prevLedValues[LEDS + 1] = { 0 };
uint16_t actLedValues[LEDS] = { 0 };

uint16_t *p_ledValues = ledValues;
uint16_t *p_prevLedValues = prevLedValues;
uint16_t *p_actLedValues = actLedValues;
uint16_t *p_incLedValues = incLedValues;

int8_t isteps = 0;

uint8_t _data[PWM_BITS] = { 0 };      //double buffer for port values
uint8_t _data_buff[PWM_BITS] = { 0 };
uint8_t *_d;
uint8_t *_d_b;
uint8_t interpolationStart = 0; //flag

volatile unsigned char newData = 0; //flag
volatile uint8_t pwm_status = 0;
volatile uint8_t inc_pwm_data = 1;
volatile uint8_t effect = 0;

int16_t tmp;
int16_t tmp2;
int delta;

//interpolace s mezemi min a max, 8bit
static uint8_t map_minmax(uint8_t x, uint8_t in_min, uint8_t in_max,
		uint8_t out_min, uint8_t out_max) {
	int16_t ret = (x - in_min) * (out_max - out_min) / (in_max - in_min)
			+ out_min;
	return (uint8_t) (ret > out_max ? out_max : ret < out_min ? out_min : ret);
}

//linear interpolation
static long map(long x, long in_min, long in_max, long out_min, long out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
/*
 * PWM / bit angle modulation
 */
// Timer1 handler.
ISR(TIMER1_COMPB_vect) {
	uint8_t r1, r2, r3;
	bitmask = tbl_loop_bitmask[loop];

	if (loop == 0) {
		r1 = _d[0];
		r2 = _d[1];
		r3 = _d[2];

		__builtin_avr_delay_cycles(4L); //vyrovnani posledniho bitu

		//bit 0
		LED_PORT = r1; //45 cyklu
		__builtin_avr_delay_cycles(WAIT_0);

		//bit 1
		LED_PORT = r2;
		__builtin_avr_delay_cycles(WAIT_1);

		//bit 2
		LED_PORT = r3;
		__builtin_avr_delay_cycles(WAIT_2); //7 taktu na zpracovani

		//bit 3
		LED_PORT = _d[3];
		__builtin_avr_delay_cycles(WAIT_3);

		//bit 4
		LED_PORT = _d[4];
		__builtin_avr_delay_cycles(WAIT_4);

		//bit 5
		LED_PORT = _d[5];
		__builtin_avr_delay_cycles(WAIT_5);

		//bit 7a
		LED_PORT = _d[7];
		__builtin_avr_delay_cycles(WAIT_7a);

		//bit 6a
		LED_PORT = _d[6];
		__builtin_avr_delay_cycles(WAIT_6a);

		//bit 7b
		LED_PORT = _d[7];
		__builtin_avr_delay_cycles(WAIT_7b);

		//bit 6b
		LED_PORT = _d[6];
		__builtin_avr_delay_cycles(WAIT_6b);

		//bit 7c
		LED_PORT = _d[7];
		__builtin_avr_delay_cycles(WAIT_7c);

		//bit 8a
		LED_PORT = _d[8];
		OCR1B = tbl_loop_len[loop];
	} else {
		LED_PORT = _d[bitmask];
		OCR1B = tbl_loop_len[loop];
	}

	//loop++;
	if (++loop > MAX_LOOP) {
		loop = 0;
		//switch buffers
		if (newData) {
			uint8_t *tmp;
			tmp = _d;
			_d = _d_b;
			_d_b = tmp;
			newData = 0;
		}
	} else {
		//vyvazeni vetve
		__builtin_avr_delay_cycles(28L);
	}

}

static void _pwm_init(void) {
	// Hardware init.
	LED_DIR = 0xFF; //led pins output
	LED_PORT = 0x00;   //off
	OCR1A = tbl_loop_len[MAX_LOOP] + 1;
	TCCR1B = 1 << WGM12 | 1 << CS10; // CTC-mode, F_CPU / 1
	TIMSK |= 1 << OCIE1B;  		  //start timer
}

void pwm_update(void) {
	//clear
	memset(_d_b, 0, 12);
	//rearrange values to ports
	for (int i = 0; i < PWM_BITS; i++) {
		for (int j = 0; j < LEDS; j++) {
			_d_b[(PWM_BITS - 1) - i] = (_d_b[(PWM_BITS - 1) - i] << 1)
					| (((actLedValues[j]) >> ((PWM_BITS - 1) - i)) & 0x01);
		}
	}

	//wait for prev. data process
	while (newData) {
		;
	};
	//set new data flag
	newData = 1;
}

/*
 * Rutiny pro teplomer
 *
 */

uint8_t ds18b20crc8(uint8_t *data, uint8_t length) {
	//Generate 8bit CRC for given data (Maxim/Dallas)

	uint8_t i = 0;
	uint8_t j = 0;
	uint8_t mix = 0;
	uint8_t crc = 0;
	uint8_t byte = 0;

	for (i = 0; i < length; i++) {
		byte = data[i];

		for (j = 0; j < 8; j++) {
			mix = (crc ^ byte) & 0x01;
			crc >>= 1;
			if (mix)
				crc ^= 0x8C;
			byte >>= 1;
		}
	}
	return crc;
}

static uint8_t therm_reset() {

	uint8_t i;
	//Pull line low and wait for 480uS
	//ATOMIC_BLOCK(ATOMIC_FORCEON) {
	//cekani na casove okno
	// 480us = cca 7680 cyklu, ktere by mely byt k dispozici
	while (OCR1B < 45045) {
		;
	};
	//ATOMIC_BLOCK(ATOMIC_FORCEON) {
	THERM_LOW();
	THERM_OUTPUT_MODE();
	_delay_us(480);
	THERM_INPUT_MODE();
	_delay_us(40);
	i = (THERM_PIN & (1 << THERM_DQ));
	//}
	_delay_us(480);
	//Return the value read from the presence pulse (0=OK, 1=WRONG)
	return i;
}

static void therm_write_bit(uint8_t bit) {
	//Pull line low for 1uS
	while (OCR1B < 6133) {
		;
	};
	THERM_LOW();
	THERM_OUTPUT_MODE();
	_delay_us(1);
	//If we want to write 1, release the line (if not will keep low)
	if (bit)
		THERM_INPUT_MODE();
	//Wait for 60uS and release the line
	_delay_us(60);
	THERM_INPUT_MODE();
}

static uint8_t therm_read_bit(void) {

	uint8_t bit = 0;
	//Pull line low for 1uS
	while (OCR1B < 6133) {
		;
	};
	THERM_LOW();
	THERM_OUTPUT_MODE();
	_delay_us(1);
	//Release line and wait for 14uS
	THERM_INPUT_MODE();
	_delay_us(10);
	//Read line value
	if (THERM_PIN & (1 << THERM_DQ))
		bit = 1;
	//Wait for 45uS to end and return read value
	_delay_us(45);
	return bit;
}

static uint8_t therm_read_byte(void) {
	uint8_t i = 8, n = 0;
	while (i--) {
		n >>= 1; //Shift one position right and store read value
		n |= (therm_read_bit() << 7);
	}
	return n;
}

static void therm_write_byte(uint8_t byte) {

	uint8_t i = 8;
	while (i--) {
		//Write actual bit and shift one position right to make the next bit ready
		therm_write_bit(byte & 1);
		byte >>= 1;
	}
}

/*
 * Rutiny pro i2c
 *
 */

void i2cWriteToRegister(uint8_t reg, uint8_t value) {
	switch (reg) {
	case reg_MASTER:
		pwm_status = value;
		break;
		/*
		 case reg_CRC_H:
		 BYTEHIGH(crc) = value;
		 break;
		 case reg_CRC_L:
		 BYTELOW(crc) = value;
		 break;
		 */
	case reg_LED_L_0 ... reg_CRC_H:
		(*((uint8_t *) (p_incLedValues) + reg)) = value;
		break;
	case reg_DATA_OK:
		inc_pwm_data = value;
		break;
	case reg_EFFECT:
		effect = value;
		break;
	}
}

uint8_t i2cReadFromRegister(uint8_t reg) {
	uint8_t ret = 0x00;
	switch (reg) {
	case reg_LED_L_0 ... reg_LED_H_6:
		ret = (*((uint8_t *) (p_actLedValues) + reg));
		break;
	case reg_MASTER:
		ret = pwm_status;
		break;
	case reg_THERM_STATUS:  //temperature status
		ret = therm_ok;
		break;
	case reg_RAW_THERM:
		ret = rawTemperature;
		break;
	case reg_VERSION_MAIN:
		ret = pgm_read_byte(&version);
		break;
	case reg_VERSION_SUB:
		ret = pgm_read_byte(&version_sub);
		break;
	}
	return ret;
}

/*
 * Milis()
 */

#define PRESCALER  8
#define clockCyclesToMicroseconds(a) ( ((a) * 1000L) / (F_CPU / 1000L) )
#define MICROSECONDS_PER_TIMER0_OVERFLOW (clockCyclesToMicroseconds(PRESCALER * 256))
#define MILLIS_INC (MICROSECONDS_PER_TIMER0_OVERFLOW / 1000)
// the fractional number of milliseconds per timer0 overflow. we shift right
// by three to fit these numbers into a byte. (for the clock speeds we care
// about - 8 and 16 MHz - this doesn't lose precision.)
#define FRACT_INC ((MICROSECONDS_PER_TIMER0_OVERFLOW % 1000) >> 3)
#define FRACT_MAX (1000 >> 3)

volatile unsigned long timer0_overflow_count = 0;
volatile unsigned long timer0_millis = 0;
static unsigned char timer0_fract = 0;

ISR(TIMER0_OVF_vect) {
	unsigned long m = timer0_millis;
	unsigned char f = timer0_fract;

	m += MILLIS_INC;
	f += FRACT_INC;
	if (f >= FRACT_MAX) {
		f -= FRACT_MAX;
		m += 1;
	}

	timer0_fract = f;
	timer0_millis = m;
	timer0_overflow_count++;
}

unsigned long millis() {
	unsigned long m;
	uint8_t oldSREG = SREG;

	cli();
	m = timer0_millis;
	SREG = oldSREG;
	return m;
}

/*
 * Hlavni smycka
 *
 */

static void set_fan(uint8_t pwm) {
	ATOMIC_BLOCK(ATOMIC_FORCEON)
	{
		if (pwm == 0) {
			TCCR0A &= ~(1 << COM0A1);
		} else {
			TCCR0A |= (1 << COM0A1);
		}
		OCR0A = pwm;  //port B2 je PWM fan
	}
}

static uint16_t crc16_update(uint16_t crc, uint8_t a) {
	int i;

	crc ^= a;
	for (i = 0; i < 8; ++i) {
		if (crc & 1)
			crc = (crc >> 1) ^ 0xA001;
		else
			crc = (crc >> 1);
	}

	return crc;
}

void led_flash() {
	uint16_t l0, l1, l2;
	l0 = actLedValues[0];
	l1 = actLedValues[1];
	l2 = actLedValues[2];
	actLedValues[0] = 4095;
	actLedValues[1] = 4095;
	actLedValues[2] = 4095;
	pwm_update();
	_delay_ms(2);
	actLedValues[0] = l0;
	actLedValues[1] = l1;
	actLedValues[2] = l2;
	pwm_update();
	return;
}

void led_storm() {
	return;
}

uint8_t checkActLedVal() {
	uint8_t lv = 0;
	for (uint8_t i = 0; i < LEDS; i++) {
		if (actLedValues[i] != 0) {
			lv = 1;
			break;
		}
	}
	return lv;
}

/**************************************
 * Main routine
 *
 *************************************/

int main(void) {

#ifdef DEBUG
	dbg_tx_init();
#endif

	_d = _data;
	_d_b = _data_buff;

	//disable irq
	cli();

	/*
	 * Watchdog enable 4sec
	 */
	wdt_reset();
	wdt_enable(WDTO_4S);

	//input a pullup, PB0=A0. PB1=A2, PB3=A3     na B6 je spinac ON/OFF, na B4 je DS1820
#ifdef DEBUG
	PORTB |= (1 << PB1) | (1 << PB3) | (1 << PB6); //| (1 << PB0) ; PB0 je debug output
	DDRB &= ~(1 << PB1) & ~(1 << PB3) & ~(1 << PB6);//& ~(1 << PB0) ;
#else
	PORTB |= (1 << PB1) | (1 << PB3) | (1 << PB6) | (1 << PB0);
	DDRB &= ~(1 << PB1) & ~(1 << PB3) & ~(1 << PB6) & ~(1 << PB0);
#endif

	/*
	 * Precteni a nastaveni TWI adresy
	 * podle propojek na portech PB0, PB1, PB3
	 */
#ifdef DEBUG   //port B0 je debug output
	if (!(PINB & (1 << PB3))) {twiaddr |= (1 << 3);}
	if (!(PINB & (1 << PB1))) {twiaddr |= (1 << 2);}
	twiaddr |= (1 << 1);
#else
	if (!(PINB & (1 << PB3))) {
		twiaddr |= (1 << 3);
	}
	if (!(PINB & (1 << PB1))) {
		twiaddr |= (1 << 2);
	}
	if (!(PINB & (1 << PB0))) {
		twiaddr |= (1 << 1);
	}
#endif

	usiTwiSlaveInit(twiaddr, i2cReadFromRegister, i2cWriteToRegister);

	/* Inicializace timeru milis()
	 * Inicialzace hw PWM ventilatoru
	 * ventilator je pripojeny na PWM port B2
	 */
	DDRB |= (1 << PB2);  //B2 na output

	TIFR |= (1 << TOV0);
	TIMSK |= (1 << TOIE0);

#if (PRESCALER == 64L)
	TCCR0B |= (1 << CS01) | (1 << CS00); // prescale 64
#else
	TCCR0B |= (1 << CS01); // prescale 8
#endif
	//fast PWM
	TCCR0A |= (1 << COM0A1) | (1 << WGM00) | (1 << WGM01);

	//start pwm
	OCR0A = 0;

	_pwm_init();

	sei();

	/*
	 * test ventilatoru
	 *
	 */
	set_fan(255);
	_delay_ms(2000);
	set_fan(0);
	/*
	 * Inicializace teplomeru
	 * start konverze
	 */

	if (therm_reset()) {
		therm_ok = 0;
		set_fan(FAN_MAX);
	} else {
		therm_ok = 1;
		therm_reset();
		therm_write_byte(THERM_CMD_SKIPROM);
		therm_write_byte(THERM_CMD_WSCRATCHPAD);
		therm_write_byte(0); //Th register
		therm_write_byte(0); //TL register
		therm_write_byte(0b00011111); //conf register, 9bit resolution

		therm_reset();
		therm_write_byte(THERM_CMD_SKIPROM);
		therm_write_byte(THERM_CMD_CONVERTTEMP);
		while (!therm_read_bit())
			;
	}

	/*
	 * Precteni konfiguracnich prepinacu, nastaveni doby sviceni
	 * pro autonomni provoz
	 * pocitame cas od zapnuti
	 * dle nastavenych propojek
	 */
#ifdef DEBUG
	setDayTime = (520); setNightTime = 0;
#else
	switch ((twiaddr & 0b00001110) >> 1) {
	case 1:
		setDayTime = (10L * HOUR);
		setNightTime = 0;
		break;
	case 2:
		setDayTime = (11L * HOUR);
		setNightTime = 0;
		break;
	case 3:
		setDayTime = (12L * HOUR);
		setNightTime = 0;
		break;
	case 4:
		setDayTime = (10L * HOUR);
		setNightTime = 1 * HOUR;
		break;
	case 5:
		setDayTime = (11L * HOUR);
		setNightTime = 1 * HOUR;
		break;
	case 6:
		setDayTime = (12L * HOUR);
		setNightTime = 1 * HOUR;
		break;
	default:
		setDayTime = (10L * HOUR);
		setNightTime = 0;
	}

#endif

#define MASTER_TIMEOUT  2 //sec
	//cekej  na pwm_status from master
	uint8_t wait_tmp = 0;

	//testujeme stav prepinace.
	//pokud je sepnuto, pak jede demo provoz
	//pokud je rozepnuto, oak cekame na master status
	//pokud master status neprijde, jede autonomni provoz
	//demo provoz lze pustit i z mastera zaslanim hodnoty 0xde
	//do pwm_status
	if (!(PINB & (1 << PB6))) {
		//sepnuto, demo
		pwm_status = DEMO;
	} else {
		//cekej na master status
		while ((pwm_status != MASTER) || (pwm_status != DEMO)) {
			wdt_reset();
			_delay_ms(1000);
			if (++wait_tmp > MASTER_TIMEOUT)
				break;
		}
	}
	while (1) {
		wdt_reset();

		/*
		 * Mereni teploty
		 */
		if (therm_ok && ((millis() - tempTicks) >= TEMPERATURE_DELAY)) { //precteni teploty a start nove konverze
			therm_reset();
			therm_write_byte(THERM_CMD_SKIPROM);
			therm_write_byte(THERM_CMD_RSCRATCHPAD);

			uint8_t i = 0;
			do {
				scratchpad[i] = therm_read_byte();
			} while (++i < 9);

			if (ds18b20crc8(scratchpad, 8) == scratchpad[8]) {
				rawTemperature = scratchpad[0] >> 4;
				rawTemperature |= (scratchpad[1] & 0x7) << 4;
			}

			/*
			 * ventilator dle teploty, pokud je ledval > 0
			 */

			if ((checkActLedVal() == 0)
					&& (rawTemperature < TEMPERATURE_TRESHOLD_STOP)) {
				set_fan(0);
			} else {
				if (rawTemperature != THERM_TEMP_ERR) {
					if (rawTemperature < TEMPERATURE_TRESHOLD) {
						set_fan(0);
					} else {
						uint8_t fanVal = map_minmax(rawTemperature,
								TEMPERATURE_TRESHOLD, TEMPERATURE_MAX, FAN_MIN,
								FAN_MAX);
						if (fanVal < 150)
							set_fan(FAN_MAX);
						_delay_ms(500);
						set_fan(fanVal);
					}
				}
			}
			/*
			 * start dalsiho mereni
			 */
			therm_reset();
			therm_write_byte(THERM_CMD_SKIPROM);
			therm_write_byte(THERM_CMD_CONVERTTEMP);
			tempTicks = millis();
		} else {
			//teplomer nefunguje, ale chladit potrebujeme
			if (checkActLedVal() == 0) {
				set_fan(0);
			} else {
				set_fan(FAN_MAX);
			}
		}

		/*
		 *  Hlavni rizeni
		 */

		uint16_t xcrc = 0xffff;

		milis_time = millis();

		switch (pwm_status) {
		case EFFECT:
			switch (effect) {
			case FLASH:
				led_flash();
				break;
			case STORM:
				led_storm();
				break;
			}
			effect = NO_EFFECT;
		case MASTER:
			if (inc_pwm_data == 0) {  //dostali jsme data, kontrola CRC
				for (uint8_t i = 0; i < 8; i++) {
					xcrc = crc16_update(xcrc, LOW_BYTE(p_incLedValues[i]));
					xcrc = crc16_update(xcrc, HIGH_BYTE(p_incLedValues[i]));
				}

				if (xcrc == 0) {
					uint16_t *tmpptr = p_prevLedValues;
					p_prevLedValues = p_ledValues;
					p_ledValues = p_incLedValues;
					p_incLedValues = tmpptr;
					//priznak startu interpolace
					interpolationStart = 1;
				}
				inc_pwm_data = 1;
			}

			break;		
		case DEMO:
			//demo provoz
			//postupne  zapina kazdou led na hodnotu 0 .. 250 .. 0
			for (uint8_t i = 0; i < LEDS; i++) {
				for (uint8_t v = 0; v < 170; v++) {
					actLedValues[i] = pgm_read_byte(&pwmtable1[v]);
					_delay_ms(2);
					pwm_update();
					wdt_reset();
				}
				for (uint8_t v = 169; v > 0; v--) {
					actLedValues[i] = pgm_read_byte(&pwmtable1[v]);
					_delay_ms(2);
					pwm_update();
					wdt_reset();
				}
				actLedValues[i] = 0;
			}

			//flash
			led_flash();
			_delay_ms(100);
			led_flash();
			_delay_ms(80);
			led_flash();
			//TODO: storm
			led_storm();
			break;
		}

		// interpolace hodnot
#define ISTEPS 100       //pocet kroku
#define ISTEPTIMEOUT 10  //ms mezi kroky, celkovy cas prechodu ms = ISTEPS * ISTEPTIMEOUT
		if (interpolationStart == 1) {
			if ((milis_time - i_timeTicks) > ISTEPTIMEOUT) {
				i_timeTicks = milis_time;
				for (uint8_t x = 0; x < LEDS; x++) {
					actLedValues[x] = map(isteps, 0, ISTEPS, p_prevLedValues[x],
							p_ledValues[x]);
				}
				isteps++;
				if (isteps > ISTEPS) {
					interpolationStart = 0;
					isteps = 0;
				}
				pwm_update();
			}
		}
	}
}

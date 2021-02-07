/*
 * main.c
 *  Created on: 9. 7. 2015
 *      Author: LSL<ludek.slouf@gmail.com>
 *
 *      Porty:
 *      B0, B1, B3 : IN -slave adresa
 *      B2 : OUT ventilator
 *      B4 : IN-OUT termistor
 *      B5, B7: SDA, SCL - i2c slave
 * 		B6 : Pocatecni TWI adresa
 *      D0 - D6: OUT pwm
 * 
 */

 

#define VERSION      200
#define VERSION_SUB    6

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

#define DEMOLEDVALUE 100

#define MASTER    0xFF
#define DEMO      0xde

#define LOW_BYTE(x)    		(x & 0xff)
#define HIGH_BYTE(x)       	((x >> 8) & 0xff)

#define BYTELOW(v)   (*(((unsigned char *) (&v))))
#define BYTEHIGH(v)  (*((unsigned char *) (&v)+1))

#define TWIADDR1 0b00100000
#define TWIADDR2 0b00110000
uint8_t twiaddr = TWIADDR1;

/* Thermometer */
#define THERM_ERROR_ERR 0x00
#define THERM_ERROR_OK  0x01
#define THERM_TEMP_ERR  0x80

/* Thermometer Connections  */
#define THERM_PORT 	PORTB
#define THERM_DDR 	DDRB
#define THERM_PIN 	PINB
#define THERM_DQ 	PB4
#define TEMPERATURE_DELAY    2000
#define TEMPERATURE_TRESHOLD 28       //fan on
#define TEMPERATURE_TRESHOLD_STOP 40  //fan off, when led is off
#define TEMPERATURE_MAX      40  //fan max
#define FAN_MIN              80  //minimum pwm fan 30%
#define FAN_MAX              255 //max pwm fan 100%
#define TEMPERATURE_OVERHEAT 45 

/* Utils */
#define THERM_INPUT_MODE() 		THERM_DDR&=~(1<<THERM_DQ)
#define THERM_OUTPUT_MODE()		THERM_DDR|=(1<<THERM_DQ)
#define THERM_LOW() 			THERM_PORT&=~(1<<THERM_DQ)
#define THERM_HIGH() 			THERM_PORT|=(1<<THERM_DQ)

#define THERM_CMD_CONVERTTEMP 	0x44
#define THERM_CMD_RSCRATCHPAD 	0xbe
#define THERM_CMD_WSCRATCHPAD 	0x4e
#define THERM_CMD_CPYSCRATCHPAD 0x48
#define THERM_CMD_RECEEPROM 	0xb8
#define THERM_CMD_RPWRSUPPLY 	0xb4
#define THERM_CMD_SEARCHROM 	0xf0
#define THERM_CMD_READROM 		0x33
#define THERM_CMD_MATCHROM 		0x55
#define THERM_CMD_SKIPROM 		0xcc
#define THERM_CMD_ALARMSEARCH 	0xec

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

	
//teplomer
int8_t therm_ok = 0;
uint8_t scratchpad[9];
int8_t rawTemperature = 25;
uint16_t crc = 0xFFFF;

// pwm
int16_t val, nval = 0;

//	#define PWM_FREQ      480    //test

#define PWM_FREQ      240    

#define LED_PORT      PORTD  // Port for PWM
#define LED_DIR       DDRD   // Register for PWM
#define PWM_BITS      12
#define PWM_CHANNELS  7
#define MOONLED       5

//sw resistor - set max current for channel
// Rs resitor = 0.1 Ohm
// uv, rb, white, red, green, yellow, blue
// Iout = (0.1 * D) /  Rs

#define RAMPUPTIME 180
#define MAXLIGHTTIME 540
#define RAMPDOWNTIME 720
#define MOONTIME 1080

const uint16_t ledValues[] PROGMEM = {4000,600,600,100,400,4000,4000,10 };
const uint16_t dayTimes[] PROGMEM = { RAMPUPTIME,MAXLIGHTTIME,RAMPDOWNTIME,MOONTIME}; //ramp up, max, ramp down, moon
const uint8_t sw_resistor[] PROGMEM = {100,70,100,70,100,100,35};

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


int16_t incLedValues[PWM_CHANNELS + 1] = { 0 };
int16_t actLedValues[PWM_CHANNELS] = { 0 };

int16_t *p_actLedValues = actLedValues;
int16_t *p_incLedValues = incLedValues;

uint8_t _data[PWM_BITS] = { 0 };      //double buffer for port values
uint8_t _data_buff[PWM_BITS] = { 0 };
uint8_t *_d;
uint8_t *_d_b;

volatile unsigned char newData = 0; //flag
volatile uint8_t pwm_status = 0;
volatile uint8_t inc_pwm_data = 1;
volatile uint8_t effect = 0;

int16_t tmp;

unsigned long milis_time = 0;
unsigned long day_milis = 0;
uint16_t daytime = 0;

//interpolace s mezemi min a max, 8bit
static uint8_t map_minmax(uint8_t x, uint8_t in_min, uint8_t in_max,
		uint8_t out_min, uint8_t out_max) {
	int16_t ret = (x - in_min) * (out_max - out_min) / (in_max - in_min)
			+ out_min;
	return (uint8_t) (ret > out_max ? out_max : ret < out_min ? out_min : ret);
}

long map(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/*
 * NO-PWM / bit angle modulation
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

static void pwm_update(void) {
    //clear
    uint16_t l = 0;
    uint8_t i,j;
    memset(_d_b, 0, 12);                    
    //rearrange values to ports
    for (i = 0; i < PWM_BITS; i++) {
        for (j = 0; j < PWM_CHANNELS; j++) {
            l = ((uint32_t)(actLedValues[j]) * pgm_read_byte(&sw_resistor[j])/100);
            _d_b[(PWM_BITS - 1) - i] = (_d_b[(PWM_BITS - 1) - i] << 1)
                    | (((l) >> ((PWM_BITS - 1) - i)) & 0x01);
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
	case reg_LED_L_0 ... reg_CRC_H:
		(*((uint8_t *) (p_incLedValues) + reg)) = value;
		break;
	case reg_DATA_OK:
		inc_pwm_data = value;
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
		ret = VERSION;
		break;
	case reg_VERSION_SUB:
		ret = VERSION_SUB;
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

static uint8_t checkActLedVal() {
	uint8_t lv = 0;
	for (uint8_t i = 0; i < PWM_CHANNELS; i++) {
		if (actLedValues[i] > 0) {
			lv = 1;
			break;
		}
	}
	return lv;
}

/**************************************
 * Main routine
 *************************************/

int main(void) {

#ifdef DEBUG
	dbg_tx_init();
#endif

// rizeni chodu
unsigned long tempTicks = 0;
unsigned long i_timeTicks = 0;
uint8_t overheat = 0;
int8_t istep = 0;


//priznak
uint8_t updateStart = 0; 

//bufer
int16_t ledValues[PWM_CHANNELS + 1] = { 0 };
int16_t prevLedValues[PWM_CHANNELS + 1] = { 0 };
int16_t *p_ledValues = ledValues;
int16_t *p_prevLedValues = prevLedValues;

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
	 * podle propojek na portech PB0, PB1, PB3, PB2
	 */
	 /*
if (!(PINB & (1 << PB6))) {
		//sepnuto, pocatecni adresa je TWIADDR2
		twiaddr = TWIADDR2;
}
  */
#ifdef DEBUG   //port B0 je debug output
	if (!(PINB & (1 << PB3))) {twiaddr |= (1 << 3);}
	if (!(PINB & (1 << PB1))) {twiaddr |= (1 << 2);}
	twiaddr |= (1 << 1);
#else
	twiaddr = ((PINB & 0b00000011) << 1);
	twiaddr |= TWIADDR2 | twiaddr | (PINB & 0b00001000);
#endif

	usiTwiSlaveInit(twiaddr, i2cReadFromRegister, i2cWriteToRegister);

	/* Inicializace timeru milis()
	 * Inicialzace hw PWM ventilatoru
	 * ventilator je pripojeny na PWM port B2
	 */
	DDRB |= (1 << PB2);  //B2 na output

	TIFR |= (1 << TOV0);
	TIMSK |= (1 << TOIE0);

	TCCR0B |= (1 << CS01); // prescale 8

	//fast PWM
	TCCR0A |= (1 << COM0A1) | (1 << WGM00) | (1 << WGM01);

	//start pwm
	OCR0A = 0;

	// Hardware init.
	LED_DIR = 0xFF; //led pins output
	LED_PORT = 0x00;   //off
	OCR1A = tbl_loop_len[MAX_LOOP] + 1;
	TCCR1B = 1 << WGM12 | 1 << CS10; // CTC-mode, F_CPU / 1
	TIMSK |= 1 << OCIE1B;  		     //start timer

	sei();

    uint16_t rampUpTime = pgm_read_word((uint16_t*)&dayTimes[0]);
    uint16_t maxLightTime = rampUpTime + pgm_read_word((uint16_t*)&dayTimes[1]);
    uint16_t rampDownTime = maxLightTime + pgm_read_word((uint16_t*)&dayTimes[2]);
    uint16_t moonTime = rampDownTime + pgm_read_word((uint16_t*)&dayTimes[3]);
    
	set_fan(255);
	_delay_ms(2000);
	set_fan(0);
	/*
	 * Inicializace teplomeru
	 * start konverze
	 */
	if (therm_reset()) {
		therm_ok = 0;
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


#define MASTER_TIMEOUT  2 //sec
	//cekej  na pwm_status from master
	uint8_t wait_tmp = 0;

	// Cekame na master status
	// pokud master status neprijde, demo provoz
	// demo provoz lze pustit i z mastera zaslanim hodnoty 0xde
	// do pwm_status
	while (pwm_status == 0) {
			wdt_reset();
			_delay_ms(1000);
			if (++wait_tmp > MASTER_TIMEOUT) {
				pwm_status = DEMO;
				break;
			}
	}
    // main loop
	while (1) {
		wdt_reset();


		/*
		 * Mereni teploty
		 */
		if (therm_ok && ((millis() - tempTicks) >= TEMPERATURE_DELAY)) { 
			//precteni teploty a start nove konverze
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
					overheat = rawTemperature > TEMPERATURE_OVERHEAT?1:0;
				}
			}
			/*
			 * start dalsiho mereni
			 */
			therm_reset();
			therm_write_byte(THERM_CMD_SKIPROM);
			therm_write_byte(THERM_CMD_CONVERTTEMP);
			tempTicks = millis();
		} 
		
		if (!therm_ok) {
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

		if (pwm_status == MASTER) {
			if (inc_pwm_data == 0) {  //dostali jsme data, kontrola CRC
				for (uint8_t i = 0; i < 8; i++) {
					xcrc = crc16_update(xcrc, LOW_BYTE(p_incLedValues[i]));
					xcrc = crc16_update(xcrc, HIGH_BYTE(p_incLedValues[i]));
				}

				if (xcrc == 0) {
					int16_t *tmpptr = p_prevLedValues;
					p_prevLedValues = p_ledValues;
					p_ledValues = p_incLedValues;
					p_incLedValues = tmpptr;
					//priznak startu interpolace
					updateStart=1;
				}
				inc_pwm_data = 1;
			}		
		} else  if (pwm_status == DEMO) {
			//test. provoz
			//zapne kazdou led na testovaci hodnotu	
			if (milis_time - day_milis > 60000) {
				day_milis = milis_time;
				daytime++;
				daytime = daytime % 1440;
				uint8_t i;
				memset(p_incLedValues,0,sizeof(actLedValues));

				if (daytime <= rampUpTime) {
					for (i = 0; i < PWM_CHANNELS; i++) {
						p_incLedValues[i] = map(daytime,0,rampUpTime,0,pgm_read_word((uint16_t*)&ledValues[i]));
					}
				} else if (daytime <= maxLightTime) {
					for (i = 0; i < PWM_CHANNELS; i++) {
						p_incLedValues[i] = pgm_read_word((uint16_t*)&ledValues[i]);
					}
				} else if (daytime <= rampDownTime) {
					for ( i = 0; i < PWM_CHANNELS; i++) {
						p_incLedValues[i] = map(daytime,maxLightTime,rampDownTime,pgm_read_word((uint16_t*)&ledValues[i]),0);
					}
					p_incLedValues[MOONLED] = map(daytime,maxLightTime,rampDownTime,pgm_read_word((uint16_t*)&ledValues[MOONLED]),pgm_read_word((uint16_t*)&ledValues[7]));
				}  else if (daytime <= moonTime) {
					p_incLedValues[MOONLED] = map(daytime,rampDownTime,moonTime,pgm_read_word((uint16_t*)&ledValues[7]),0);
				}
				int16_t *tmpptr = p_prevLedValues;
				p_prevLedValues = p_ledValues;
				p_ledValues = p_incLedValues;
				p_incLedValues = tmpptr;
				//priznak startu interpolace
				updateStart=1;
			}
		}

		if (updateStart == 1) {
#define ISTEPS       100 //pocet kroku
#define ISTEPTIMEOUT 10  //ms mezi kroky, celkovy cas prechodu ms = ISTEPS * ISTEPTIMEOUT
			if ((milis_time - i_timeTicks) > ISTEPTIMEOUT) {
				i_timeTicks = milis_time;
				for (uint8_t x = 0; x < PWM_CHANNELS; x++) {
					int16_t tmp = istep * (p_ledValues[x] - p_prevLedValues[x]);
					actLedValues[x] =  p_prevLedValues[x] + (  tmp / ISTEPS);	
					//omezeni pri prehrati ?? :TODO
					//actLedValues[x] = overheat?actLedValues[x] / 2:actLedValues[x];			
				}				
				if (++istep > ISTEPS) {
					updateStart = 0;
					istep = 0;
				}
				pwm_update();				
			}			
		}								
	}
}

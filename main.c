
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
 *      provoz bez kontroleru:
 *      po zapnuti napajeni stoupa jas po dobu jedne hodiny do maxima
 *      - delka sviceni je  urcena propojkou (10,11,12 hodin)
 *      - po uplynuti doby klesa jas na nulu po dobu jedne hodiny
 *      - externi spinaci hodiny je potreba nastavit tak, aby se vyply
 *      - po ukonceni celeho cyklu
 */


//TODO:  if overheat lower brightness
//TODO: interpolation, 200 -> 250 = 50x 1000ms/50 > 50x +1 brightness

//#define DEBUG

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

/* Thermometer Connections (At your choice) */
#define THERM_PORT 	PORTB
#define THERM_DDR 	DDRB
#define THERM_PIN 	PINB
#define THERM_DQ 	PB4

/* Utils */
#define THERM_INPUT_MODE() 		THERM_DDR&=~(1<<THERM_DQ)
#define THERM_OUTPUT_MODE()		THERM_DDR|=(1<<THERM_DQ)
#define THERM_LOW() 			THERM_PORT&=~(1<<THERM_DQ)
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
unsigned long tempTicks1 = 0;
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

const uint8_t pwmtable1[170] PROGMEM =
{
	0,1,1,1,1,1,1,1,1,1,1,1,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,3,
	3,3,3,3,3,3,3,3,3,4,4,4,4,4,4,4,4,5,5,5,5,5,5,6,6,6,6,6,7,
	7,7,7,7,8,8,8,9,9,9,9,10,10,10,11,11,11,12,12,13,13,13,14,
	14,15,15,16,16,17,17,18,19,19,20,21,21,22,23,23,24,25,26,27,
	27,28,29,30,31,32,33,35,36,37,38,39,41,42,43,45,46,48,49,51,
	53,54,56,58,60,62,64,66,68,71,73,75,78,80,83,86,89,91,95,98,
	101,104,108,111,115,119,123,127,131,135,140,144,149,154,159,
	164,170,175,181,187,193,200,206,213,220,227,235,242,250
};

const uint16_t pwmtable2[86] PROGMEM = {
	259,267,276,285,295,304,314,325,336,347,358,370,382,395,408,421,
	435,450,464,480,496,512,529,546,564,583,602,622,643,664,686,
	708,732,756,781,807,833,861,889,919,949,980,1013,1046,1081,1116,
	1153,1191,1231,1271,1313,1357,1402,1448,1496,1545,1596,1649,1703,
	1759,1818,1878,1940,2004,2070,2138,2209,2282,2357,2435,2515,2598,
	2684,2773,2864,2959,3057,3158,3262,3370,3481,3596,3715,3837,3964,4000
};

//teplomer
int8_t therm_ok = 0;
uint8_t scratchpad[9];
int8_t rawTemperature = 0;
uint16_t crc = 0xFFFF;

// pwm
int16_t val, nval = 0;

#define LED_PORT      PORTD              // Port for PWM
#define LED_DIR       DDRD               // Register for PWM
#define PWM_CHANNELS  7                  // count PWM channels
#define PWMNIGHT      19  //5% max hodnoty
#define MAXPWM        256 //63
#define PWM_BITS   12
#define LEDS       7
#define MAX_LOOP   7
#define LOOP_COUNT 8

#define NIGHT_LED1  0
#define NIGHT_LED2  8

volatile uint8_t loop    = 0;
volatile uint8_t bitmask = 0;

const unsigned int tbl_loop_bitmask[LOOP_COUNT] =  {8, 9, 11, 10, 11, 11, 10, 11};
const unsigned int tbl_loop_len[LOOP_COUNT]     =  {2044,4092,6140,8188,10236,12284,14332,16380};

int16_t incLedValues[LEDS] = {0};
int16_t ledValues[LEDS] = {0};
int16_t prevLedValues[LEDS] = {0};
int16_t actLedValues[LEDS] = {0};

int8_t interpolateTime = 0;

uint8_t _data[PWM_BITS] = {0};      //double buffer for port values
uint8_t _data_buff[PWM_BITS] = {0};
uint8_t *_d;
uint8_t *_d_b;
uint8_t newValIsOK = 0; //flag

volatile unsigned char newData = 0; //flag
volatile uint8_t pwm_status = 0;
volatile uint8_t inc_pwm_data = 1;

int16_t tmp;
int16_t tmp2;
int delta;

//linear interpolation
static inline long map(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/*
 * PWM / bit angle modulation
 */
// Timer1 handler.
ISR(TIMER1_COMPB_vect) {
	uint8_t r1, r2, r3;
    bitmask = tbl_loop_bitmask[loop];
	switch (loop) {
	 case 0:
		 r1 = _d[0];
		 r2 = _d[1];
		 r3 = _d[2];

		  __builtin_avr_delay_cycles(4L); //vyrovnani

		 //bit 0
		 LED_PORT = r1;
		 __builtin_avr_delay_cycles(3L);

		 //bit 1
		 LED_PORT = r2;
		 __builtin_avr_delay_cycles(7L);

		 //bit 2
		 LED_PORT = r3;
		 __builtin_avr_delay_cycles(9L); //6 ticks na zpracovani

		 //bit 3
		 LED_PORT = _d[3];
		 __builtin_avr_delay_cycles(25L);

		 //bit 4
		 LED_PORT = _d[4];
		 __builtin_avr_delay_cycles(57L);

		 //bit 5
		 LED_PORT = _d[5];
		 __builtin_avr_delay_cycles(121L);

		 //bit 7a
		 LED_PORT = _d[7];
		 __builtin_avr_delay_cycles(121L);

		 //bit 6a
		 LED_PORT = _d[6];
		 __builtin_avr_delay_cycles(121L);

		 //bit 7b, 7c
		 LED_PORT = _d[7];
		 __builtin_avr_delay_cycles(249L);

		 //bit 6b
		 LED_PORT = _d[6];
		 __builtin_avr_delay_cycles(121L);

		 //bit 7d
		 LED_PORT = _d[7];
		 __builtin_avr_delay_cycles(121L);

		 //bit 8
		 LED_PORT = _d[8];
		 OCR1B=tbl_loop_len[loop];
		 break;
	 case 1 ... 7:
		 LED_PORT = _d[bitmask];
		 OCR1B=tbl_loop_len[loop];
		 break;
	}

	 //loop++;
	 if(++loop > MAX_LOOP) {
		 loop=0;
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
 OCR1A = tbl_loop_len[MAX_LOOP]+1;
 TCCR1B = 1<<WGM12 | 1<<CS10; // CTC-mode, F_CPU / 1
 TIMSK |=  1<<OCIE1B;  		  //start timer
}

void pwm_update(void) {
	//clear
	memset(_d_b, 0, 12);
	//rearrange values to ports
	for(int i = 0; i < PWM_BITS; i++) {
		for (int j = 0; j < LEDS; j++) {
			_d_b[(PWM_BITS-1)-i] = (_d_b[(PWM_BITS-1)-i] << 1) | ((actLedValues[j] >> ((PWM_BITS-1) - i)) & 0x01);
		}
	}

	//wait for prev. data process
	while (newData) {;};
	//set new data flag
	newData = 1;
}

/*
 * Rutiny pro teplomer
 *
 */

static uint8_t therm_reset() {

	uint8_t i;
		//Pull line low and wait for 480uS
	ATOMIC_BLOCK(ATOMIC_FORCEON) {
		THERM_LOW();
		THERM_OUTPUT_MODE();
		_delay_us(330);
		//Release line and wait for 60uS
		THERM_INPUT_MODE();
		_delay_us(40);
		//Store line value and wait until the completion of 480uS period
		i=(THERM_PIN & (1<<THERM_DQ));
	}
	_delay_us(330);
	//Return the value read from the presence pulse (0=OK, 1=WRONG)
	return i;
}

static void therm_write_bit(uint8_t bit) {
	ATOMIC_BLOCK(ATOMIC_FORCEON) {
		//Pull line low for 1uS
		THERM_LOW();
		THERM_OUTPUT_MODE();
		_delay_us(1);
		//If we want to write 1, release the line (if not will keep low)
		if(bit) THERM_INPUT_MODE();
		//Wait for 60uS and release the line
		_delay_us(40);
		THERM_INPUT_MODE();
	}
}

static uint8_t therm_read_bit(void) {

	uint8_t bit=0;
	ATOMIC_BLOCK(ATOMIC_FORCEON) {
		//Pull line low for 1uS
		THERM_LOW();
		THERM_OUTPUT_MODE();
		_delay_us(1);

		//Release line and wait for 14uS
		THERM_INPUT_MODE();
		_delay_us(10);

		//Read line value
		if(THERM_PIN&(1<<THERM_DQ)) bit=1;
	}
	//Wait for 45uS to end and return read value
	_delay_us(32);
	return bit;
}

static uint8_t therm_read_byte(void) {
	uint8_t i=8, n=0;
	while(i--) {
		n>>=1; //Shift one position right and store read value
		n|=(therm_read_bit()<<7);
	}
	return n;
}

static void therm_write_byte(uint8_t byte) {

	uint8_t i=8;
	while(i--) {
		therm_write_bit(byte&1); //Write actual bit and shift one position right to make the next bit ready
		byte>>=1;
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
		case reg_CRC_H:
			BYTEHIGH(crc) = value;
			break;
		case reg_CRC_L:
			BYTELOW(crc) = value;
			break;
		case reg_LED_H_0:
		case reg_LED_H_1:
		case reg_LED_H_2:
		case reg_LED_H_3:
		case reg_LED_H_4:
		case reg_LED_H_5:
		case reg_LED_H_6:
			BYTEHIGH(incLedValues[reg/2]) = value;
			break;
		case reg_LED_L_0:
		case reg_LED_L_1:
		case reg_LED_L_2:
		case reg_LED_L_3:
		case reg_LED_L_4:
		case reg_LED_L_5:
		case reg_LED_L_6:
			BYTELOW(incLedValues[reg/2]) = value;
			break;
		case reg_DATA_OK:
			inc_pwm_data = value;
			break;
	}
}

uint8_t i2cReadFromRegister(uint8_t reg) {
	uint8_t ret = 0x00;
	switch (reg) {
		case reg_LED_H_0:
		case reg_LED_H_1:
		case reg_LED_H_2:
		case reg_LED_H_3:
		case reg_LED_H_4:
		case reg_LED_H_5:
		case reg_LED_H_6:
			ret =  HIGH_BYTE(ledValues[reg/2]);
			break;
		case reg_LED_L_0:
		case reg_LED_L_1:
		case reg_LED_L_2:
		case reg_LED_L_3:
		case reg_LED_L_4:
		case reg_LED_L_5:
		case reg_LED_L_6:
			ret =   LOW_BYTE(ledValues[reg/2]);
			break;
		case reg_CRC_H:
			ret =   HIGH_BYTE(crc);
			break;
		case reg_CRC_L:
			ret =   LOW_BYTE(crc);
			break;
		case reg_MASTER:
			ret =   pwm_status;
			break;
		case reg_THERM_STATUS:  //temperature status
			ret =   therm_ok;
			break;
		case reg_RAW_THERM:
			ret =   rawTemperature;
			break;
		}
	return ret;
}


/*
 * Milis()
 */


#define clockCyclesToMicroseconds(a) ( ((a) * 1000L) / (F_CPU / 1000L) )
#define MICROSECONDS_PER_TIMER0_OVERFLOW (clockCyclesToMicroseconds(64 * 256))
#define MILLIS_INC (MICROSECONDS_PER_TIMER0_OVERFLOW / 1000)
// the fractional number of milliseconds per timer0 overflow. we shift right
// by three to fit these numbers into a byte. (for the clock speeds we care
// about - 8 and 16 MHz - this doesn't lose precision.)
#define FRACT_INC ((MICROSECONDS_PER_TIMER0_OVERFLOW % 1000) >> 3)
#define FRACT_MAX (1000 >> 3)

volatile unsigned long timer0_overflow_count = 0;
volatile unsigned long timer0_millis = 0;
static unsigned char timer0_fract = 0;

ISR(TIMER0_OVF_vect)
{
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

unsigned long millis()
{
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
	ATOMIC_BLOCK(ATOMIC_FORCEON) {
		if (pwm == 0) {
			TCCR0A  &= ~(1 << COM0A1);
			PORTB &= ~(1 << PB3);
		} else {
			TCCR0A  |= (1 << COM0A1);
		}
		OCR0A = pwm;  //port B2 je PWM fan
	}
}

static uint16_t crc16_update(uint16_t crc, uint8_t a) {
  int i;

  crc ^= a;
  for (i = 0; i < 8; ++i)
  {
    if (crc & 1)
      crc = (crc >> 1) ^ 0xA001;
    else
      crc = (crc >> 1);
  }

  return crc;
}

/**************************************
 * Main routine
 *
 *************************************/

int main(void) {

#ifdef DEBUG
	dbg_tx_init();
#endif

    _d   = _data;
    _d_b = _data_buff;

    //disable irq
	cli();

	/*
	 * Watchdog enable 4sec
	 */
/*
	wdt_reset();
	MCUSR &= ~(1<<WDRF);
	WDTCR |= (1<<WDCE) | (1<<WDE);
	WDTCR  = (1<<WDE)  | (1<<WDP3);;
*/

	//input a pullup, PB0=A0. PB1=A2, PB3=A3     na B6 je spinac ON/OFF, na B4 je DS1820
#ifdef DEBUG
	PORTB |= (1 << PB1) | (1 << PB3) | (1 << PB6); //| (1 << PB0) ; PB0 je debug output
	DDRB  &= ~(1 << PB1) & ~(1 << PB3)  & ~(1 << PB6); //& ~(1 << PB0) ;
#else
	PORTB |= (1 << PB1) | (1 << PB3) | (1 << PB6) | (1 << PB0) ;
	DDRB  &= ~(1 << PB1) & ~(1 << PB3)  & ~(1 << PB6) & ~(1 << PB0) ;
#endif


	/*
	 * Precteni a nastaveni TWI adresy
	 * podle propojek na portech PB0, PB1, PB3
	 */
#ifdef DEBUG   //port B0 je debug output
	if (!(PINB & (1 << PB3))) { twiaddr |=  (1 << 3); }
	if (!(PINB & (1 << PB1))) { twiaddr |=  (1 << 2); }
	twiaddr |=  (1 << 1);
#else
	if (!(PINB & (1 << PB3))) { twiaddr |=  (1 << 3); }
	if (!(PINB & (1 << PB1))) { twiaddr |=  (1 << 2); }
	if (!(PINB & (1 << PB0))) { twiaddr |=  (1 << 1); }
#endif

	usiTwiSlaveInit(twiaddr, i2cReadFromRegister, i2cWriteToRegister);

	/* Inicializace timeru milis()
	 * Inicialzace hw PWM ventilatoru
	 * ventilator je pripojeny na PWM port B2
	 */
	DDRB   |= (1 << PB2);  //B2 na output

	TIFR |= (1 << TOV0);
	TIMSK |= (1 << TOIE0);

	TCCR0B |= (1 << CS01) | (1 << CS00); // prescale 64

	//fast PWM
	TCCR0A |= (1 << COM0A1) | (1 << WGM00) | (1 << WGM01) ;

	//start pwm
	OCR0A = 0;

	_pwm_init();

	sei();

	/*
	 * Inicializace teplomeru
	 *
	 */

	if(therm_reset()) {
		therm_ok = 0;
		set_fan(255);
	} else {
		therm_ok = 1;
		set_fan(0);
		therm_write_byte(THERM_CMD_SKIPROM);
		therm_write_byte(THERM_CMD_CONVERTTEMP);
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
			setDayTime = (10L * HOUR); setNightTime = 0;
			break;
		case 2:
			setDayTime = (11L * HOUR); setNightTime = 0;
			break;
		case 3:
			setDayTime = (12L * HOUR); setNightTime = 0;
			break;
		case 4:
			setDayTime = (10L * HOUR); setNightTime = 1 * HOUR;
			break;
		case 5:
			setDayTime = (11L * HOUR); setNightTime = 1 * HOUR;
			break;
		case 6:
			setDayTime = (12L * HOUR); setNightTime = 1 * HOUR;
			break;
		default:
			setDayTime = (10L * HOUR); setNightTime = 0;
	}

#endif

	//wait 30 sec for pwm_status from master
	uint8_t wait_tmp=0;

	while (pwm_status != 0xFF) {
		_delay_ms(1000);
		if (++wait_tmp > 20) break;
	}

	while(1) {
		 //wdt_reset();

		 /*
		  * Mereni teploty
		  */

		 if (therm_ok) {
			 if ((millis() - tempTicks1) >= 2000) {  //precteni teploty a start nove konverze
    			therm_reset();
				therm_write_byte(THERM_CMD_SKIPROM);
				therm_write_byte(THERM_CMD_RSCRATCHPAD);
				for (uint8_t i = 0; i < 9; i++) {
					scratchpad[i] = therm_read_byte();;
				}
				rawTemperature = (int8_t)((scratchpad[1] << 8) | scratchpad[0]) >> 1;

				therm_reset()  ;
				therm_write_byte(THERM_CMD_SKIPROM);
				therm_write_byte(THERM_CMD_CONVERTTEMP);
				tempTicks1 = millis();
			}

			 /*
			 * ventilator dle teploty
			 * (x - in_min) * (out_max - out_min) / (50 - 20) + out_min;
			 */

			 if (rawTemperature > 40) {
				 set_fan(255);
			 } else if (rawTemperature > 25) {
				 set_fan((rawTemperature-20) * 10);
			 } else {
				 set_fan(0);
			 }
		}

		/*
		 *  Hlavni rizeni
		 */

		uint16_t xcrc = 0xffff;

		milis_time = millis();

		switch (pwm_status) {
			case  0xFF:
					if (inc_pwm_data == 0) {  //dostali jsme data, kontrola CRC
							for (uint8_t i = 0; i < 7; i++) {
								xcrc = crc16_update(xcrc,LOW_BYTE(incLedValues[i]));
								xcrc = crc16_update(xcrc,HIGH_BYTE(incLedValues[i]));
							}

							xcrc = crc16_update(xcrc, LOW_BYTE(crc));
							xcrc = crc16_update(xcrc, HIGH_BYTE(crc));

							if (xcrc == 0) {
								for(uint8_t x = 0; x < LEDS; x++) {
									//ulozime predchozi hodnoty
									//prevLedValues[x] = ledValues[x];
									//zkopirujeme prichozi data
									//ledValues[x] = incLedValues[x];
									actLedValues[x] = incLedValues[x];
								}
								//priznak startu interpolace
								//newValIsOK = 1;
								pwm_update();
							}
							inc_pwm_data = 1;
					}

					/*
					* TODO:
					* interpolace na 1000 ms?
					* mezi dvemi body je 25 hodnot
					* funkce:
					* _prevLedValues[x] = startovaci hodnota
					* ledValues[x]  = nova hodnota
					* actLedValues[x] = vypocitana hodnota, ktera jde do fce pwm_update
					* actLedValues[x] = map(cas++,0,25,_prevLedValues[x],ledValues[x])
					*/
/*
					if ( newValIsOK && (milis_time - i_timeTicks > 40)) {
						i_timeTicks = milis_time;
						for (uint8_t x=0;x < LEDS; x++) {
							actLedValues[x]=map(interpolateTime,0,24,prevLedValues[x],ledValues[x]);
							//prevLedValues[x] = actLedValues[x];
						}
						pwm_update();
						interpolateTime++;
						if (interpolateTime > 24) { newValIsOK = 0;interpolateTime=0;};
					}
*/
					break;
			default: //autonomni provoz, dle nastavenych prepinacu adresy pocita delku dne od zapnuti
					 //TODO: fan off v noci
					 //TODO: opravit nval
					if ((milis_time - timeTicks) >= SEC)  {
						timeTicks = milis_time;

						dayTime++;
						if (dayTime > DAY) dayTime = 0;

						if (dayTime <= RAMPUP ) {
							//ramp up
							val = map(dayTime,0,RAMPUP,0,MAXPWM);
						} else if ( (dayTime >= (setDayTime - RAMPDOWN) ) && (dayTime <= setDayTime)) {
							//rampDown
							val = map(dayTime,setDayTime,setDayTime-RAMPDOWN,MAXPWM,0);
						} else if ((dayTime > setDayTime) && (dayTime <= (setDayTime + setNightTime) ) && (setNightTime > 0) ) {
							//night ramp down
							val = map(dayTime,setDayTime,setDayTime+setNightTime,PWMNIGHT,0);
						} else if ((dayTime > RAMPUP) && (dayTime < setDayTime-RAMPDOWN)) {
							//day
							val = MAXPWM;
						} else {
							//night
							val  = 0;
						}
					if (val < 170) {
						tmp = pgm_read_byte (& pwmtable1[val]);
					} else if (val < 256) {
						tmp = pgm_read_word (& pwmtable2[val-170]);
					} else {
						tmp = 0;
					}

					tmp = tmp > 4095?4095:tmp;

					for (uint8_t x = 0; x < LEDS; x++) {

						//ledValues[x] = tmp;
						actLedValues[x] =  tmp;
						//newValIsOK = 1;
					}

					pwm_update();

				}
		}
	}
}

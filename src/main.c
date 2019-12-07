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
 *	Verze pro attiny 2313
 */

 
#define VERSION      100
#define VERSION_SUB  6

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

// pwm
int16_t val, nval = 0;

#define PWM_FREQ      240    

#define LED_PORT      PORTD  // Port for PWM
#define LED_DIR       DDRD   // Register for PWM
#define PWM_BITS      12
#define PWM_CHANNELS  7

//sw resistor - set max current for channel
// Rs resitor = 0.1 Ohm
// blue, yellow, green, red, white, rb, uv
// 
//na konektoru zleva, pohled shora, konektor smeruje ke me,
// Iout = (0.1 * D) /  Rs
const uint8_t sw_resistor[PWM_CHANNELS] PROGMEM= {70,70,100,100,100,100,35};

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
 *  vyznamnou chybu muzou zpusobit dalsi preruseni
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
int16_t *p_incLedValues = incLedValues;
int16_t ledValues[PWM_CHANNELS + 1] = { 0 };
int16_t *p_ledValues = ledValues;

uint8_t _data[PWM_BITS] = { 0 };      //double buffer for port values
uint8_t _data_buff[PWM_BITS] = { 0 };
uint8_t *_d;
uint8_t *_d_b;
uint8_t updateStart = 0;

volatile unsigned char newData = 0; //flag
volatile uint8_t pwm_status = 0;
volatile uint8_t inc_pwm_data = 1;
volatile uint8_t effect = 0;

int16_t tmp;

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
	uint32_t l = 0;
	uint8_t r = 0;
	int16_t led_r = 0;

void pwm_update(void) {
	
	//clear
	l = 0;
	r = 0;
	led_r = 0;
	memset(_d_b, 0, 12);
					
	//rearrange values to ports
	for (int i = 0; i < PWM_BITS; i++) {
		for (int j = 0; j < PWM_CHANNELS; j++) {
			r = pgm_read_byte(&sw_resistor[j]);
			l = (uint32_t)(p_ledValues[j]) * r;
			led_r = l/100UL;
			_d_b[(PWM_BITS - 1) - i] = (_d_b[(PWM_BITS - 1) - i] << 1)
					| (((led_r) >> ((PWM_BITS - 1) - i)) & 0x01);
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
 * Rutiny pro i2c
 *
 */
void i2cWriteToRegister(uint8_t reg, uint8_t value) {
	switch (reg) {
	case reg_MASTER:
		pwm_status = value;
		updateStart = 0;			
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
		ret = (*((uint8_t *) (p_ledValues) + reg));		
		break;
	case reg_MASTER:
		ret = pwm_status;
		break;
	case reg_THERM_STATUS:  //temperature status
		ret = 0;
		break;
	case reg_RAW_THERM:
		ret = 0;
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
		if (p_ledValues[i] > 0) {
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

//bufer
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

	//povoleni interruptu pro timer0 - millis()
	//TIFR |= (1 << TOV0);
	//TIMSK |= (1 << TOIE0);

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

#define MASTER_TIMEOUT  10 //sec
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

		//fan
		if (checkActLedVal() == 0) {
			set_fan(0);
		} else {
			set_fan(255);
		}

		 // Hlavni rizeni
		uint16_t xcrc = 0xffff;

		if (pwm_status == MASTER) {
			if (inc_pwm_data == 0) {  //dostali jsme data, kontrola CRC
				for (uint8_t i = 0; i < 8; i++) {
					xcrc = crc16_update(xcrc, LOW_BYTE(p_incLedValues[i]));
					xcrc = crc16_update(xcrc, HIGH_BYTE(p_incLedValues[i]));
				}

				if (xcrc == 0) {
					int16_t *tmpptr = p_incLedValues;
					p_incLedValues = p_ledValues;
					p_ledValues = tmpptr;					
					updateStart = 1; 
				}
				inc_pwm_data = 1;
			}		
		} else  if (pwm_status == DEMO) {
			//test. provoz
			//zapne kazdou led na testovaci hodnotu				
			for (uint8_t i = 0; i < PWM_CHANNELS; i++) {
				ledValues[i] = 100;	

			}
			if (updateStart == 0) {
				updateStart = 2;	
				pwm_update();
			}					
			
		}
		if (updateStart == 1) {
			 pwm_update();								
			 updateStart = 0;
		}
	}
}

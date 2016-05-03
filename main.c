
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
 *      B6 : IN spinac den/noc
 *
 *      D0 - D6: OUT pwm
 */

//#define DEBUG

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

//PWM http://www.mikrocontroller.net/articles/Soft-PWM
#define F_CPU         16000000L
#define F_PWM         120L               // PWM-Freq
#define PWM_PRESCALER 8                  // Vorteiler für den Timer
#define PWM_STEPS     1024              // PWM-Schritte pro Zyklus(1..256)
#define PWM_PORT      PORTD              // Port for PWM
#define PWM_DDR       DDRD               // Register for PWM
#define PWM_CHANNELS  7                  // count PWM channels

#define STEPS  PWM_STEPS

#define T_PWM (F_CPU/(PWM_PRESCALER*F_PWM*PWM_STEPS)) // Systemtakte pro PWM-Takt

#if ((T_PWM*PWM_PRESCALER)<(111+5))
    #error T_PWM be too small, must be enlarged or F_CPU f_PWM or PWM_STEPS reduced
#endif

#if ((T_PWM*PWM_STEPS)>65535)
    #error Period of the PWM too big! F_PWM or PWM_PRESCALER increase.
#endif


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
//unsigned long startTime = 0;

#define LED_ON     1
#define LED_OFF    0
#define LED_UP     2
#define LED_DOWN   3


//teplomer
int8_t therm_ok = 1;
uint8_t scratchpad[9];
int8_t rawTemperature = 0;
uint16_t crc = 0xFFFF;

// pwm
uint16_t pwm_timing[PWM_CHANNELS+1];          // Zeitdifferenzen der PWM Werte
uint16_t pwm_timing_tmp[PWM_CHANNELS+1];

uint8_t  pwm_mask[PWM_CHANNELS+1];            // Bitmaske für PWM Bits, welche gelöscht werden sollen
uint8_t  pwm_mask_tmp[PWM_CHANNELS+1];        // ändern uint16_t oder uint32_t für mehr Kanäle


uint16_t  pwm_set[PWM_CHANNELS] = {0,0,0,0,0,0,0};           // Einstellungen für die einzelnen PWM-Kanäle
volatile uint16_t  *pwm_setting=pwm_set;

uint16_t  pwm_sett_buff[PWM_CHANNELS] = {0,0,0,0,0,0,0};
volatile uint16_t  *pwm_setting_buffer=pwm_sett_buff;

uint16_t  pwm_setting_tmp[PWM_CHANNELS+1];     // Einstellungen der PWM Werte, sortiert
                                              // ändern auf uint16_t für mehr als 8 Bit Auflösung

volatile uint8_t pwm_cnt_max=1;               // Zählergrenze, Initialisierung mit 1 ist wichtig!
volatile uint8_t pwm_sync;                    // Update jetzt möglich

uint16_t *isr_ptr_time  = pwm_timing;
uint16_t *main_ptr_time = pwm_timing_tmp;

uint8_t *isr_ptr_mask  = pwm_mask;              // Bitmasken fuer PWM-Kanäle
uint8_t *main_ptr_mask = pwm_mask_tmp;          // ändern uint16_t oder uint32_t für mehr Kanäle

volatile uint8_t pwm_status = 0;
volatile uint8_t pwm_dirty = 1;


static inline long map(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

//#define map(x,in_min,in_max,out_min,out_max) ((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

/*
 * Software PWM 10 bit
 *
 */
static inline void tausche_zeiger(void) {
    uint16_t *tmp_ptr16;
    uint8_t *tmp_ptr8;                          // ändern uint16_t oder uint32_t für mehr Kanäle

    tmp_ptr16 = isr_ptr_time;
    isr_ptr_time = main_ptr_time;
    main_ptr_time = tmp_ptr16;
    tmp_ptr8 = isr_ptr_mask;
    isr_ptr_mask = main_ptr_mask;
    main_ptr_mask = tmp_ptr8;
}

// PWM Update, berechnet aus den PWM Einstellungen
// die neuen Werte für die Interruptroutine

static inline void pwm_update(void) {

    uint8_t i, j, k;
    uint8_t m1, m2, tmp_mask;                   // ändern uint16_t oder uint32_t für mehr Kanäle
    uint16_t min, tmp_set;                       // ändern auf uint16_t für mehr als 8 Bit Auflösung

    // PWM Maske für Start berechnen
    // gleichzeitig die Bitmasken generieren und PWM Werte kopieren

    m1 = 1;
    m2 = 0;
    for(i=1; i<=(PWM_CHANNELS); i++) {
        main_ptr_mask[i]=~m1;                       // Maske zum Löschen der PWM Ausgänge
        pwm_setting_tmp[i] = pwm_setting[i-1];
        if (pwm_setting_tmp[i]!=0) m2 |= m1;        // Maske zum setzen der IOs am PWM Start
        m1 <<= 1;
    }
    main_ptr_mask[0]=m2;                            // PWM Start Daten

    // PWM settings sortieren; Einfügesortieren

    for(i=1; i<=PWM_CHANNELS; i++) {
        min=PWM_STEPS-1;
        k=i;
        for(j=i; j<=PWM_CHANNELS; j++) {
            if (pwm_setting_tmp[j]<min) {
                k=j;                                // Index und PWM-setting merken
                min = pwm_setting_tmp[j];
            }
        }
        if (k!=i) {
            // ermitteltes Minimum mit aktueller Sortiertstelle tauschen
            tmp_set = pwm_setting_tmp[k];
            pwm_setting_tmp[k] = pwm_setting_tmp[i];
            pwm_setting_tmp[i] = tmp_set;
            tmp_mask = main_ptr_mask[k];
            main_ptr_mask[k] = main_ptr_mask[i];
            main_ptr_mask[i] = tmp_mask;
        }
    }

    // Gleiche PWM-Werte vereinigen, ebenso den PWM-Wert 0 löschen falls vorhanden

    k=PWM_CHANNELS;             // PWM_CHANNELS Datensätze
    i=1;                        // Startindex

    while(k>i) {
        while ( ((pwm_setting_tmp[i]==pwm_setting_tmp[i+1]) || (pwm_setting_tmp[i]==0))  && (k>i) ) {

            // aufeinanderfolgende Werte sind gleich und können vereinigt werden
            // oder PWM Wert ist Null
            if (pwm_setting_tmp[i]!=0)
                main_ptr_mask[i+1] &= main_ptr_mask[i];        // Masken vereinigen

            // Datensatz entfernen,
            // Nachfolger alle eine Stufe hochschieben
            for(j=i; j<k; j++) {
                pwm_setting_tmp[j] = pwm_setting_tmp[j+1];
                main_ptr_mask[j] = main_ptr_mask[j+1];
            }
            k--;
        }
        i++;
    }

    // letzten Datensatz extra behandeln
    // Vergleich mit dem Nachfolger nicht möglich, nur löschen
    // gilt nur im Sonderfall, wenn alle Kanäle 0 sind
    if (pwm_setting_tmp[i]==0) k--;

    // Zeitdifferenzen berechnen

    if (k==0) { // Sonderfall, wenn alle Kanäle 0 sind
        main_ptr_time[0]=(uint16_t)T_PWM*PWM_STEPS/2;
        main_ptr_time[1]=(uint16_t)T_PWM*PWM_STEPS/2;
        k=1;
    }
    else {
        i=k;
        main_ptr_time[i]=(uint16_t)T_PWM*(PWM_STEPS-pwm_setting_tmp[i]);
        tmp_set=pwm_setting_tmp[i];
        i--;
        for (; i>0; i--) {
            main_ptr_time[i]=(uint16_t)T_PWM*(tmp_set-pwm_setting_tmp[i]);
            tmp_set=pwm_setting_tmp[i];
        }
        main_ptr_time[0]=(uint16_t)T_PWM*tmp_set;
    }

    // auf Sync warten

    pwm_sync=0;             // Sync wird im Interrupt gesetzt
    while(pwm_sync==0);
    cli();
    // Zeiger tauschen
    	tausche_zeiger();
    	pwm_cnt_max = k;
    sei();

}

// Timer 1 Output COMPARE B Interrupt
ISR(TIMER1_COMPA_vect) {
    static uint16_t pwm_cnt = 0;                // ändern auf uint16_t für mehr als 8 Bit Auflösung
    uint8_t tmp;                                // ändern uint16_t oder uint32_t für mehr Kanäle

    OCR1A += isr_ptr_time[pwm_cnt];
    tmp    = isr_ptr_mask[pwm_cnt];

    if (pwm_cnt == 0) {
        PWM_PORT = tmp;                         // Ports setzen zu Begin der PWM
        										// zusätzliche PWM-Ports hier setzen
        pwm_cnt++;
    } else {
        PWM_PORT &= tmp;                        // Ports löschen
                                                // zusätzliche PWM-Ports hier setzen
        if (pwm_cnt == pwm_cnt_max) {
            pwm_sync = 1;                       // Update jetzt möglich
            pwm_cnt  = 0;
        } else pwm_cnt++;
    }
}

/*
 * Rutiny pro teplomer
 *
 */

static uint8_t therm_reset()
{

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

static void therm_write_bit(uint8_t bit)
{
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

static uint8_t therm_read_bit(void)
{

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

static uint8_t therm_read_byte(void)
{
	uint8_t i=8, n=0;
	while(i--)
	{
		//Shift one position right and store read value
		n>>=1;
		n|=(therm_read_bit()<<7);
	}
	return n;
}

static void therm_write_byte(uint8_t byte)
{

	uint8_t i=8;

	while(i--)
	{
		//Write actual bit and shift one position right to make the next bit ready
		therm_write_bit(byte&1);
		byte>>=1;
	}
}

/*
 * Rutiny pro i2c
 *
 */

void i2cWriteToRegister(uint8_t reg, uint8_t value) {

	switch (reg) {
		case 16:
			pwm_status = value;
			break;
		case 14:
			BYTEHIGH(crc) = value;
			break;
		case 15:
			BYTELOW(crc) = value;
			pwm_dirty = 0;
			break;
		case 0:
		case 2:
		case 4:
		case 6:
		case 8:
		case 10:
		case 12:
			BYTEHIGH(pwm_setting_buffer[reg/2]) = value;
			break;
		case 1:
		case 3:
		case 5:
		case 7:
		case 9:
		case 11:
		case 13:
			BYTELOW(pwm_setting_buffer[reg/2]) = value;
			break;

/*
		default:
			if((reg & 1)) {
				//LOW BYTE
				BYTELOW(pwm_setting[reg/2]) = value;
				pwm_dirty = 0; //if (pwm_status == 0xFF) pwm_update();
			} else {
				//HIGH BYTE
				BYTEHIGH(pwm_setting[reg/2]) = value;
				pwm_dirty = 1;
			}
*/
	}
/*
*/
}

uint8_t i2cReadFromRegister(uint8_t reg) {
	if (reg >= 0 && reg <= 14) {
		if((reg & 1)) {
			//LOW BYTE
			return BYTELOW(pwm_setting[reg/2]);
		} else {
			//HIGH BYTE
			return BYTEHIGH(pwm_setting[reg/2]);
		}
	} else {
		switch (reg) {
		case 16:
			return pwm_status;
			break;
		case 17:  //temperature status
			return therm_ok;
			break;
		case 18:
			return rawTemperature;
			break;
		default:
			return 0xFF;
			break;
		}
	}
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

int main(void) {

#ifdef DEBUG
	dbg_tx_init();
#endif

	cli();

	/*
	 * Watchdog enable 4sec
	 */

	wdt_reset();
	MCUSR &= ~(1<<WDRF);
	WDTCR |= (1<<WDCE) | (1<<WDE);
	WDTCR  = (1<<WDE)  | (1<<WDP3);;

	/*
	 * Inicializace PWM
	 */
	memset(pwm_set,0,sizeof(pwm_set));
	memset(pwm_sett_buff,0,sizeof(pwm_set));

	/*
	 * Inicializace vzstupu
	 */
	PWM_DDR = 0xFF;  	// Port na vystup
	PORTD = 0x00; 		// port D na LOW

	//input a pullup, PB0=A0. PB1=A2, PB3=A3     na B6 je spinac ON/OFF, na B4 je DS1820
#ifdef DEBUG
	PORTB |= (1 << PB1) | (1 << PB3) | (1 << PB6); //| (1 << PB0) ;
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
	if (!(PINB & (1 << PB3))) {
	twiaddr |=  (1 << 3);
	}
	if (!(PINB & (1 << PB1))) {
	twiaddr |=  (1 << 2);
	}
	twiaddr |=  (1 << 1);
#else
	if (!(PINB & (1 << PB3))) {
	twiaddr |=  (1 << 3);
	}
	if (!(PINB & (1 << PB1))) {
	twiaddr |=  (1 << 2);
	}
	if (!(PINB & (1 << PB0))) {
	twiaddr |=  (1 << 1);
	}
#endif

	usiTwiSlaveInit(twiaddr, i2cReadFromRegister, i2cWriteToRegister);

	/* Inicialiyace timeru milis()
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

	/*
	 *  Inicializace Timer1 pro sw PWM LED
	 */
    // Timer 1 OCRA1, als variablen Timer nutzen
	TIMSK  |= (1 << OCIE1A);
    //TCCR1B |= (1 << CS10) |  (1 << CS11);  //  Prescaler 64
	TCCR1B |= (1 << CS11);  //  Prescaler 8

	sei();

	/*
	 * Inicializace teplomeru
	 *
	 */

	if(therm_reset()) {
				therm_ok = 0;
#ifdef DEBUG
	dbg_putchar('T');dbg_putchar('0');;dbg_putchar('\r');dbg_putchar('\n');
#endif
	} else {
				therm_ok = 1;
#ifdef DEBUG
	dbg_putchar('T');dbg_putchar('1');;dbg_putchar('\r');dbg_putchar('\n');
#endif
	}

	if (therm_ok) {
		set_fan(0);
		therm_write_byte(THERM_CMD_SKIPROM);


		therm_write_byte(THERM_CMD_CONVERTTEMP);
	} else {
		set_fan(255);
	}

	/*
	 * Precteni konfiguracnich prepinacu, nastaveni doby sviceni
	 * pro autonomni provoz
	 * pokud je sepnut prepinac na PB6
	 * pak pocitame cas od zapnuti
	 * dle nastavenych propojek
	 */

	switch ((twiaddr & 0b00001110) >> 1) {
		case 1:
			setDayTime = 10L * 3600L - 60L; setNightTime = 0;
			break;
		case 2:
			setDayTime = 11L * 3600L - 60L; setNightTime = 0;
			break;
		case 3:
			setDayTime = 12L * 3600L - 60L; setNightTime = 0;
			break;
		case 4:
			setDayTime = 10L * 3600L - 60L; setNightTime = 1 * 3600;
			break;
		case 5:
			setDayTime = 11L * 3600L - 60L; setNightTime = 1 * 3600;
			break;
		case 6:
			setDayTime = 12L * 3600L - 60L; setNightTime = 1 * 3600;
			break;
		default:
			setDayTime = 24L * 3600L; setNightTime = 0;
	}


	while(1) {

		 wdt_reset();

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
		uint16_t xcrc = 0xFFFF;
		switch (pwm_status) {
		case  0xFF:
		    	if (pwm_dirty == 0) {  //dostali jsme data
						for (uint8_t i = 0; i < 7; i++) {
							//kontrolujeme crc
							xcrc = crc16_update(xcrc, LOW_BYTE(pwm_setting_buffer[i]));
							xcrc = crc16_update(xcrc, HIGH_BYTE(pwm_setting_buffer[i]));
						}
						xcrc = crc16_update(xcrc, LOW_BYTE(crc));
						xcrc = crc16_update(xcrc, HIGH_BYTE(crc));

						if (xcrc == 0) {
							//TODO:  if not overheat
							uint16_t *tmpptr =  pwm_setting;
							pwm_setting = pwm_setting_buffer;
							pwm_setting_buffer = tmpptr;
							//TODO: else lower brightness
							pwm_update();
						}
						pwm_dirty = 1;
		    	}
				break;
		default: //autonomni provoz, dle nastavenych prepinacu adresy pocita delku dne
			if ((millis() - timeTicks) >= 40) {
				timeTicks = millis();
				dayTime++;
				if (dayTime > (24*3600L)) dayTime = 0;
				int val, nval;
				if (dayTime <= (4*3600L) ) {
					//ramp up
					val = map(dayTime,0,(4*3600L),0,511);
					nval = val;
				} else if ( (dayTime >= (setDayTime - (4*3600)) ) && (dayTime <= setDayTime)) {
					//rampDown
					nval = map(dayTime,setDayTime-(4*3600L),setDayTime,511,setNightTime>0?100:0);
					val = map(dayTime,setDayTime-(4*3600L),setDayTime,511,0);
				} else if ((dayTime > setDayTime) && (dayTime <= (setDayTime + setNightTime) ) && (setNightTime > 0) ) {
					//night ramp down
					val = map(dayTime,setDayTime,setDayTime+setNightTime,100,0);
					nval = val;
				} else if ((dayTime > (4*3600)) && (dayTime < setDayTime-(4*3600L))) {
					//day
					val = 511;
					nval = val;
				} else {
					//night
					val  = 0;
					nval = val;
				}
				pwm_setting[0] = val;
				pwm_setting[1] = nval;
				pwm_setting[2] = val;
				pwm_setting[3] = val;
				pwm_setting[4] = val;
				pwm_setting[5] = nval;
				pwm_setting[6] = val;
				pwm_update();
			}
		}
	}
}

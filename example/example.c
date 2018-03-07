/* example.c
 *
 *  Author: Mishsael Oguntimehin
 *   Notes:
 *
 *          - Pin assignment:
 *            | Port | Pin | Use                         |
 *            |------+-----+-----------------------------|
 *            | A    | PA0 | Busbar Voltage              |
 *            | A    | PA1 | Busbar Current              |
 *            | A    | PA2 | Wind Turbine Capacity       |
 *            | A    | PA3 | PV Capacity                 |
 *            | A    | PA4 | Call For Load 1             |
 *            | A    | PA5 | Call For Load 2             |
 *            | A    | PA6 | Call For Load 3             |


 *            | D    | PD0 | Charge battery call         |
 *            | D    | PD1 | Discharge battery call      |
 *            | D    | PD2 | Switch load 1               |
 *            | D    | PD3 | Switch Load 2               |
 *            | D    | PD4 | Switch Load 3               |
 */

#define PICTOR_FASTMODE
#include <avr/io.h>

#include "../pictor.h"
#include "../fonts/OryxB.h"
#include "../fonts/Mash.h"
#include "plug.h"
#include <util/delay.h>
#include <math.h>
#include <avr/interrupt.h>
#include<stdio.h>
#include<stdlib.h>

#define PWM_DUTY_MAX 255
#define ADCMAXREAD   1023
#define ADCREF_V     3.3
#define testnumber 400
void Switch_Char_Battery(const uint8_t State);
void Switch_Dis_Battery(const uint8_t State);
void Switch_load1(const uint8_t State);
void Switch_load2(const uint8_t State);
void Switch_load3(const uint8_t State);
void init_pwm(void);
void pwm_duty(uint8_t x);
void init_adc(int channel);
float pv_capacity(void);
float wind_capacity(void);
float battery_capacity(void);
int read_adc(void);
int display_float(float x);
float map(float x, float in_max, float out_max);


//this text is stored entirely in program memory, useful for long strings that won't change
//it is also useful if the same string is used many times in the code, to declare the string once at the beginning rather than every time it is used
//this means that it is only in the code once, rather than once for every time it is used

int main() {
	//DDRB |= _BV(PB7);
	pictorInit(0);
	//PORTB |= _BV(PB7);
	pictorSetRotation(1);
	pictorDrawAll(BLACK);

//SET PORT D AS Output
	DDRD = 0xFF;
	PORTD = 0x00;
//setup pwm for analouge output
	init_pwm();
//pictorBacklightState(1);
//	Welcome
////////////set up load demand calc///////////////////////////
int Call;
int Call_2;
int Call_3;

////////////////////Left Section statics/////////////////////////
//Draw value boxes
	pictorDrawBox((point){2, 50}, (point){77, 78}, CYAN);
	pictorDrawBox((point){4, 52}, (point){75, 76}, BLACK);

	pictorDrawBox((point){2, 80}, (point){77, 108}, CYAN);
	pictorDrawBox((point){4, 82}, (point){75, 106}, BLACK);

	pictorDrawBox((point){2, 110}, (point){77, 138}, CYAN);
	pictorDrawBox((point){4, 112}, (point){75, 136}, BLACK);
//Middle Section statics
	pictorDrawLine((point){249, 0}, (point){249, 240}, CYAN);
	pictorDrawS("MICRO-GRID", (point){55,0},WHITE, BLACK, Mash,1);
	pictorDrawS("SMART METER", (point){55,8},WHITE, BLACK, Mash,2);
	pictorDrawS("Current Mains Usage", (point){85,30},CYAN, BLACK, Mash,1);
////////////////////Right Section statics//////////////////////////////////////
	pictorDrawS("ACTIVE", (point){260,0},YELLOW, BLACK, OryxB, 1 );
	pictorDrawS("LOADS", (point){263,8}, YELLOW, BLACK, OryxB, 1);
	//smart texts, possibly to be written in another functions (copy purposes)
		pictorDrawD(1, (point){300,53}, BLACK, BLACK, Mash, 4, 1);

	//pictorDrawS("Last Modified:", (point){0, 231}, MAGENTA, BLACK, Mash,1);
	//pictorDrawS(__TIMESTAMP__, (point){120, 231}, MAGENTA, BLACK, Mash,1);
	//PORTB &= ~_BV(PB7);
	while (1)
	{
		Switch_load1(1);
		_delay_ms(1000);
		Switch_load1(0);
		_delay_ms(1000);
		//////////////mains A//////////////
		pictorDrawD(display_float(1.16), (point){5,53},PALE CYAN, BLACK, Mash, 3, 2);
		pictorDrawBox((point){25, 71}, (point){27,73}, MAGENTA);
		pictorDrawS("W", (point){56,60}, MAGENTA, BLACK, Mash,2);

		/////////////solar A////////////////
		pictorDrawD(display_float(pv_capacity()), (point){5,83},PALE CYAN, BLACK, Mash, 3, 2);
		pictorDrawBox((point){26, 101}, (point){28,103}, MAGENTA);
		pictorDrawS("W", (point){56,90}, MAGENTA, BLACK, Mash,2);

		////////////wind A ////////////////
		pictorDrawD(display_float(wind_capacity()), (point){5,113},PALE CYAN, BLACK, Mash, 3, 2);
		pictorDrawBox((point){26, 131}, (point){28,133}, MAGENTA);
		pictorDrawS("W", (point){56,120}, MAGENTA, BLACK, Mash,2);

		pwm_duty(200);
	}
}
////////digital outputs///////////
void Switch_Char_Battery(const uint8_t State)
{
	if (State == 0)
	{
		PORTD &= ~_BV(0);
	}
	if (State == 1)
	{
		PORTD |= _BV(0);
	}
}

void Switch_Dis_Battery(const uint8_t State)
{
	if (State == 0)
	{
		PORTD &= ~_BV(1);
	}
	if (State == 1)
	{
		if ((PINC & _BV(PC7)) == 1)
		{
			PORTD |= _BV(1);
		}
	}
}


void Switch_load1(const uint8_t State)
{
	if (State == 0)
	{
		PORTD &= ~_BV(2);
		pictorDrawD(1, (point){265,53}, BLACK, BLACK, Mash, 4, 1);
	}
	if ((PINA & _BV(PA4)) != 0)
		{
			if (State == 1)
		{
			PORTD |= _BV(2);
			pictorDrawD(1, (point){265,53}, CYAN, BLACK, Mash, 4, 1);
		}
	}
}

void Switch_load2(const uint8_t State)
{
	if (State == 0)
	{
		PORTD &= ~_BV(3);
		pictorDrawD(2, (point){265,101}, BLACK, BLACK, Mash, 4, 1);
	}
	if ((PINA & _BV(PA5)) != 0)
	{
		if (State == 1)
		{
			PORTD |= _BV(3);
			pictorDrawD(2, (point){265,101}, CYAN, BLACK, Mash, 4, 1);
		}
	}
}

void Switch_load3(const uint8_t State)
{
	if (State == 0)
	{
		PORTD &= ~_BV(4);
		pictorDrawD(3, (point){265,149}, BLACK, BLACK, Mash, 4, 1);
	}
	if ((PINA & _BV(PA6)) != 0)
	{
		if (State == 1)
		{
			PORTD |= _BV(4);
			pictorDrawD(3, (point){265,149}, CYAN, BLACK, Mash, 4, 1);
		}
	}
}

///////// Analogue output to testbed ////////////////
void init_pwm(void)
{
    /* TIMER 2 */
    DDRD |= _BV(PD6); /* PWM out */
    DDRD |= _BV(PD7); /* inv. PWM out */


    TCCR2A = _BV(WGM20) | /* fast PWM/MAX */
	     _BV(WGM21) | /* fast PWM/MAX */
	     _BV(COM2A1); /* A output */
    TCCR2B = _BV(CS20) | _BV(CS21);   /* no prescaling */
}
/////////////// control//////////////
int power_usage(void)
{
	int power = 50;
	int load1;
	int load2;
	int load3;
	return power;
}

int solar(void)
{
	int solar = 50;
	int load1;
	int load2;
	int load3;
	return solar;
}

void pwm_duty(uint8_t x)
{
    x = x > PWM_DUTY_MAX ? PWM_DUTY_MAX : x;
    OCR2A = 200;
}

//////////// Can we turn on a load??//////////////
float available(void)
{
	float pv = 0;
	float wind = 0;
	float bat = 0;
	float available1 = 0 ;
	pv = pv_capacity();
	wind = wind_capacity();
	bat = battery_capacity();
	available1 = pv + wind + bat;
	return available1;
}

float battery_capacity(void)
{
	return 1.1;
}

float pv_capacity(void)
{
	float load = 0;
	init_adc(3);
	load = (read_adc()/1024);
	return load;
}

float wind_capacity(void)
{
	float load = 0;
	init_adc(2);
	load = (read_adc()/1024);
	return load;
}

float bus_current(void)
{
	float load = 0;
	init_adc(1);
	load = read_adc()/1024;
	return load;
}

void request(uint8_t load)
{
	if (load = 1)
	{
		available();
	}
}

float map(float x, float in_max, float out_max)
{
    return (x) * (out_max) / (in_max);
}

int display_float(float x)
{
	int i;
	float d;
	int dec;
	i = (int)x;
	d = x - i;
	d = d * 10;
	dec = round(d);
	int value;
	value = (i*10) + dec;
	return value;
}
////////////Analogue input from testbed//////////
void init_adc (int channel)
{
	/* TODO: Initialisation code */
	ADCSRA |= _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0); /* F_ADC = F_CPU/128 */
	ADMUX = channel; /* Select channel n, where n = State */
	ADMUX |= _BV(REFS0) | _BV(REFS1); /* AVCC reference */
	ADMUX |= _BV(ADLAR); /* ADCH contains 8 MSBs */
	ADCSRA |= _BV(ADEN); /* Enable ADC */
}

int read_adc(void)
{
	/* TODO: Acquisition code */
	ADCSRA |= _BV(ADSC); /* Start Conversions */
	while(ADCSRA & _BV(ADSC));
	return ADC;
}
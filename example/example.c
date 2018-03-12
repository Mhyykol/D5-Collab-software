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
 *            | D    | PD5 | LED Output                  |
 *	      | D    | PD6 | PWM Output                  |
 *	      | D    | PD7 | inv PWM Output for mains    |
 */

#define PICTOR_FASTMODE
#include <avr/io.h>

#include "../pictor.h"
#include "../fonts/OryxB.h"
#include "../fonts/Mash.h"
//#include "plug.h"
#include <util/delay.h>
#include <math.h>
#include <avr/interrupt.h>
#include <stdio.h>
//#include <time.h>

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
void pwm_duty(int x);
void init_adc(int channel);
float pv_capacity(void);
float wind_capacity(void);
float power(void);
float bus_current(void);
float bus_voltage(void);
float available(void);
float mains_supply(void);
int read_adc(void);
int display_float(float x);
float map(float x, float in_max, float out_max);
int battery_capacity(void);
int mains_capacity(void);

int charge = 0;
int discharge = 0;
int mains = 0;
int check = 0;

//this text is stored entirely in program memory, useful for long strings that won't change
//it is also useful if the same string is used many times in the code, to declare the string once at the beginning rather than every time it is used
//this means that it is only in the code once, rather than once for every time it is used

int main() {
	//DDRB |= _BV(PB7);
	pictorInit(0);
	//PORTB |= _BV(PB7);
	pictorSetRotation(1);
	pictorDrawAll(BLACK);

//SET PORT A FOR Inputs, pins 0-6 inputs, pin 7 output so doesnt affect system
	DDRA = 0x00;
	PORTA = 0xFF;
//SET PORT D AS Output
	DDRD = 0xFF;
	PORTD = 0x00;
//setup pwm for analouge output
	init_pwm();
//pictorBacklightState(1);
//	Welcome

//LED PIN/////
	PORTD |= (1<<PD5);

//Variable Declarations//
	int value;
	float total;
	float available_supply;
	float required_supply = 0;
	int Call;
	int Call_2;
	int Call_3;
	int Flag = 0;
  Switch_load1(0);
  Switch_load2(0);
  Switch_load3(0);

	////////////////////Left Section statics/////////////////////////
	//Draw value boxes
		pictorDrawBox((point){2, 50}, (point){77, 78}, CYAN);
		pictorDrawBox((point){4, 52}, (point){75, 76}, BLACK);
		pictorDrawS("Current Power Usage", (point){79,60},CYAN, BLACK, Mash,1);

		pictorDrawBox((point){2, 80}, (point){77, 108}, CYAN);
		pictorDrawBox((point){4, 82}, (point){75, 106}, BLACK);
		pictorDrawS("Total Power Usage", (point){79,90},CYAN, BLACK, Mash,1);

		pictorDrawBox((point){2, 110}, (point){77, 138}, CYAN);
		pictorDrawBox((point){4, 112}, (point){75, 136}, BLACK);
		pictorDrawS("Mains Capacity", (point){79,120},CYAN, BLACK, Mash,1);

		pictorDrawBox((point){2, 140}, (point){77, 168}, CYAN);
		pictorDrawBox((point){4, 142}, (point){75, 166}, BLACK);
		pictorDrawS("Wind Capacity", (point){79,150},CYAN, BLACK, Mash,1);

		pictorDrawBox((point){2, 170}, (point){77, 198}, CYAN);
		pictorDrawBox((point){4, 172}, (point){75, 196}, BLACK);
		pictorDrawS("Solar Capacity", (point){79,180},CYAN, BLACK, Mash,1);

	//Middle Section statics
		pictorDrawLine((point){249, 0}, (point){249, 240}, CYAN);
		pictorDrawS("MICRO-GRID", (point){0,0},CYAN, BLACK, Mash,1);
		pictorDrawS("SMART METER", (point){0,8},WHITE, BLACK, Mash,2);
		pictorDrawS("Current Mains Usage", (point){85,30},CYAN, BLACK, Mash,1);
	////////////////////Right Section statics//////////////////////////////////////
		pictorDrawS("ACTIVE", (point){260,0},YELLOW, BLACK, OryxB, 1 );
		pictorDrawS("LOADS", (point){263,8}, YELLOW, BLACK, OryxB, 1);
		pictorDrawS("Total", (point){263,185}, YELLOW, BLACK, OryxB, 1);
		pictorDrawS("Demand", (point){263,193}, WHITE, BLACK, OryxB, 1);
		//smart texts, possibly to be written in another functions (copy purposes)
			pictorDrawD(1, (point){300,53}, BLACK, BLACK, Mash, 4, 1);

		//pictorDrawS("Last Modified:", (point){0, 231}, MAGENTA, BLACK, Mash,1);
		//pictorDrawS(__TIMESTAMP__, (point){120, 231}, MAGENTA, BLACK, Mash,1);
		//PORTB &= ~_BV(PB7);


while (1)
{
//////////////////draw boxes////////////////////////////////////////
pictorDrawD(display_float(power()), (point){5,53},PALE CYAN, BLACK, Mash, 3, 2);
pictorDrawBox((point){25, 71}, (point){27,73}, MAGENTA);
pictorDrawS("W", (point){56,60}, MAGENTA, BLACK, Mash,2);

/////////////solar A////////////////
pictorDrawD(display_float(pv_capacity()), (point){5,83},PALE CYAN, BLACK, Mash, 3, 2);
pictorDrawBox((point){26, 101}, (point){28,103}, MAGENTA);
pictorDrawS("A", (point){56,90}, MAGENTA, BLACK, Mash,2);

////////////wind A ////////////////
pictorDrawD(display_float(wind_capacity()), (point){5,113},PALE CYAN, BLACK, Mash, 3, 2);
pictorDrawBox((point){26, 131}, (point){28,133}, MAGENTA);
pictorDrawS("A", (point){56,120}, MAGENTA, BLACK, Mash,2);


//////////////////// total power /////////////////////////////////////
	pictorDrawD(display_float(power()), (point){5,143},PALE CYAN, BLACK, Mash, 3, 2);
	pictorDrawBox((point){26, 161}, (point){28,163}, MAGENTA);
	pictorDrawS("A", (point){56,150}, MAGENTA, BLACK, Mash,2);
/////////////////available Capacity ///////////////////////////////
	pictorDrawBox((point){26, 191}, (point){28,193}, MAGENTA);
	pictorDrawD(display_float(available()), (point){5,173},PALE CYAN, BLACK, Mash,
	3, 2);
	pictorDrawS("A", (point){56,180}, MAGENTA, BLACK, Mash,2);
/////////////////////////////////////////////////////////////////
	pictorDrawD(display_float(required_supply), (point){263,204}, CYAN, BLACK, Mash, 2, 2);
	pictorDrawBox((point){277, 210}, (point){279,212}, MAGENTA);
	pictorDrawS("A", (point){300,204}, MAGENTA, BLACK, Mash,2);


//////////////CHECK WIND & PV CAPACITY////////////////////////////


////////////Check for load calls/////////////////////////////////

	if(PINA & (1<<PA4))
		{
			Call = 1;
			//Switch_load1(1);
		}
	else
		{
		    Call = 0;
			//Switch_load1(0);
		}

	if(PINA & (1<<PA5))
		{
			Call_2 = 1;
			//Switch_load2(1);
		}
	else
		{
			Call_2 = 0;
			//Switch_load2(0);
		}

	if(PINA & (1<<PA6))
		{
			Call_3 = 1;
			//Switch_load3(1);
		}
	else
		{
		    Call_3 = 0;
			//Switch_load3(0);
		}
///////////////how much juice do we need ///////////////////

	if(Call == 1)
	{
		required_supply = 0.8;
		if(Call_2 == 1)
		{
			required_supply = required_supply + 1.8;
			if(Call_3 == 1)
			{
				required_supply = required_supply + 1.4;
				//pictorDrawD(4, (point){,}, CYAN, BLACK, Mash, 4, 1);
			}
			else if(Call_3 == 0)
			{
				required_supply = required_supply;
				//pictorDrawD(2.6, (point){,}, CYAN, BLACK, Mash, 4, 1);
			}
		}
		else if(Call_2 == 0)
		{
			required_supply = required_supply;
			if(Call_3 == 1)
			{
				required_supply = required_supply + 1.4;
				//pictorDrawD(2.2, (point){,}, CYAN, BLACK, Mash, 4, 1);
			}
			else if(Call_3 == 0)
			{
				required_supply = required_supply;
				//pictorDrawD(0.8, (point){,}, CYAN, BLACK, Mash, 4, 1);
			}
		}
	}
	else if(Call == 0)
	{
		required_supply = 0;
		if(Call_2 == 1)
			{
				required_supply = 1.8;
				if(Call_3 == 1)
				{
					required_supply = required_supply + 1.4;
					//pictorDrawD(3.2, (point){,}, CYAN, BLACK, Mash, 4, 1);
				}
				else if(Call_3 == 0)
				{
					required_supply = required_supply;
					//pictorDrawD(1.8, (point){,}, CYAN, BLACK, Mash, 4, 1);
				}
			}
		else if(Call_2 == 0)
			{
				required_supply = 0;
				if(Call_3 == 1)
				{
					required_supply = 1.4;
					//pictorDrawD(1.4, (point){,}, CYAN, BLACK, Mash, 4, 1);
				}
				else if(Call_3 == 0)
				{
					required_supply = 0;
					//pictorDrawD(0, (point){,}, CYAN, BLACK, Mash, 4, 1);
				}
			}
	}


	check = 3;

	if(PINA & (1<<PA6) && (available() >= required_supply))
		{
			//Call_3 = 1;
			Switch_load3(1);

		}
	else if(required_supply > available())
		{
			Switch_load3(0);
			required_supply = required_supply - 1.4;
		}
	else
		{
		    //Call_3 = 0;
			Switch_load3(0);
		}

	if(PINA & (1<<PA5) && (available() >= required_supply))
		{
			//Call_2 = 1;
			Switch_load2(1);
		}
	else if(required_supply > available())
		{
			Switch_load2(0);
			required_supply = required_supply - 1.8;
		}
	else
		{
			//Call_2 = 0;
			Switch_load2(0);
		}

	if(PINA & (1<<PA4) && (available() >= required_supply))
		{
			//Call = 1;
			Switch_load1(1);
		}
	else if(required_supply > available())
		{
			Switch_load1(0);
			required_supply = required_supply - 0.8;
		}
	else
		{
		    //Call = 0;
			Switch_load1(0);
		}

	if(Call == 1)
	{
		required_supply = 0.8;
		if(Call_2 == 1)
		{
			required_supply = required_supply + 1.8;
			if(Call_3 == 1)
			{
				required_supply = required_supply + 1.4;
				//pictorDrawD(4, (point){,}, CYAN, BLACK, Mash, 4, 1);
			}
			else if(Call_3 == 0)
			{
				required_supply = required_supply;
				//pictorDrawD(2.6, (point){,}, CYAN, BLACK, Mash, 4, 1);
			}
		}
		else if(Call_2 == 0)
		{
			required_supply = required_supply;
			if(Call_3 == 1)
			{
				required_supply = required_supply + 1.4;
				//pictorDrawD(2.2, (point){,}, CYAN, BLACK, Mash, 4, 1);
			}
			else if(Call_3 == 0)
			{
				required_supply = required_supply;
				//pictorDrawD(0.8, (point){,}, CYAN, BLACK, Mash, 4, 1);
			}
		}
	}
	else if(Call == 0)
	{
		required_supply = 0;
		if(Call_2 == 1)
			{
				required_supply = 1.8;
				if(Call_3 == 1)
				{
					required_supply = required_supply + 1.4;
					//pictorDrawD(3.2, (point){,}, CYAN, BLACK, Mash, 4, 1);
				}
				else if(Call_3 == 0)
				{
					required_supply = required_supply;
					//pictorDrawD(1.8, (point){,}, CYAN, BLACK, Mash, 4, 1);
				}
			}
		else if(Call_2 == 0)
			{
				required_supply = 0;
				if(Call_3 == 1)
				{
					required_supply = 1.4;
					//pictorDrawD(1.4, (point){,}, CYAN, BLACK, Mash, 4, 1);
				}
				else if(Call_3 == 0)
				{
					required_supply = 0;
					//pictorDrawD(0, (point){,}, CYAN, BLACK, Mash, 4, 1);
				}
			}
	}

	/*if(required_supply > available())
	{
		Switch_load3(0);
		required_supply = required_supply - 1.4;
		if(required_supply > available())
		{
			Switch_load2(0);
			required_supply = required_supply - 1.8;
			if(required_supply > available())
			{
				Switch_load1(0);
				required_supply = required_supply - 0.8;
			}
		}
	}
	else
	{}*/
//////////DISCHARGE/CHARGE BATTERY IF REQUIRED///////////////////
	/*
		do{
			Switch_Dis_Battery(1);
			Switch_Char_Battery(0);
			available_supply = available_supply + 1;
			Flag = Flag - 1;
		} while ((required_supply > available_supply) && (Flag > 0))
	*/

	check = 1;
	if((required_supply > available()) && (Flag > 100))
	{
		Switch_Char_Battery(0);
		Switch_Dis_Battery(1);
		discharge = 1;
		charge = 0;
		Flag = Flag - 1;
	}
	else if(available() > required_supply + 1)
	{
		Switch_Dis_Battery(0);
		Switch_Char_Battery(1);
		charge = 1;
		discharge = 0;
		Flag = Flag + 1;
	}
	else
	{
		Switch_Dis_Battery(0);
		Switch_Char_Battery(0);
		charge = 0;
		discharge = 0;
	}

//////////SUPPLY MAINS IF REQURED//////////////////////////
	check = 2;
	if(required_supply > available())
	{
		//pwm_duty(200);
		mains = 1;
		//Switch_load1(1);
	}
	else
	{
		//pwm_duty(0);
		mains = 0;
		//Switch_load1(0);
	}

/*//CHECK THERE IS SUFFICENT SUPPLY FOR LOADS////
	check = 3;
	if(required_supply > available())
	{
		Switch_load3(0);
		required_supply = required_supply - 1.4;
		if(required_supply > available_supply)
		{
			Switch_load2(0);
			required_supply = required_supply - 1.8;
			if(required_supply > available_supply)
			{
				Switch_load1(0);
				required_supply = required_supply - 0.8;
			}
		}
	}
	else
	{
	}*/
//////////////////////////////////////////////////////////////////////
pwm_duty(150);
}
	return 1;
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
		PORTD |= _BV(1);
	}
}


void Switch_load1(const uint8_t State)
{
	if (State == 0)
	{
		PORTD &= ~_BV(2);
		pictorDrawD(1, (point){265,53}, BLACK, BLACK, Mash, 4, 1);
	}
	if (State == 1)
	{
		PORTD |= _BV(2);
		pictorDrawD(1, (point){265,53}, CYAN, BLACK, Mash, 4, 1);
	}
}

void Switch_load2(const uint8_t State)
{
	if (State == 0)
	{
		PORTD &= ~_BV(3);
		pictorDrawD(2, (point){265,101}, BLACK, BLACK, Mash, 4, 1);
	}
	if (State == 1)
	{
		PORTD |= _BV(3);
		pictorDrawD(2, (point){265,101}, CYAN, BLACK, Mash, 4, 1);
	}
}

void Switch_load3(const uint8_t State)
{
	if (State == 0)
	{
		PORTD &= ~_BV(4);
		pictorDrawD(3, (point){265,149}, BLACK, BLACK, Mash, 4, 1);
	}
	if (State == 1)
	{
		PORTD |= _BV(4);
		pictorDrawD(3, (point){265,149}, CYAN, BLACK, Mash, 4, 1);
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
    TCCR2B = _BV(CS21);   /*128 prescaler */
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

void pwm_duty(int x)
{
  //  x = x > PWM_DUTY_MAX ? PWM_DUTY_MAX : x;
    OCR2A = x;
}

////////////Analogue input from testbed//////////
void init_adc(int channel)
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
	init_adc(0);
	load = read_adc()/1024;
	return load;
}


float bus_voltage(void)
{
	float load = 0;
	init_adc(1);
	load = read_adc()/1024;
	return load;
}

float power(void)
{
  return bus_current()*bus_voltage();
}

float available(void)
{
	float pv = 0;
	float wind = 0;
	float bat = 0;
	float mai = 0;
	float available1 = 0 ;

	if (check == 1)
	{
		pv = pv_capacity();
		wind = wind_capacity();
	}
	else if (check == 2)
	{
		pv = pv_capacity();
		wind = wind_capacity();
		bat = battery_capacity();
	}
	else
	{
		pv = pv_capacity();
		wind = wind_capacity();
		bat = battery_capacity();
		mai = mains_capacity();
	}

	available1 = ((pv + wind)*0.25) + bat + mai;

	/*if (required_supply > available1)
	{
		mains = mains_supply();
		available1 = available1 + mains;
	}*/
	return available1;
}

int battery_capacity(void)
{
	float load = 0;
	if(charge == 1)
	{
		load = -1;
	}
	else if(discharge == 1)
	{
		load = 1;
	}
	else
	{
		load = 0;
	}
	return load;
}

int mains_capacity(void)
{
	float load = 0;
	if(mains == 1)
	{
		load = 3;
	}
	else
	{
		load = 0;
	}
	return load;
}

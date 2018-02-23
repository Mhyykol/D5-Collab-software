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
 *            | D    | PD5 | LED OUTPUT                  |
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

#define PWM_DUTY_MAX 255
#define ADCMAXREAD   1023
#define ADCREF_V     3.3
void Switch_Char_Battery(const uint8_t State);
void Switch_Dis_Battery(const uint8_t State);
void Switch_load1(const uint8_t State);
void Switch_load2(const uint8_t State);
void Switch_load3(const uint8_t State);
void init_pwm(void);
void pwm_duty(uint8_t x);
void init_adc (void);
void init_adc2 (const uint8_t State);
uint16_t read_adc(void);


//this text is stored entirely in program memory, useful for long strings that won't change
//it is also useful if the same string is used many times in the code, to declare the string once at the beginning rather than every time it is used
//this means that it is only in the code once, rather than once for every time it is used

int main() {
	DDRB |= _BV(PB7);
	pictorInit(0);
	PORTB |= _BV(PB7);
	pictorSetRotation(1);
	pictorDrawAll(BLACK);

//SET PORT A FOR Inputs, pins 0-6 inputs, pin 7 output so doesnt affect system
	DDRA = 0x80;
	PORTA = 0x00;
//SET PORT D AS Output
	DDRD = 0xFF;
	PORTD = 0x00;
//setup pwm for analouge output
	init_pwm();
//pictorBacklightState(1);
//	Welcome

//LED PIN/////
	PORTD |= _BV(5);
for(;;)
{
////////////////////Left Section statics/////////////////////////
//Draw value boxes
	pictorDrawBox((point){2, 50}, (point){105, 78}, CYAN);
	pictorDrawBox((point){4, 52}, (point){103, 76}, BLACK);
	pictorDrawD(5000, (point){5,53},PALE CYAN, BLACK, Mash, 3, 4);
	pictorDrawS("W", (point){68,60}, MAGENTA, BLACK, Mash,1);
//Middle Section statics
	pictorDrawLine((point){239, 0}, (point){239, 240}, CYAN);
	pictorDrawS("MICRO-GRID", (point){55,0},WHITE, BLACK, Mash,1);
	pictorDrawS("SMART METER", (point){55,8},WHITE, BLACK, Mash,2);
	pictorDrawS("Current Mains Usage", (point){85,30},CYAN, BLACK, Mash,1);
////////////////////Right Section statics//////////////////////////////////////
	pictorDrawBox((point){239,0}, (point){320,240}, CYAN);
	pictorDrawS("ACTIVE", (point){260,0},YELLOW, BLACK, OryxB, 1 );
	pictorDrawS("LOADS", (point){263,8}, YELLOW, BLACK, OryxB, 1);
	//smart texts, possibly to be written in another functions
		pictorDrawD(1, (point){250,53}, BLACK, CYAN, Mash, 4, 1);
		pictorDrawD(2, (point){250,101}, BLACK, CYAN, Mash, 4, 1);
		pictorDrawD(3, (point){250,149}, BLACK, CYAN, Mash, 4, 1);
		pictorDrawCircle((point){298,69}, 5, GREEN);
		pictorDrawCircle((point){298,117}, 5, GREEN);
		pictorDrawCircle((point){298,165}, 5, GREEN);

	pictorDrawS("Last Modified:", (point){0, 231}, MAGENTA, BLACK, Mash,1);
	pictorDrawS(__TIMESTAMP__, (point){120, 231}, MAGENTA, BLACK, Mash,1);
	//PORTB &= ~_BV(PB7);

//////////////READ BUSBAR CURRENT AND VOLTAGE VALUES///////////////
	uint16_t value;
	double busbar_voltage;
	double busbar_current;
	int power;
	double wind_capacity;
	double pv_capacity;
	double available supply;
	double required supply;
	int Call;
	int Call_2;
	int Call_3;
	int Flag;

	init_adc2(0);
	read_adc();
	value = ADC;
	busbar_voltage = 100*3.3*(value/1024);

	init_adc2(1);
	read_adc();
	value = ADC;
	busbar_current = 3.3*(value/1024);

	power = busbar_current * busbar_voltage;
/////////////////////////////////////////////////////////////////

////////////Check for load calls/////////////////////////////////
	if(PINA4)
		{
			Call = 1;
			Switch_load1(1);
		}
	else if(~PINA4)
		{
		    Call = 0;
			Switch_load1(0);
		}

	if(PINA5)
		{
			Call_2 = 1;
			Switch_load2(1);
		}
	else if(~PINA5)
		{
			Call_2 = 0;
			Switch_load2(0);
		}

	if(PINA6)
		{
			Call_3 = 1;
			Switch_load3(1);
		}
	else if(~PINA6)
		{
		    Call_3 = 0;
			Switch_load3(0);
		}
///////////////////////////////////////////////////////////////////

//////////////CHECK WIND & PV CAPACITY////////////////////////////

	init_adc2(2);
	read_adc();
	value = ADC;
	wind_capacity = 3.3*(value/1024);

	init_adc2(3);
	read_adc();
	value = ADC;
	pv_capacity = 3.3*(value/1024);

	available_supply = wind_capacity + pv_capacity;

	if(Call == 1)
	{
		required_supply = 0.8;
		if(Call_2 == 1)
		{
			required_supply = required_supply + 1.8;
			if(Call_3 == 1)
			{
				required_supply = required_supply + 1.4;
			}
			else if(Call_3 == 0)
			{
				required_supply = required_supply;
			}
		}
		else if(Call_2 == 0)
		{
			required_supply = required_supply;
			if(Call_3 == 1)
			{
				required_supply = required_supply + 1.4;
			}
			else if(Call_3 == 0)
			{
				required_supply = required_supply;	
			}
		}
	}
	else if(Call == 0)
	{
		required_supply = 0;
		if(Call_2 == 1)
			{
				required_supply = required_supply + 1.8;
				if(Call_3 == 1)
				{
					required_supply = required_supply + 1.4;
				}
				else if(Call_3 == 0)
				{
						required_supply = required_supply;
				}
			}
		else if(Call_2 == 0)
			{
				required_supply = required_supply;
				if(Call_3 == 1)
				{
					required_supply = required_supply + 1.4;
				}
				else if(Call_3 == 0)
				{
					required_supply = required_supply;
				}
			}
	}
	
	//////////DISCHARGE/CHARGE BATTERY IF REQUIRED///////////////////
	if((required_supply > available_supply) && (Flag > 100))
	{
		Switch_Dis_Battery(1);
		available_supply = available_supply + 1;
	}
	else if(required_supply < (available - 1))
	{
		Switch_Char_Battery(1);
		Flag = Flag + 1;
		available_supply = available_supply - 1;
	}	
	
	///////////SUPPLY MAINS IF REQURED//////////////////////////
	if(required_supply > available_supply)
	{
		Switch_Dis_Battery(0);
		Switch_Char_Battery(0);
		//Supply analogue input for Mains Power
	}
	else
	{
		
	}
//////////////////////////////////////////////////////////////////////

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
	}
	if (State == 1)
	{
		PORTD |= _BV(2);
	}
}

void Switch_load2(const uint8_t State)
{
	if (State == 0)
	{
		PORTD &= ~_BV(3);
	}
	if (State == 1)
	{
		PORTD |= _BV(3);
	}
}

void Switch_load3(const uint8_t State)
{
	if (State == 0)
	{
		PORTD &= ~_BV(4);
	}
	if (State == 1)
	{
		PORTD |= _BV(4);
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
    TCCR2B = _BV(CS20);   /* no prescaling */
}

void pwm_duty(uint8_t x)
{
    x = x > PWM_DUTY_MAX ? PWM_DUTY_MAX : x;
    OCR2A = x;
}

/*
ISR(TIMER1_COMPA_vect)
{
	v_error = SETPOINT_V - v_load();

		if (control == 1)
		{
			if (v_load() < SETPOINT_V )
				{
					SET_PWM ++;
					if (SET_PWM == 246)
					{
						SET_PWM = 245;


					}
					pwm_duty(SET_PWM);  // Limited by PWM_DUTY_MAX
				}
			else
				{
					SET_PWM --;
					if (SET_PWM == 0)
					{
						SET_PWM = 1;
					}
					pwm_duty(SET_PWM);
				}
		}
		else
		{
				float iMax = 50;
				float iMin = 0;

				float P_Term;
				float I_Term;
				float D_Term;
				int new_ADC_value;
				float PWM_Duty;

				new_ADC_value = v_load();

				err_value = (SETPOINT_V - new_ADC_value);

				i_Temp += err_value;

				if (i_Temp > iMax)
					{
						i_Temp = iMax;
					}

				else if (i_Temp < iMin)
					{
						i_Temp = iMin;
					}

				P_Term = Kp * err_value;
				I_Term = Ki * i_Temp;
				D_Term = Kd * (d_Temp - err_value);
				d_Temp = err_value;

				PWM_Duty = PWM_Temp + (P_Term + I_Term + D_Term);

				if (PWM_Duty > 250)
				{
					PWM_Duty = 245;
				}
				else if (PWM_Duty < 0)
				{
					PWM_Duty = 0;
				}
				pwm_duty(PWM_Duty);

				PWM_Temp = PWM_Duty;

		}

				/*if(v_error <= 0.2) // If the difference of voltage is significant - set the LED high
						{
						PORTB |= _BV(7);
						adjusting = 0;
						}

					else // If not, then low
					{

					PORTB &= !_BV(7);
					adjusting = 1;
					}*/

////////////Analogue input from testbed//////////
void init_adc (void)
{
    /* REFSx = 0 : Select AREF as reference
     * ADLAR = 0 : Right shift result
     *  MUXx = 0 : Default to channel 0
     */
    ADMUX = 0x0F;
    /*  ADEN = 1 : Enable the ADC
     * ADPS2 = 1 : Configure ADC prescaler
     * ADPS1 = 1 : F_ADC = F_CPU / 64
     * ADPS0 = 0 :       = 187.5 kHz
     */
    ADCSRA = _BV(ADEN) | _BV(ADPS2) | _BV(ADPS1);
}

void init_adc2 (const uint8_t State)
{
	/* TODO: Initialisation code */
	ADCSRA |= _BV(ADPS2) | _BV(ADPS1); /* F_ADC = F_CPU/64 */
	ADMUX = State; /* Select channel n, where n = State */
	ADMUX |= _BV(REFS0); /* AVCC reference */
	ADMUX |= _BV(ADLAR); /* ADCH contains 8 MSBs */
	ADCSRA |= _BV(ADEN); /* Enable ADC */
}

uint16_t read_adc(void)
{
	/* TODO: Acquisition code */
	ADCSRA |= _BV(ADSC); /* Start Conversions */
	while(ADCSRA & _BV(ADSC));
	return ADC;
}


/*
 * D5.c
 *
 *  Created on: 13 Feb 2018
 *      Author: lc8g16
 */

#include <stdio.h>
#include <stdlib.h>
//#include <avr/io.h>
//#include <util/delay.h>
#include <math.h>

double busbar_current;//global variables
double busbar_voltage;
double power;
int call;

int main()
{
disp_power();
charging(0);
discharging(1);

if(1==1)//(PINA4)//check for load calls
	{
		call = 1;
	}
else if(0==1)//(PINA5)
	{
		call = 2;
	}
else if(0==1)//(PINA6)
	{
		call = 3;
	}

loads(call,1);

return 0;
}

void disp_power()
{
	int i = 0;
	double p;
	double total = 0;

	for(i=0;i<10;i++)
	{
	busbar_current = 5;
	busbar_voltage = 7;
	//current = PINA1;
	//voltage = PINA0;
	p = busbar_current * busbar_voltage;
	total = total + p;

	}

	printf("Total Power is %fW\n",total);

	power = total/10;
	//power = current * voltage;

	printf("Average Power is %fW\n",power);

	return;
}

void charging(int charge)
{
	if(charge == 1)
		{
			//PIND0 = 1;
			printf("Charging\n");
		}
	else if(charge == 0)
		{
			//PIND0 = 0;
			printf("Not Charging\n");
		}

	return;
}

void discharging(int discharge)
{
	if(discharge == 1)
		{
			//PIND1 = 1;
			printf("Discharging\n");
		}
	else if(discharge == 0)
		{
			//PIND1 = 0;
			printf("Not Discharging\n");
		}

	return;
}

void loads(int call , int switches)
{
	switch(call)
	{
		case 1 :
					if(switches == 1)
					{
						printf("Load 1 on\n");
						//PIND2 = 1;
					}
					else if(switches == 0)
					{
						printf("Load 1 off\n");
						//PIND2 = 0;
					}
					break;
		case 2 :
					if(switches == 1)
					{
						printf("Load 2 on\n");
						//PIND3 = 1;
					}
					else if(switches == 0)
					{
						printf("Load 2 off\n");
						//PIND3 = 0;
					}
					break;
		case 3 :
					if(switches == 1)
					{
						printf("Load 3 on\n");
						//PIND4 = 1;
					}
					else if(switches == 0)
					{
						printf("Load 3 off\n");
						//PIND4 = 0;
					}
					break;
	}
}

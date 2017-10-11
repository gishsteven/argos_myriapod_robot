/******************************************************************************/
/* Files to Include                                                           */
/******************************************************************************/

/* Device header file */
#if defined(__XC16__)
    #include <xc.h>
#endif


#include <stdint.h>        /* Includes uint16_t definition                    */
#include <stdbool.h>       /* Includes true/false definition                  */

#include "system.h"        /* System funct/params, like osc/peripheral config */
#include "user.h"          /* User funct/params, such as InitApp              */

/******************************************************************************/
/* Global Variable Declaration                                                */
/******************************************************************************/

/* i.e. uint16_t <variable_name>; */

/******************************************************************************/
/* Main Program                                                               */
/******************************************************************************/

// Delay Function
unsigned int limit1, limit2;
void delay(limit1, limit2)
{
	unsigned int i, j;
	for (i=0; i<limit1; i++)
	{
		for (j=0; j<limit2; j++)
	}
}

int16_t main(void)
{

    /* Configure the oscillator for the device */
    ConfigureOscillator();

    /* Initialize IO ports and peripherals */
    InitApp();

    /* TODO <INSERT USER APPLICATION CODE HERE> */    
	while(1) // Allows the servo arm swing left to right and back.
	{
		P1DC1 = 120;	//1msec pulse for -90 Degrees
		delay(10,10); 	//See delay function: Every 10 'j' increments, 'i' increments by 1. (10*10 Cycles)

		P1DC1 = 240;	//2msec pulse for 90 Degrees
		delay(10,10);
	}
}

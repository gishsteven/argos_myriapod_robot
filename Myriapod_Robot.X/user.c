/******************************************************************************/
/* Files to Include                                                           */
/******************************************************************************/

/* Device header file */
#if defined(__XC16__)
    #include <xc.h>
#endif

#include <stdint.h>          /* For uint16_t definition                       */
#include <stdbool.h>         /* For true/false definition                     */
#include "user.h"            /* variables/params used by user.c               */

/******************************************************************************/
/* User Functions                                                             */
/******************************************************************************/

/* <Initialize variables in user.h and insert code for user algorithms.> */

void InitApp(void)
{
    /* TODO Initialize User Ports/Peripherals/Project here */

    /* Setup analog functionality and port direction */
PWM1CON1bits.PMOD3 = 1;
	PWM1CON1bits.PMOD2 = 1;
	PWM1CON1bits.PMOD1 = 1; 
	PWM1CON1bits.PEN3H = 0;
	PWM1CON1bits.PEN2H = 0;
	PWM1CON1bits.PEN1H = 1;			//PIN 25
	PWM1CON1bits.PEN3L = 0;
	PWM1CON1bits.PEN2L = 0;
	PWM1CON1bits.PEN1L = 0;


    /* Initialize peripherals */
    PWM1CON2bits.IUE = 0;  
	PWM1CON2bits.UDIS = 0; 

	p1tconBITS.PTEN = 1;
}

/******************************************************************************/
/* Files to Include                                                           */
/******************************************************************************/

/* Device header file */
#if defined(__XC16__)
    #include <xc.h>          /* <xc.h> allows the code insource file to directly 
                                 access compilers or device-specific features.*/
#endif              
#include <stdint.h>          /* For uint16_t definition                       */
#include <stdbool.h>         /* For true/false definition                     */
#include "system.h"          /* variables/params used by system.c             */

/******************************************************************************/
/* System Level Functions                                                     */
/*                                                                            */
/* Custom oscillator configuration functions, reset source evaluation         */
/* functions, and other non-peripheral microcontroller initialization         */
/* functions get placed in system.c.                                          */
/*                                                                            */
/******************************************************************************/

/* Refer to the device Family Reference Manual Oscillator section for
information about available oscillator configurations.  Typically
this would involve configuring the oscillator tuning register or clock
switching using the compiler's __builtin_write_OSCCON functions.
Refer to the C Compiler for PIC24 MCUs and dsPIC DSCs User Guide in the
compiler installation directory /doc folder for documentation on the
__builtin functions.*/


//OSCTUN Register = FRC Oscillator Tuning Bits
/*
111111 = Center frequency -0.375% (7.345 MHz)
?
?
?
100001 = Center frequency -11.625% (6.52 MHz)
100000 = Center frequency -12% (6.49 MHz)
011111 = Center frequency +11.625% (8.23 MHz)
011110 = Center frequency +11.25% (8.20 MHz)
?
?
?
000001 = Center frequency +0.375% (7.40 MHz)
000000 = Center frequency (7.37 MHz nominal)
OSCTUN functionality has been provided to help customers compensate for temperature effects on the
FRC frequency over a wide range of temperatures. The tuning step size is an approximation and is neither
characterized nor tested.
http://microchipdeveloper.com/8bit:intosc*/
//CLKDIV Register = Clock Divisor Register
/*
	DOZE = Processor Clock Reduction Select bits
		111 = FCY/128
		110 = FCY/64
		101 = FCY/32
		100 = FCY/16
		011 = FCY/8 (default)
		010 = FCY/4
		001 = FCY/2
		000 = FCY/1
	DOZEN = DOZE Mode Enable bit
		1 = DOZE<2:0> field specifies the ratio between the peripheral clocks and the processor clocks
		0 = Processor clock/peripheral clock ratio forced to 1:1
	FRCDIV = Internal Fast RC Oscillator Postscaler bits
		111 = FRC divide by 256
		110 = FRC divide by 64
		101 = FRC divide by 32
		100 = FRC divide by 16
		011 = FRC divide by 8
		010 = FRC divide by 4
		001 = FRC divide by 2
		000 = FRC divide by 1 (default)*/
//PWM1 Timebase Control Register (PXTCONbits)
/*
	The table is under 6-OUTPUT PWM1 REGISTER MAP
	PTEN = PWM Time Base Timer Enable bit
		1 = PWM time base is on
		0 = PWM time base is off
	PTCKPS = PWM Time Base Input Clock Prescale Select bits
		11 = PWM time base input clock period is 64 TCY (1:64 prescale)
		10 = PWM time base input clock period is 16 TCY (1:16 prescale)
		01 = PWM time base input clock period is 4 TCY (1:4 prescale)
		00 = PWM time base input clock period is TCY (1:1 prescale)
		TCY (Time for an Instruction to Complete)
	PTMOD = PWM Time Base Mode Select bits
		11 = PWM time base operates in a Continuous Up/Down Count mode with interrupts for double PWM updates
		10 = PWM time base operates in a Continuous Up/Down Count mode
		01 = PWM time base operates in Single Pulse mode
		00 = PWM time base operates in a Free-Running mode*/
//PxTMR: PWM TIMER COUNT VALUE REGISTER
/*
	PTMR = PWM Time Base Register Count Value bits
	PXTPER = PWM Time Base Period Register*/
    
    

/* TODO Add clock switching code if appropriate.  An example stub is below.   */
void ConfigureOscillator(void)
{
    
#if 0
    
    
    
        /* Disable Watch Dog Timer */
        RCONbits.SWDTEN = 0;

        /* When clock switch occurs switch to Primary Osc (HS, XT, EC) */
        __builtin_write_OSCCONH(0x02);  /* Set OSCCONH for clock switch */
        __builtin_write_OSCCONL(0x01);  /* Start clock switching */
        while(OSCCONbits.COSC != 0b011);

        /* Wait for Clock switch to occur */
        /* Wait for PLL to lock, only if PLL is needed */
        /* while(OSCCONbits.LOCK != 1); */
#endif
        
OSCTUNbits.TUN = 0;

CLKDIVbits.DOZE = 1;	// FCY = DOZE/2 = 28.8KHz (DOZE = FOSC/2)
CLKDIVbits.DOZEN = 1;
CLKDIVbits.FRCDIV = 6;	//FRCDIVN = FRC/64 = 7.37/64 = 115.15KHz = FOSC

P1TCONbits.PTEN = 0;
P1TCONbits.PTCKPS = 0; //Prescale is 1:1 so Timer1 clock = 57.6KHz = FP (115.15 /2?)
P1TCONbits.PTMOD = 0;

P1TMRbits.PTMR = 0; //Initial Value in PWM1 Counter Register
P1TPER = 1200;	//PWM1 Period Register produced 20msec PWM Period (1200/60 = 20msec)

}


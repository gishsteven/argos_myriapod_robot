/******************************************************************************/
/* Files to Include                                                           */
/******************************************************************************/

/* Device header file */
#if defined(__XC16__)
    #include <xc.h>
#endif

/******************************************************************************/
/* Configuration Bits                                                         */
/*                                                                            */
/* This is not all available configuration bits for all dsPIC devices.        */
/* Refer to the dsPIC device specific .h file in the compiler                 */
/* support\dsPIC33F\h directory for complete options specific to the device   */
/* selected.  For additional information about what hardware configurations   */
/* mean in terms of device operation, refer to the device datasheet           */
/* 'Special Features' chapter.                                                */
/*                                                                            */
/* A feature of MPLAB X is the 'Generate Source Code to Output' utility in    */
/* the Configuration Bits window.  Under Window > PIC Memory Views >          */
/* Configuration Bits, a user controllable configuration bits window is       */
/* available to Generate Configuration Bits source code which the user can    */
/* paste into this project.                                                   */
/******************************************************************************/

/* TODO Fill in your configuration bits from the config bits generator here.  */

//Configuration Register - FOSC (Frequency of Oscillator)	
/*
	FCKSM = Frequency Clock Switching Mode
        	1x = Clock switching is disabled, Fail-Safe Clock Monitor is disabled
			01 = Clock switching is enabled, Fail-Safe Clock Monitor is disabled
			00 = Clock switching is enabled, Fail-Safe Clock Monitor is enabled
        	http://www.microchip.com/forums/m430530.aspx
        	http://microchipdeveloper.com/8bit:fscm
	OSCIOFNC = OSC2 Pin Function
			Allows the option to use Clock Output(1) or GPIO(0)
			http://microchipdeveloper.com/32bit:mz-osc-posc
	POSCMD = Primary Oscillator Mode select bits
			11 = Primary oscillator disabled
			10 = HS Crystal Oscillator mode (10MHz - 20MHz) (High Speed Crystal) (Tied to OSC1 and OSC2)
			01 = XT Crystal Oscillator mode (3MHz - 10MHz) (Crystals and Ceramic Resonators)
			00 = EC (External Clock) mode (Tied to OSC1 Pin)
			http://www.microchip.com/forums/m28108.aspx
	#pragma configs = Configure settings which live in their own section of flash memory 
			that is outside of your program code. They are available as soon as the PIC 
			gets power, regardless where they are written in your code. 
			This is important because sometimes they are needed before your program is executed.*/

#pragma config FCKSM = 3 //1X = Clock switching is disabled, Fail-Safe Clock Monitor is disabled
#pragma config OSCIOFNC = 0
#pragma config POSCMD = 3

//Configuration Register - FOSCSEL (Frequency of Oscillator Select)
/*
	IESO = Two-speed Oscillator Start-up Enable bit
		1 = Start-up device with FRC, then automatically switch to the user-selected oscillator source when ready.
		0 = Start-up device with user-selected oscillator source
		Notes on FRC: http://microchipdeveloper.com/16bit:osc-frc-lpfrc
	FNOSC = Initial Oscillator Source Selection bits
		111 = Internal Fast RC (FRC) oscillator with postscaler
		110 = Internal Fast RC (FRC) oscillator with divide-by-16
		101 = LPRC oscillator
		100 = Secondary (LP) oscillator
		011 = Primary (XT, HS, EC) oscillator with PLL (Phased Locked Loop: https://en.wikipedia.org/wiki/Phase-locked_loop)
		010 = Primary (XT, HS, EC) oscillator
		001 = Internal Fast RC (FRC) oscillator with PLL
		000 = FRC oscillator*/

#pragma config IESO = 0
#pragma config FNOSC = 7

//Configuration Register - Motor Control PWM FPOR (Pulse Width Module, Power-On Reset Configuration Register)
/*
	PWMPIN = Motor Control PWM Module Pin Mode bit
		1 = PWM module pins controlled by PORT register at device Reset (tri-stated)
		0 = PWM module pins controlled by PWM module at device Reset (configured as output pins)
	HPOL = Motor Control PWM High Side Polarity bit
		1 = PWM module high side output pins have active-high output polarity
		0 = PWM module high side output pins have active-low output polarity
	LPOL = Motor Control PWM Low Side Polarity bit
		1 = PWM module low side output pins have active-high output polarity
		0 = PWM module low side output pins have active-low output polarity 
	FWDTEN = Watchdog Timer Enable bit
		1 = Watchdog Timer always enabled (LPRC oscillator cannot be disabled. 
			Clearing the SWDTEN bit in the RCON register has no effect.)
		0 = Watchdog Timer enabled/disabled by user software 
			(LPRC can be disabled by clearing the SWDTEN bit in the RCON register)
		https://en.wikipedia.org/wiki/Watchdog_timer*/

#pragma config PWMPIN = 0;
#pragma config HPOL = 1;
#pragma config LPOL = 1;
#pragma config FWDTEN = 0;
/* 
 * 
 * Authors: Onyema, Muzik, Steven, Geoff, Rajvir
 * Project Centipede 
 * 03/06/2018 @ 9:45AM
 * 
 * ---------------------------------------Centipede---Slave--------------------------------------------- 
 * 
 */


#if defined(__XC16__)
    #include <xc.h>
#endif

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <p33FJ128MC802.h>    
#include <libpic30.h>   

#pragma config FCKSM = 3     //1X = Clock switching is disabled, Fail-Safe Clock Monitor is disabled
#pragma config OSCIOFNC = 0 // OSCO pin is a general purpose I/O pin
#pragma config POSCMD = 3  // Primary oscillators are disabled
#pragma config IESO = 0     // start with a user-selected oscillator at reset
#pragma config FNOSC = 7    // select FRC oscillator with postscalar at reset
#pragma config FWDTEN = 0   // disable watchdog timer
#pragma config JTAGEN = 0   //JTAG is disabled
#pragma config PWMPIN = 0   // All Motor Control PWM outputs are active at Reset
#pragma config HPOL = 1     // Motor control HIGH outputs are active high
#pragma config LPOL = 1     // Motor control LOW outputs are active high

// Peripheral Pin Select
#define PPSUnLock       __builtin_write_OSCCONL(OSCCON & 0xBF)
#define PPSLock         __builtin_write_OSCCONL(OSCCON | 0x40)

void delay(int limit1, int limit2) {
    unsigned int i, j;
    for (i = 0; i < limit1; i++) {
        for (j = 0; j < limit2; j++) {
        }
    }
}
unsigned char SPI_Receive (){
    while(SPI1STATbits.SPITBF == 1); // Wait until the TX buffer is empty due to a prior process
    SPI1BUF = 0x00;                  // When empty, send the junk byte (0x00) to the TX buffer             
    while(SPI1STATbits.SPIRBF == 0); // As junk bits shifts out of the SDO port, valid bits are received from the SDI port
                                     // Wait until the RX buffer is full of valid data
    return SPI1BUF;                  // When full, read the valid data in RX buffer through SPI1BUF
    }
int main(int argc, char** argv) {

    // oscillator setup
    OSCTUNbits.TUN = 0;     // select FRC = 7.37MHz
    CLKDIVbits.FRCDIV = 6;    // FOSC = FRC/8 = 7.37MHz/8 and FP = FOSC/2 = 460KHz
    CLKDIVbits.DOZE = 1;	// FCY = DOZE/2 = 28.8KHz (DOZE = FOSC/2)    * DOZE = processor clock reduction bit
    CLKDIVbits.DOZEN = 1;       // DOZE mode enable bit (field specifies ratio between peripheral clock and cpu clock                                          

    SPI1CON1bits.DISSCK = 1;  // Disable the internal SPI clock 
    SPI1CON1bits.MODE16 = 1;  // Enable the 16-bit data mode
    SPI1CON1bits.SSEN = 1;    // This unit IS a slave so Slave Select pin IS used
    SPI1CON1bits.MSTEN = 0;   // Disable MASTER mode
    SPI1CON1bits.SMP = 0;     // Sample input in the middle of data output period (data is sampled at the pos edge when received at the neg edge of SCK)
    SPI1CON1bits.CKE = 1;     // SCK edge: Output data changes when SCK goes from ACTIVE to IDLE state (data is transmitted at the neg edge of SCK)
    SPI1CON1bits.CKP = 0;     // SCK polarity: IDLE is low phase and ACTIVE is high phase of SCK
    SPI1CON1bits.PPRE = 1;    // Primary SPI clock pre-scale is 1:16
    SPI1CON1bits.SPRE = 7;    // Secondary SPI clock pre-scale is 1:1 -> SCK = 460/16=29KHz
    SPI1STATbits.SPIROV = 0;  // Clear initial overflow bit in case an overflow condition in SPI1BUF
    SPI1STATbits.SPIEN = 1;   // Enable the SPI interface

    PPSUnLock;
    RPOR4bits.RP8R = 8; // RP8 is an output for SCK1              (PIN 17)
    RPINR20bits.SCK1R = 8; // RP8 is an input for SCK1               (PIN 17)
    RPINR20bits.SDI1R = 7; // RP7 is an input for SDI1               (PIN 16)
    RPOR3bits.RP6R = 9; // RP6 is an input for SS1                 (PIN 15)  
    PPSLock;

    TRISBbits.TRISB6 = 1; // Configure RB6 as an input for SS
    TRISBbits.TRISB8 = 1; // Configure RB8 as an input for SCK1 
    TRISBbits.TRISB7 = 1; // Configure RB7 as an input for SDI1    
    
    // PWM1 Timebase control register
    P1TCONbits.PTEN = 0;    // Do not enable the clock delivery to PWM1 timer yet	
    P1TCONbits.PTCKPS = 0;  // Prescale is 1:1 so Timer1 clock = 57.6KHz = FP
    P1TCONbits.PTMOD  = 0;  // PWM1 is in free-running mode

    // PWM1 counter and period
    P1TMRbits.PTMR = 0;     // Initial value in PWM1 counter register
    P1TPER = 1200;          // PWM1 period register produces 20msec PWM period   
    
    // PWM1 control register1
    PWM1CON1bits.PMOD3 = 1;                                                     // PWM I/O Pin =  Independent PWM Output Mode    
    PWM1CON1bits.PMOD2 = 1;                                                     // PWM I/O Pin =  Independent PWM Output Mode  
    PWM1CON1bits.PMOD1 = 1;                                                     // PWM I/O Pin =  Independent PWM Output Mode   
    
    PWM1CON1bits.PEN1L = 0;                                                     // PIN 26 (PWM1) //GPIO
    PWM1CON1bits.PEN1H = 1;                                                     // PIN 25 (PWM2) //PWM Output
    PWM1CON1bits.PEN2L = 1;                                                     // PIN 24 (PWM3) //PWM Output  
    PWM1CON1bits.PEN2H = 0;                                                     // PIN 23 (PWM4) //GPIO
    PWM1CON1bits.PEN3L = 0;                                                     // PIN 22 (PWM5) //GPIO
    PWM1CON1bits.PEN3H = 0;                                                     // PIN 21 (PWM6) //GPIO

    // PWM1 control register2
    PWM1CON2bits.IUE = 0; // Updates are synchronized with timebase
    PWM1CON2bits.UDIS = 0; // Updates from period and duty cycle registers are enabled

    // Enable timebase control register
    P1TCONbits.PTEN = 1; // Enable the clock delivery to PWM1 timer

    /* Angle Values (CLOCKWISE)
     * 280   =   0 Degrees 
     * 227.5 =  45 Degrees
     * 175   =  90 Degrees
     * 122.5 = 135 Degrees
     * 70    = 180 Degrees
     */    

    while (1) {

        if (SPI_Receive() == 10) {
            while (1) {
                // This should allow servo on PIN 24 to mirror PIN 25. 
                P1DC1 = 280;
                PDC2 = 175;
                delay(200, 200);
                P1DC1 = 70;
                PDC2 = 70;
                delay(200, 200);
                P1DC1 = 175;
                PDC2 = 280;
                delay(200, 200);
            }
        }

    }
    return (EXIT_SUCCESS);
}

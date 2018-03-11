	/* 
 * 
 * Authors: Onyema, Muzik, Steven, Geoff, Rajvir
 * Project Centipede 
 * 
 * 
 * ---------------------------------------Master---Main--------------------------------------------- 
 * 
 */

#if defined(__XC16__)
    #include <xc.h>
#endif

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <p33FJ128MC802.h>     

    #pragma config FCKSM = 3     //1X = Clock switching is disabled, Fail-Safe Clock Monitor is disabled (CORRECT))
    #pragma config OSCIOFNC = 0 // OSCO pin is a general purpose I/O pin (CORRECT)
    #pragma config POSCMD = 3  // Primary oscillators are disabled (CORRECT)
    #pragma config IESO = 0     // start with a user-selected oscillator at reset (CORRECT)
    #pragma config FNOSC = 7    // select FRC oscillator with postscalar at reset (CORRECT)
    #pragma config FWDTEN = 0   // wacthdog timer set by software (CORRECT)
    #pragma config JTAGEN = 0   //JTAG is disabled (CORRECT)

#define PPSUnLock       __builtin_write_OSCCONL(OSCCON & 0xBF)
#define PPSLock         __builtin_write_OSCCONL(OSCCON | 0x40)

void delay(limit1, limit2){
        unsigned int i, j;
        for (i=0; i<limit1; i++){
            for (j=0; j<limit2; j++){
            }
        }
}
unsigned char SPI_Transmit (unsigned char TxValue){
    while (SPI1STATbits.SPITBF == 1); // Wait until the TX buffer is empty due to a prior process
    SPI1BUF = TxValue;                // When empty, send the byte to the TX buffer             
    while (SPI1STATbits.SPIRBF == 0); // As valid bits shifts out of the SDO port, junk bits are received from the SDI port
                                      // Wait until the RX buffer is full of junk data
    return SPI1BUF;                   // When full, read the junk data in RX buffer through SPI1BUF
}

int main(int argc, char** argv) {
   
    // Oscillator value set up
    OSCTUNbits.TUN = 0;       // select FRC = 7.37MHz (center frequency) (CORRECT)
    CLKDIVbits.FRCDIV = 3;    // FOSC = FRC/8 = 7.37MHz/8 and FP = FOSC/2 = 460KHz (CORRECT)

    // SPI setup -> MASTER mode sending 8 bits
    SPI1CON1bits.DISSCK = 0;  // Enable the internal SPI clock (CORRECT)
    SPI1CON1bits.DISSDO = 0;  // Enable the SPI data output, SDO (CORRECT)    
    SPI1CON1bits.MODE16 = 1;  // Enable the 16-bit data mode (CORRECT)
    SPI1CON1bits.SSEN = 0;    // This unit is not a slave so Slave Select pin is not used (CORRECT)
    SPI1CON1bits.MSTEN = 1;   // Enable MASTER mode (CORRECT)
    SPI1CON1bits.SMP = 0;     // Sample input in the middle of data output period (data is sampled at the pos edge when received at the neg edge of SCK (CORRECT)
    SPI1CON1bits.CKE = 1;     // SCK edge: Output data changes when SCK goes from ACTIVE to IDLE state (data is transmitted at the neg edge of SCK) (CORRECT)
    SPI1CON1bits.CKP = 0;     // SCK polarity: IDLE is low phase and ACTIVE is high phase of SCK (CORRECT)
    SPI1CON1bits.PPRE = 1;    // Primary SPI clock pre-scale is 1:16 (CORRECT)
    SPI1CON1bits.SPRE = 7;    // Secondary SPI clock pre-scale is 1:1 -> SCK = 460/16=29KHz (CORRECT)
    SPI1STATbits.SPIROV = 0;  // Clear initial overflow bit in case an overflow condition in SPI1BUF (CORRECT)
    SPI1STATbits.SPIEN = 1;   // Enable the SPI interface (CORRECT)

    PPSUnLock;
        RPOR4bits.RP8R = 8;       // RP8 is an output for SCK1              (PIN 17)
        RPINR20bits.SCK1R = 8;    // RP8 is an input for SCK1               (PIN 17)
        RPOR4bits.RP9R = 7;       // RP9 is an output for SDO1              (PIN 18)
        RPOR3bits.RP6R = 9;     // RP6 is an output for SS1                 (PIN 15)  
    PPSLock;

    TRISBbits.TRISB8 = 0;     // Configure RB8 as an output for SCK1    (PIN 17)
    TRISBbits.TRISB9 = 0;     // Configure RB9 as an output SDO1        (PIN 18)
    TRISBbits.TRISB6 = 0;     // Configure RB6 as an output SS1        (for testing)

    
    TRISBbits.TRISB12 = 0;                                                       // PIN 23 (RB12 OUTPUT for SS0)
    LATBbits.LATB12 = 1;                                                          
    TRISBbits.TRISB13 = 0;                                                       // PIN 24 (RB12 OUTPUT for SS1)
    LATBbits.LATB13 = 1;                                                        
    TRISBbits.TRISB14 = 0;                                                       // PIN 25 (RB12 OUTPUT for SS2)
    LATBbits.LATB14 = 1;                      

    unsigned char junk, txData; 
    
    txData = 'F';
    
    while(1){  
          LATBbits.LATB6 = 0;      // enable the GPIO slave select
          junk = SPI_Transmit (10);      // Send a data byte          
          LATBbits.LATB6 = 1;      // disable the GPIO slave select
          delay(200, 200);
          delay(200, 200);
          delay(200, 200);
          delay(200, 200);
    }  
    return (EXIT_SUCCESS);
}
				
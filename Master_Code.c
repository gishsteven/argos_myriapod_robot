/*******************************************************************************
 * File:   Myriapod_Master_main.c
 * Author: Authors: Onyema, Muzik, Steven, Geoff, Rajvir
 * 
 *******************************************************************************
 * Master Main
 ******************************************************************************/

#include <xc.h>   

#pragma config FCKSM = 3     //1X = Clock switching is disabled, Fail-Safe Clock Monitor is disabled  )
#pragma config OSCIOFNC = 0 // OSCO pin is a general purpose I/O pin  
#pragma config POSCMD = 3  // Primary oscillators are disabled  
#pragma config IESO = 0     // start with a user-selected oscillator at reset  
#pragma config FNOSC = 7    // select FRC oscillator with postscalar at reset  
#pragma config FWDTEN = 0   // wacthdog timer set by software 
#pragma config JTAGEN = 0   //JTAG is disabled  

#define PPSUnLock       __builtin_write_OSCCONL(OSCCON & 0xBF)
#define PPSLock         __builtin_write_OSCCONL(OSCCON | 0x40)

void delay(int limit1,int limit2){
        unsigned int i, j;
        for (i=0; i<limit1; i++){
            for (j=0; j<limit2; j++){
            }
        }
}

unsigned char SPI_Transmit1 (unsigned char TxValue){
    LATAbits.LATA0 = 0;                 // enable the GPIO SS1
    while (SPI1STATbits.SPITBF == 1); // Wait until the TX buffer is empty due to a prior process
    SPI1BUF = TxValue;                // When empty, send the byte to the TX buffer             
    while (SPI1STATbits.SPIRBF == 0); // As valid bits shifts out of the SDO port, junk bits are received from the SDI port
                                      // Wait until the RX buffer is full of junk data
    LATAbits.LATA0 = 1;         // disable the GPIO SS1
    return SPI1BUF;                   // When full, read the junk data in RX buffer through SPI1BUF
}

unsigned char SPI_Transmit2 (unsigned char TxValue){
    LATAbits.LATA1 = 0;        // enable the GPIO SS2
    while (SPI2STATbits.SPITBF == 1); // Wait until the TX buffer is empty due to a prior process
    SPI2BUF = TxValue;                // When empty, send the byte to the TX buffer             
    while (SPI2STATbits.SPIRBF == 0); // As valid bits shifts out of the SDO port, junk bits are received from the SDI port
                                      // Wait until the RX buffer is full of junk data
    LATAbits.LATA1 = 1;        // disable the GPIO SS2
    return SPI2BUF;                   // When full, read the junk data in RX buffer through SPI1BUF
}
//Add SPI2STATbits.

void init_SPI() {
//SPI Setup
    SPI1CON1bits.DISSCK = 0;    // Enable the internal SPI clock
    SPI1CON1bits.DISSDO = 0;    // Enable the SPI data output, SDO
    SPI1CON1bits.MODE16 = 1;    // Enable the 16-bit data mode
    SPI1CON1bits.SSEN = 0;      // This unit is not a slave so Slave Select pin is not used.
    SPI1CON1bits.MSTEN = 1;     // Enable MASTER mode
    SPI1CON1bits.SMP = 0;       // Sample input in the middle of data output period (data is sampled at the pos edge when received at the neg edge of SCK
    SPI1CON1bits.CKE = 1;       // SCK edge: Output data changes when SCK goes from ACTIVE to IDLE state (data is transmitted at the neg edge of SCK)
    SPI1CON1bits.CKP = 0;       // SCK polarity: IDLE is low phase and ACTIVE is high phase of SCK
    SPI1CON1bits.PPRE = 1;      // Primary SPI clock pre-scale is 1:16
    SPI1CON1bits.SPRE = 7;      // Secondary SPI clock pre-scale is 1:1 -> SCK = 460/16=29KHz
    SPI1STATbits.SPIROV = 0;    // Clear initial overflow bit in case an overflow condition in SPI1BUF
    SPI1STATbits.SPIEN = 1;     // Enable the SPI interface

    SPI2CON1bits.DISSCK = 0;    // Enable the internal SPI clock
    SPI2CON1bits.DISSDO = 0;    // Enable the SPI data output, SDO
    SPI2CON1bits.MODE16 = 1;    // Enable the 16-bit data mode
    SPI2CON1bits.SSEN = 0;      // This unit is not a slave so Slave Select pin is not used.
    SPI2CON1bits.MSTEN = 1;     // Enable MASTER mode
    SPI2CON1bits.SMP = 0;       // Sample input in the middle of data output period (data is sampled at the pos edge when received at the neg edge of SCK
    SPI2CON1bits.CKE = 1;       // SCK edge: Output data changes when SCK goes from ACTIVE to IDLE state (data is transmitted at the neg edge of SCK)
    SPI2CON1bits.CKP = 0;       // SCK polarity: IDLE is low phase and ACTIVE is high phase of SCK
    SPI2CON1bits.PPRE = 1;      // Primary SPI clock pre-scale is 1:16
    SPI2CON1bits.SPRE = 7;      // Secondary SPI clock pre-scale is 1:1 -> SCK = 460/16=29KHz
    SPI2STATbits.SPIROV = 0;    // Clear initial overflow bit in case an overflow condition in SPI1BUF
    SPI2STATbits.SPIEN = 1;     // Enable the SPI interface


//------------------------------------------------------------------------------
//Peripheral Pin Select Configuration

    RPOR0bits.RP0R = 8;         // PIN 04 (RP0 - SCK1)
    RPOR0bits.RP1R = 11;        // PIN 05 (RP1 - SCK2)
    RPOR1bits.RP2R = 7;         // PIN 06 (RP2 - SDO1)
    RPOR1bits.RP3R = 10;        // PIN 07 (RP3 - SD02)

//------------------------------------------------------------------------------
//Pin Input/Output Configuration

    //GPIO used for Slave Select
    TRISAbits.TRISA0 = 0;       // PIN 02 (RA0 - OUTPUT for SS1)
    LATAbits.LATA0 = 1;         // Disable SS1
    TRISAbits.TRISA1 = 0;       // PIN 03 (RA1 - OUTPUT for SS2)
    LATAbits.LATA1 = 1;         // Disable SS2

    TRISBbits.TRISB0 = 0;       // PIN 04 (RB0 - OUTPUT for SCK1)    
    TRISBbits.TRISB1 = 0;       // PIN 05 (RB1 - OUTPUT for SCK2)
    TRISBbits.TRISB2 = 0;       // PIN 06 (RB2 - OUTPUT for SD01)
    TRISBbits.TRISB3 = 0;       // PIN 07 (RB3 - OUTPUT for SDO2)   
}

int main(void) {
   
    // Oscillator value set up
    OSCTUNbits.TUN = 0;       // select FRC = 7.37MHz (center frequency)  
    CLKDIVbits.FRCDIV = 3;    // FOSC = FRC/8 = 7.37MHz/8 and FP = FOSC/2 = 460KHz  

    //TODO: Testing PPS unlock at an earlier stage in the code, check if this actually works. 
    PPSUnLock;
    init_SPI();
    PPSLock;
    
    while (1) {
        SPI_Transmit1('L');  // Send a data byte          
        SPI_Transmit2('R');  // Send a data byte           
        delay(10, 10);
    }  
    return (1);
}

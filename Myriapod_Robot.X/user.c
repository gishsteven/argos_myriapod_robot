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


/* Peripheral Pin Select compiler declarations  
	The SPIx module inputs must be configured as digital pins by setting the corresponding bits in
	the ADxPCFG registers, or by clearing the bits in the ANSELx or ANSx registers with respect to
	a particular device. If the device has a Peripheral Pin Select (PPS) feature, the SCKx pin must
	be mapped as both input and output in Master mode.
	https://www.youtube.com/watch?v=KZcL8cjqphE*/
#define PPSUnLock       __builtin_write_OSCCONL(OSCCON & 0xBF)
#define PPSLock         __builtin_write_OSCCONL(OSCCON | 0x40)
/******************************************************************************/
/* User Functions                                                             */
/******************************************************************************/

/* Register setup functions
	1. Lower SS for Instruction Delivery
	2. Send Instruction
	3. Send Address of Register
	4. Send Data Byte
	5. Raise SS for Instruction Completion. */
//	SPI Transmit and Receive Functions
/* 
	SPITBF: SPIx Transmit Buffer Full Status bit
		1 = Transmit has not yet started, SPIxTXB buffer is full
		0 = Transmit has started, SPIxTXB buffer is empty
	SPIRBF: SPIx Receive Buffer Full Status bit
		1 = Receive complete, SPIxRXB is full
		0 = Receive is incomplete, SPIxRXB is empty*/

/*
unsigned char SPI_Transmit (unsigned char TxValue)
    {
    while (SPI1STATbits.SPITBF == 1); // Wait until the TX buffer is empty due to a prior process
    SPI1BUF = TxValue;                // When empty, send the byte to the TX buffer             

    while (SPI1STATbits.SPIRBF == 0); // As valid bits shifts out of the SDO port, junk bits are received from the SDI port
                                      // Wait until the RX buffer is full of junk data
    return SPI1BUF;                   // When full, read the junk data in RX buffer through SPI1BUF
    }

unsigned char SPI_Receive ()
    {
    while(SPI1STATbits.SPITBF == 1); // Wait until the TX buffer is empty due to a prior process
    SPI1BUF = 0x00;                  // When empty, send the junk byte (0x00) to the TX buffer             
    while(SPI1STATbits.SPIRBF == 0); // As junk bits shifts out of the SDO port, valid bits are received from the SDI port
                                     // Wait until the RX buffer is full of valid data
    return SPI1BUF;                  // When full, read the valid data in RX buffer through SPI1BUF
    }



void Write_Threshold_Activity_LReg (unsigned char DataByte)
    {    
    LATBbits.LATB6 = 0;      // Lower SS for instruction delivery
    SPI_Transmit (0x0A);     // Send the Write Register instruction
    SPI_Transmit (0x20);     // Send the address of Write Threshold Activity Low Register
    SPI_Transmit (DataByte); // Send a data byte
    LATBbits.LATB6 = 1;      // Raise SS for instruction completion
    }

void Write_Threshold_Activity_HReg (unsigned char DataByte)
    {    
    LATBbits.LATB6 = 0;      // Lower SS for instruction delivery
    SPI_Transmit (0x0A);     // Send the Write Register instruction
    SPI_Transmit (0x21);     // Send the address of Write Threshold Activity High Register
    SPI_Transmit (DataByte); // Send a data byte
    LATBbits.LATB6 = 1;      // Raise SS for instruction completion
    }

void Write_Threshold_Inactivity_LReg (unsigned char DataByte)
    {    
    LATBbits.LATB6 = 0;      // Lower SS for instruction delivery
    SPI_Transmit (0x0A);     // Send the Write Register instruction
    SPI_Transmit (0x23);     // Send the address of Write Threshold Inactivity Low Register
    SPI_Transmit (DataByte); // Send a data byte
    LATBbits.LATB6 = 1;      // Raise SS for instruction completion
    }

void Write_Threshold_Inactivity_HReg (unsigned char DataByte)
    {    
    LATBbits.LATB6 = 0;      // Lower SS for instruction delivery
    SPI_Transmit (0x0A);     // Send the Write Register instruction
    SPI_Transmit (0x24);     // Send the address of Write Threshold Inactivity High Register
    SPI_Transmit (DataByte); // Send a data byte
    LATBbits.LATB6 = 1;      // Raise SS for instruction completion
    }

void Write_Time_Activity_Reg (unsigned char DataByte)
    {    
    LATBbits.LATB6 = 0;      // Lower SS for instruction delivery
    SPI_Transmit (0x0A);     // Send the Write Register instruction
    SPI_Transmit (0x22);     // Send the address of Time Activity Register
    SPI_Transmit (DataByte); // Send a data byte
    LATBbits.LATB6 = 1;      // Raise SS for instruction completion
    }

void Write_Time_Inactivity_LReg (unsigned char DataByte)
    {    
    LATBbits.LATB6 = 0;      // Lower SS for instruction delivery
    SPI_Transmit (0x0A);     // Send the Write Register instruction
    SPI_Transmit (0x25);     // Send the address of Time Inactivity Low Register
    SPI_Transmit (DataByte); // Send a data byte
    LATBbits.LATB6 = 1;      // Raise SS for instruction completion
    }

void Write_Time_Inactivity_HReg (unsigned char DataByte)
    {    
    LATBbits.LATB6 = 0;      // Lower SS for instruction delivery
    SPI_Transmit (0x0A);     // Send the Write Register instruction
    SPI_Transmit (0x26);     // Send the address of Time Inactivity High Register
    SPI_Transmit (DataByte); // Send a data byte
    LATBbits.LATB6 = 1;      // Raise SS for instruction completion
    }

void Write_Active_Inactive_Control_Reg (unsigned char DataByte)
    {    
    LATBbits.LATB6 = 0;      // Lower SS for instruction delivery
    SPI_Transmit (0x0A);     // Send the Write Register instruction
    SPI_Transmit (0x27);     // Send the address of Active-Inactive Control Register
    SPI_Transmit (DataByte); // Send a data byte
    LATBbits.LATB6 = 1;      // Raise SS for instruction completion
    }

void Write_Filter_Control_Reg (unsigned char DataByte)
    {    
    LATBbits.LATB6 = 0;      // Lower SS for instruction delivery
    SPI_Transmit (0x0A);     // Send the Write Register instruction
    SPI_Transmit (0x2C);     // Send the address of Filter Control Register
    SPI_Transmit (DataByte); // Send a data byte
    LATBbits.LATB6 = 1;      // Raise SS for instruction completion
    }

void Write_Power_Control_Reg (unsigned char DataByte)
    {    
    LATBbits.LATB6 = 0;      // Lower SS for instruction delivery
    SPI_Transmit (0x0A);     // Send the Write Register instruction
    SPI_Transmit (0x2D);     // Send the address of Power Control Register
    SPI_Transmit (DataByte); // Send a data byte
    LATBbits.LATB6 = 1;      // Raise SS for instruction completion
    }

unsigned char Read_Status_Reg ()
    {    
    LATBbits.LATB6 = 0;       // Lower SS for instruction delivery
    SPI_Transmit (0x0B);      // Send the Read Register instruction
    SPI_Transmit (0x0B);      // Send the address of Status Register
    unsigned char status = SPI_Receive ();  // Read the status register
    LATBbits.LATB6 = 1;       // Raise SS for instruction completion
    return status;
    }

signed char Read_8bit_XData ()
    {    
    LATBbits.LATB6 = 0;      // Lower SS for instruction delivery
    SPI_Transmit (0x0B);     // Send the Read Register instruction
    SPI_Transmit (0x08);     // Send the address of XData Register
    signed char XData = SPI_Receive ();  // Read the SPI1BUF register
    LATBbits.LATB6 = 1;      // Raise SS for instruction completion
    return XData;
    }

signed char Read_8bit_YData ()
    {    
    LATBbits.LATB6 = 0;      // Lower SS for instruction delivery
    SPI_Transmit (0x0B);     // Send the Read Register instruction
    SPI_Transmit (0x09);     // Send the address of YData Register
    signed char YData = SPI_Receive ();  // Read the SPI1BUF register
    LATBbits.LATB6 = 1;      // Raise SS for instruction completion
    return YData;
    }

signed char Read_8bit_ZData ()
    {    
    LATBbits.LATB6 = 0;      // Lower SS for instruction delivery
    SPI_Transmit (0x0B);     // Send the Read Register instruction
    SPI_Transmit (0x0A);     // Send the address of ZData Register
    signed char ZData = SPI_Receive ();  // Read the SPI1BUF register
    LATBbits.LATB6 = 1;      // Raise SS for instruction completion
    return ZData;
    }

*/

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

	P1TCONbits.PTEN = 1;
    
    
    // SPI setup -> MASTER mode sending 8 bits

    /*SPI1CON Register Settings
        DISSCK =
        DISSDO = 
        MODE16 = 
        SMP    = 
        CKE    =
        CKP    =
        MSTEN  = 
        SPIROV =
        SPIEN  =
        PPRE   =
        SPRE   = 
        SSEN   = 
        */
    SPI1CON1bits.DISSCK = 0;  // Enable the internal SPI clock
    SPI1CON1bits.DISSDO = 0;  // Enable the SPI data output, SDO
    SPI1CON1bits.MODE16 = 0;  // Enable the 8-bit data mode
    SPI1CON1bits.SMP = 0;     // Sample input in the middle of data output period (data is sampled at the pos edge when received at the neg edge of SCK)

    SPI1CON1bits.CKE = 1;     // SCK edge: Output data changes when SCK goes from ACTIVE to IDLE state 
                              //(data is transmitted at the neg edge of SCK)

    SPI1CON1bits.CKP = 0;     // SCK polarity: IDLE is low phase and ACTIVE is high phase of SCK

    SPI1CON1bits.MSTEN = 1;   // Enable MASTER mode
    SPI1STATbits.SPIROV = 0;  // Clear initial overflow bit in case an overflow condition in SPI1BUF
    SPI1STATbits.SPIEN = 1;   // Enable the SPI interface

    SPI1CON1bits.PPRE = 1;    // Primary SPI clock pre-scale is 1:16
    SPI1CON1bits.SPRE = 7;    // Secondary SPI clock pre-scale is 1:1 -> SCK = 460/16=29KHz
    SPI1CON1bits.SSEN = 0;    // This unit is not a slave so Slave Select pin is not used 

    /* Peripheral Pin Select (PPS) setup
        PPOR4bits 	= 
        RPINR20BITS =
        RPOR3bits	= 
        PPSLock 	=
        PPSUnlock 	=
    */

    PPSUnLock;
    RPOR4bits.RP8R = 8;       // RP8 is an output for SCK1
    RPINR20bits.SCK1R = 8;    // RP8 is an input for SCK1
    RPOR4bits.RP9R = 7;       // RP9 is an output for SDO1
    RPINR20bits.SDI1R = 7;    // RP7 is an input for SDI1
    RPOR3bits.RP6R = 9;       // RP6 is an output for SS1
    PPSLock;

    /* I/O setup 
        TRISBbits	= 
            0: Output
            1: Input
    */
    TRISBbits.TRISB6 = 0;     // Configure RB6 as an output for SS
    TRISBbits.TRISB8 = 0;     // Configure RB8 as an output for SCK1 
    TRISBbits.TRISB9 = 0;     // Configure RB9 as an output SDO1
    TRISBbits.TRISB7 = 1;     // Configure RB7 as an input for SDI1
    
}




/*******************************************************************************
 * File:   Myriapod_Master_main.c
 * Author: Authors: Onyema, Andrew, Steven, Geoffrey, Rajvir
 * 
 *******************************************************************************
 * Master Main
 ******************************************************************************/

#include <xc.h>
#include <stdio.h>
#include <math.h>
#include <stdbool.h>

#pragma config FCKSM = 3    // Clock switching is disabled, Fail-Safe Clock Monitor is disabled
#pragma config OSCIOFNC = 0 // OSCO pin is a general purpose I/O pin  
#pragma config POSCMD = 3   // Primary oscillators are disabled  
#pragma config IESO = 0     // start with a user-selected oscillator at reset  
#pragma config FNOSC = 7    // select FRC oscillator with postscalar at reset  
#pragma config FWDTEN = 0   // wacthdog timer set by software 
#pragma config JTAGEN = 0   //JTAG is disabled  

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

//enum ServoFSR90{      //Calibrations for FSR90 Continuous Servo 
//    //Clockwise X < 170
//    //C_Clockwise X > 170
//    Midpoint   =  170,
//    R_Slow  =  150,
//    R_Medium  =  130,
//    R_Fast =  100,
//    L_Slow =  190,
//    L_Medium  =  210,
//    L_Fast = 240,
//};

enum ServoTurnigy1370A {    
    Counter_Clockwise = 355, //Note: Servo moving counter-clockwise causes jitter. Might be a hardware issue. Value 348 is the fastest possible CC movement. 
    Degree_0   =  280, 
    Degree_45  =  227,
    Degree_90  =  175,
    Degree_135 =  122,
    Degree_180 =  70,
    Clockwise  =  1,
    OC_Counter_Clockwise = 185, 
    OC_Clockwise = 30
};

void forwardMovement(){
    
        P1DC1 = Counter_Clockwise;      //PWM1_1 & PWM1_2 [LEFT]
        P1DC2 = Clockwise;              //PWM1_3 & PWM1_4 [RIGHT]
        P1DC3 = Counter_Clockwise;      //PWM1_5 & PWM1_6 [LEFT]
        P2DC1 = Clockwise;              //PWM2_1 & PWM2_2 [RIGHT]
        OC1RS = OC_Counter_Clockwise;   //OC1 [LEFT]
        OC2RS = OC_Clockwise;           //OC2 [RIGHT]
        OC3RS = OC_Counter_Clockwise;   //OC3 [LEFT]
        OC4RS = OC_Clockwise;           //OC4 [RIGHT]
}

void stopMovement(){
       
        P1DC1 = 0;
        P1DC2 = 0;
        P1DC3 = 0;
        P2DC1 = 0;
        OC1RS = 0;
        OC2RS = 0;
        OC3RS = 0;
        OC4RS = 0;
        
    //TODO: Add code to implement Backwards, Left, annd Right Movement
}

void backMovement(){
    
        P1DC1 = Clockwise;                      //PWM1_1 & PWM1_2 [REVERSE LEFT]
        P1DC2 = Counter_Clockwise;              //PWM1_3 & PWM1_4 [REVERSE RIGHT]
        P1DC3 = Clockwise;                      //PWM1_5 & PWM1_6 [REVERSE LEFT]
        P2DC1 = Counter_Clockwise;              //PWM2_1 & PWM2_2 [REVERSE RIGHT]
        OC1RS = OC_Clockwise;                   //OC1 [REVERSE LEFT]
        OC2RS = OC_Counter_Clockwise;           //OC2 [REVERSE RIGHT]
        OC3RS = OC_Clockwise;                   //OC3 [REVERSE LEFT]
        OC4RS = OC_Counter_Clockwise;           //OC4 [REVERSE RIGHT]
}

void leftMovement(){
        P1DC1 = Clockwise               //Rotate
        P1DC2 = Clockwise;              //PWM1_3 & PWM1_4 [RIGHT]
        P1DC3 = Counter_Clockwise;      //PWM1_5 & PWM1_6 [LEFT]
        P2DC1 = Clockwise;              //PWM2_1 & PWM2_2 [RIGHT]
        OC1RS = OC_Counter_Clockwise;   //OC1 [LEFT]
        OC2RS = OC_Clockwise;           //OC2 [RIGHT]
        OC3RS = OC_Counter_Clockwise;   //OC3 [LEFT]
        OC4RS = OC_Clockwise;           //OC4 [RIGHT]
        
        delay(50,100);
        P1DC1 = 0;
}

void rightMovement(){
        P1DC1 = Counter_Clockwise;      //PWM1_1 & PWM1_2 [LEFT]
        P1DC2 = Counter_Clockwise;      //Rotate
        P1DC3 = Counter_Clockwise;      //PWM1_5 & PWM1_6 [LEFT]
        P2DC1 = Clockwise;              //PWM2_1 & PWM2_2 [RIGHT]
        OC1RS = OC_Counter_Clockwise;   //OC1 [LEFT]
        OC2RS = OC_Clockwise;           //OC2 [RIGHT]
        OC3RS = OC_Counter_Clockwise;   //OC3 [LEFT]
        OC4RS = OC_Clockwise;           //OC4 [RIGHT]
        
        delay(50,100);
        P1DC2 = 0;
}

bool Gyroscope_Test(){
    //TODO: If value == misaligned;
//    if (value == misaligned){
//        /*Notes: Hardware faults cause the robot to naturally turn left,
//         *       this function stops two (2) PWM's on the front right to stop,
//         *       allowing the robot to realign itself. */
//        P1DC2 = 0;
//        delay(100,100); //TODO: Re-calibrate this timing often!
//    }
        unsigned int ZH_unsigned, ZL_unsigned, Z_unsigned;
        signed int Z_signed;
        double Z, DSP;
    
        while ((Read_Status_Reg () & 0x04) == 0x00)  // Unless X, Y, Z data are all ready, keep reading the status register
        Read_Status_Reg ();

        Read_8bit_ZDataH ();
        Read_8bit_ZDataL ();

        ZH_unsigned = Read_8bit_ZDataH ();
        ZL_unsigned = Read_8bit_ZDataL ();

        Z_unsigned = (ZH_unsigned << 8) | ZL_unsigned;
        Z_signed = Z_unsigned;
        Z = Z_signed;
        DSP = Z*(250.0/32767.0);
    
        return DSP;
    /*
     * Notes:
     * A while loop would not work as it will stop the IR_Sensor (we do not have an interrupt!)
     * IR_Sensor will not work for delay(100,100) because of this function)
     */
}

unsigned char SPI_Transmit1(unsigned char TxValue) { 
//SLAVE 1 AND SLAVE 2 SPI (DAISY CHAINED)
    LATAbits.LATA0 = 0;                     // enable the GPIO SS1
    while (SPI1STATbits.SPITBF == 1);       // Wait until the TX buffer is empty due to a prior process
    SPI1BUF = TxValue;                      // When empty, send the byte to the TX buffer             
    while (SPI1STATbits.SPIRBF == 0);       // As valid bits shifts out of the SDO port, junk bits are received from the SDI port
    // Wait until the RX buffer is full of junk data
    LATAbits.LATA0 = 1;                     // disable the GPIO SS1
    return SPI1BUF;                         // When full, read the junk data in RX buffer through SPI1BUF
}

unsigned char SPI_Transmit2(unsigned char TxValue) { 
//GYROSCOPE
    LATAbits.LATA1 = 0;                     // enable the GPIO SS2
    while (SPI2STATbits.SPITBF == 1);       // Wait until the TX buffer is empty due to a prior process
    SPI2BUF = TxValue;                      // When empty, send the byte to the TX buffer             
    while (SPI2STATbits.SPIRBF == 0);       // As valid bits shifts out of the SDO port, junk bits are received from the SDI port
    // Wait until the RX buffer is full of junk data
    LATAbits.LATA1 = 1;                     // disable the GPIO SS2
    return SPI2BUF;                         // When full, read the junk data in RX buffer through SPI1BUF
}

unsigned char SPI_Receive2() {              
//GYROSCOPE SPI
    while (SPI2STATbits.SPITBF == 1);       // Wait until the TX buffer is empty due to a prior process
    SPI1BUF = 0x00;                         // When empty, send the junk byte (0x00) to the TX buffer             
    while (SPI2STATbits.SPIRBF == 0);       // As junk bits shifts out of the SDO port, valid bits are received from the SDI port
    // Wait until the RX buffer is full of valid data
    return SPI2BUF;                         // When full, read the valid data in RX buffer through SPI1BUF
}

void Write_Control_Reg1 (unsigned char DataByte) // ODR, BW and X, Y, Z axes enables
    {    
    LATBbits.LATB6 = 0;      // Lower SS for instruction delivery
    SPI_Transmit2 (0x20);     // Send a single write instruction to Control Reg1 address
    SPI_Transmit2 (DataByte); // Send a data byte
    LATBbits.LATB6 = 1;      // Raise SS for instruction completion
    }

void Write_Control_Reg2 (unsigned char DataByte) // High pass filter configuration
    {    
    LATBbits.LATB6 = 0;       // Lower SS for instruction delivery
    SPI_Transmit2 (0x21);     // Send a single write instruction to Control Reg2 address
    SPI_Transmit2 (DataByte); // Send a data byte
    LATBbits.LATB6 = 1;       // Raise SS for instruction completion
    }

void Write_Control_Reg3 (unsigned char DataByte) // Interrupt configuration
    {    
    LATBbits.LATB6 = 0;       // Lower SS for instruction delivery
    SPI_Transmit2 (0x22);     // Send a single write instruction to Control Reg3 address
    SPI_Transmit2 (DataByte); // Send a data byte
    LATBbits.LATB6 = 1;       // Raise SS for instruction completion
    }

void Write_Control_Reg4 (unsigned char DataByte) // Mode configuration
    {    
    LATBbits.LATB6 = 0;       // Lower SS for instruction delivery
    SPI_Transmit2 (0x23);     // Send a single write instruction to Control Reg4 address
    SPI_Transmit2 (DataByte); // Send a data byte
    LATBbits.LATB6 = 1;       // Raise SS for instruction completion
    }

void Write_Control_Reg5 (unsigned char DataByte) // Output port configuration
    {    
    LATBbits.LATB6 = 0;       // Lower SS for instruction delivery
    SPI_Transmit2 (0x24);     // Send a single write instruction to Control Reg5 address
    SPI_Transmit2 (DataByte); // Send a data byte
    LATBbits.LATB6 = 1;       // Raise SS for instruction completion
    }

void Write_FIFO_Control_Reg (unsigned char DataByte) // FIFO control
    {    
    LATBbits.LATB6 = 0;       // Lower SS for instruction delivery
    SPI_Transmit2 (0x2E);     // Send a single write instruction to Control Reg5 address
    SPI_Transmit2 (DataByte); // Send a data byte
    LATBbits.LATB6 = 1;       // Raise SS for instruction completion
    }

unsigned char Read_Status_Reg ()
    {    
    LATBbits.LATB6 = 0;       // Lower SS for instruction delivery
    SPI_Transmit (0xA7);      // Send a single read instruction from SR address
    unsigned char status = SPI_Receive ();  // Read the status register
    LATBbits.LATB6 = 1;       // Raise SS for instruction completion
    return status;
    }

unsigned char Read_8bit_ZDataH ()
    {    
    LATBbits.LATB6 = 0;      // Lower SS for instruction delivery
    SPI_Transmit (0xAD);     // Send a single read instruction from ZDataH address
    unsigned char ZDataH = SPI_Receive ();  // Read the SPI1BUF register
    LATBbits.LATB6 = 1;      // Raise SS for instruction completion
    return ZDataH;
    }

unsigned char Read_8bit_ZDataL ()
    {    
    LATBbits.LATB6 = 0;      // Lower SS for instruction delivery
    SPI_Transmit (0xAC);     // Send a single read instruction from ZDataL address
    unsigned char ZDataL = SPI_Receive ();  // Read the SPI1BUF register
    LATBbits.LATB6 = 1;      // Raise SS for instruction completion
    return ZDataL;
    }


void init_Oscillator(){
// Oscillator value set up
    OSCTUNbits.TUN = 0;         // select FRC = 7.37MHz (center frequency)  
    CLKDIVbits.FRCDIV = 6;      // FOSC = FRC/8 = 7.37MHz/8 and FP = FOSC/2 = 460KHz  
    CLKDIVbits.DOZE = 1;        // FCY = DOZE/2 = 28.8KHz (DOZE = FOSC/2)    * DOZE = processor clock reduction bit
    CLKDIVbits.DOZEN = 1;       // DOZE mode enable bit (field specifies ratio between peripheral clock and cpu clock                                          
}

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
    //Note: Gyroscope is an 8-bit Register.
    SPI2CON1bits.MODE16 = 0;    // Enable the 8-bit data mode
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
    //NOTE: RP4 was previously OC1 [Not enough remappable pins available]
    RPINR22bits.SDI2R = 11;     // PIN 11 (RP4 - SDI2)
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
    //NOTE: RP4 was previously OC1 [Not enough remappable pins available]
    TRISBbits.TRISB4 = 1;       // PIN 11 (RB4 - INPUT  for SDI2)
}

void init_PWM1(){
    // PWM1 Timebase control register
    P1TCONbits.PTEN = 0;                                                        // Do not enable the clock delivery to PWM1 timer yet	
    P1TCONbits.PTCKPS = 0;                                                      // Prescale is 1:1 so Timer1 clock = 57.6KHz = FP
    P1TCONbits.PTMOD = 0;                                                       // PWM1 is in free-running mode

    // PWM1 counter and period
    P1TMRbits.PTMR = 0;                                                         // Initial value in PWM1 counter register
    P1TPER = 1200;                                                              // PWM1 period register produces 20msec PWM period (originally 1200 for 20ms. Changes to 400 to make servos faster.)
    
    // PWM1 Control Register 1
    PWM1CON1bits.PMOD3 = 1;                                                     // PWM I/O Pin =  Independent PWM Output Mode    
    PWM1CON1bits.PMOD2 = 1;                                                     // PWM I/O Pin =  Independent PWM Output Mode  
    PWM1CON1bits.PMOD1 = 1;                                                     // PWM I/O Pin =  Independent PWM Output Mode   
    PWM1CON1bits.PEN1L = 1;                                                     // PIN 26 (PWM1_1)
    PWM1CON1bits.PEN1H = 1;                                                     // PIN 25 (PWM1_2)
    PWM1CON1bits.PEN2L = 1;                                                     // PIN 24 (PWM1_3)  
    PWM1CON1bits.PEN2H = 1;                                                     // PIN 23 (PWM1_4)
    PWM1CON1bits.PEN3L = 1;                                                     // PIN 22 (PWM1_5)
    PWM1CON1bits.PEN3H = 1;                                                     // PIN 21 (PWM1_6)

    // PWM1 Control Register 2
    PWM1CON2bits.IUE = 0;                                                       // Updates are synchronized with timebase
    PWM1CON2bits.UDIS = 0;                                                      // Updates from period and duty cycle registers are enabled

    // Initialize PWM1 Servo Positions 
    P1DC1 = 0;
    P1DC2 = 0;
    P1DC3 = 0;
}

void init_PWM2(){
    // PWM2 Timebase control register
    P2TCONbits.PTEN = 0;                                                        // Do not enable the clock delivery to PWM2 timer yet	
    P2TCONbits.PTCKPS = 0;                                                      // Prescale is 1:1 so Timer1 clock = 57.6KHz = FP
    P2TCONbits.PTMOD = 0;                                                       // PWM2 is in free-running mode

    // PWM2 counter and period
    P2TMRbits.PTMR = 0;                                                         // Initial value in PWM2 counter register
    P2TPER = 1200;                                                              // PWM1 period register produces 20msec PWM period

    // PWM2 Control Register 1
    PWM2CON1bits.PMOD1 = 1;                                                     // PWM I/O Pin =  Independent PWM Output Mode 
    PWM2CON1bits.PEN1H = 1;                                                     // PIN 17 (PWM2_1) PWM Output
    PWM2CON1bits.PEN1L = 1;                                                     // PIN 18 (PWM2_2) PWM Output  
    
    // PWM2 Control Register 2
    PWM2CON2bits.IUE = 0;                                                       // Updates are synchronized with timebase
    PWM2CON2bits.UDIS = 0;                                                      // Updates from period and duty cycle registers are enabled

    // Initialize PWM2 Servo Positions
    P2DC1 = 0;
}

void init_OC(){
    //Timer3 Register
    T3CONbits.TGATE = 0; // Gated (with external clock) timer 3 clock disabled.
    T3CONbits.TCKPS = 0; // Prescale is 1:1. 57.7KHz = FP
    T3CONbits.TCS = 0; // Timer3 works with internal clock = 57.6KHz = FP

    //Program Timer 3 time base. 
    TMR3 = 0; //Initial Value in Timer3 Counter Register.
    PR3 = 1200; // Timer3 Period Register Produces 20msec PWM Period

    //Output Compare Register
    //NOTE: RP4 was previously OC1 [Not enough remappable pins available]
    //OC1CONbits.OCTSEL = 1;                                                    //Timer3 Selected for OC1
    OC2CONbits.OCTSEL = 1;                                                      //Timer3 Selected for OC2
    OC3CONbits.OCTSEL = 1;                                                      //Timer3 Selected for OC3
    OC4CONbits.OCTSEL = 1;                                                      //Timer3 Selected for OC4
    //NOTE: RP4 was previously OC1 [Not enough remappable pins available]
    //OC1CONbits.OCM = 6;                                                       //OC1 in PWM Mode w/ Fault Pin Disabled
    OC2CONbits.OCM = 6;                                                         //OC2 in PWM Mode w/ Fault Pin Disabled
    OC3CONbits.OCM = 6;                                                         //OC3 in PWM Mode w/ Fault Pin Disabled
    OC4CONbits.OCM = 6;                                                         //OC4 in PWM Mode w/ Fault Pin Disabled
    
    //Output Compare 1 Primary and Secondary Initial Duty Cycle
    OC1R = 1200;                                                                  //Initial PWM Duty cycle value to compare against TIMER3
    //Output Compare 2 Primary and Secondary Initial Duty Cycle
    OC2R = 1200;                                                                  //Initial PWM Duty cycle value to compare against TIMER3
    //Output Compare 3 Primary and Secondary Initial Duty Cycle
    OC3R = 1200;                                                                  //Initial PWM Duty cycle value to compare against TIMER3
    //Output Compare 4 Primary and Secondary Initial Duty Cycle
    OC4R = 1200;                                                                  //Initial PWM Duty cycle value to compare against TIMER3

    //Output Compare Pin Assignments
    //NOTE: RP4 was previously OC1 [Not enough remappable pins available]
    //TRISBbits.TRISB4 = 0;                                                       //(Pin 11) RP4 is an Output
    TRISBbits.TRISB5 = 0;                                                       //(Pin 14) RP5 is an Output
    TRISBbits.TRISB6 = 0;                                                       //(Pin 15) RP6 is an Output
    TRISBbits.TRISB7 = 0;                                                       //(Pin 16) RP7 is an Output
    
    //NOTE: RP4 was previously OC1 [Not enough remappable pins available]
    //RPOR2bits.RP4R = 18;                                                        //(Pin 11) OC1 to RP4
    RPOR2bits.RP5R = 19;                                                        //(Pin 14) OC2 to RP5
    RPOR3bits.RP6R = 20;                                                        //(Pin 15) OC3 to RP6
    RPOR3bits.RP7R = 21;                                                        //(Pin 16) OC4 to RP7
    
    //Initialize OC Servo Positions
    //NOTE: RP4 was previously OC1 [Not enough remappable pins available]
    //OC1RS = 0;
    OC2RS = 0;
    OC3RS = 0;
    OC4RS = 0;    
}

void init_IR(){
    TRISAbits.TRISA2 = 1;       // PIN 09 (RA2 - INPUT for IR_Sensor)
}

void init_Gyroscope()
{
    Write_Control_Reg1 (0x1C);      // ODR = 100Hz, BW = 25Hz, only Z axis is enabled
    Write_Control_Reg2 (0x20);      // Normal mode, HPF cut-off freq = 8Hz at ODR = 100Hz
    Write_Control_Reg3 (0x00);      // All interrupts are disabled
    Write_Control_Reg4 (0x80);      // Update depends on reading XYZ registers, Little Endian format, 250 degrees/sec, 4-wire SPI
    Write_Control_Reg5 (0x00);      // FIFO disabled, High pass filter disabled, data comes from LPF1 output
    Write_FIFO_Control_Reg (0x00);  // Bypass mode
}


int main(void) {
   
    init_Oscillator();
    
    PPSUnLock;
    //Sets Analog Pins to Digital {Required for SPI}
    ADPCFGbits.PCFG0 = 1;
    ADPCFGbits.PCFG1 = 1;
    ADPCFGbits.PCFG2 = 1;
    ADPCFGbits.PCFG3 = 1;
    ADPCFGbits.PCFG4 = 1;
    ADPCFGbits.PCFG5 = 1;
    
    init_SPI();
    init_PWM1();
    init_PWM2();
    init_OC();
    init_IR();
    init_Gyroscope();
    PPSLock;

    P1TCONbits.PTEN = 1; //Enable the clock delivery to PWM1 Timer
    P2TCONbits.PTEN = 1; // Enable the clock delivery to PWM2 timer. 
    T3CONbits.TON = 1;   // Enable the clock delivery to  OC  timer. 

    int i = 0;
    int toggle = 1;


    delay(200, 100); //Stops the servo from taking off instantly!

    while (1) {

        /*Note: Currently untested, function should do one of the following:
         * Return a boolean value that will decide what the servo should do. 
         * Return an integer value that will tell the servo how long to readjust itself. 
         * */
        //Gyroscope_Test();
       
        /*Controlled by IR sensor, if active the robot will do the following:
         * 1. Stop movement for X seconds
         * 2. Move backwards for X seconds.
         * 3. Turn Left/Right based on a toggle to avoid "obstacle"
         */
        if (PORTAbits.RA2) { //PIN 09    
            //If IR_Sensor is blocked, stop all PWM signals.     
            stopMovement();
            for (i = 0; i < 5; i++) {
                SPI_Transmit1('S');
            }
            delay(200, 100);

            backMovement();
            for (i = 0; i < 5; i++) {
                SPI_Transmit1('B');
            }
            delay(150, 100);

            if (toggle) {
                leftMovement();
                for (i = 0; i < 5; i++) {
                    SPI_Transmit1('F'); //Slaves only move forward.
                }
                toggle = 0;
                delay(200, 100);
            } else {
                rightMovement();
                for (i = 0; i < 5; i++) {
                    SPI_Transmit1('F'); //Slaves only move forward.
                }
                toggle = 1;
                delay(200, 100);
            }
        }
            
        else {
            forwardMovement(); //Default Movement [Forward Movement]
            SPI_Transmit1('F'); // Send a data byte {Instruction to Continue Moving}
        }
    }
    return (1);
}

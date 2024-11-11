/*
 * File:   main.c
 * Author: Emiko Emiko, Eshilama Akalumhe
 *
 * Created on: USE THE INFORMATION FROM THE HEADER MPLAB X IDE GENERATES FOR YOU
 */

// FBS
#pragma config BWRP = OFF               // Table Write Protect Boot (Boot segment may be written)
#pragma config BSS = OFF                // Boot segment Protect (No boot program Flash segment)

// FGS
#pragma config GWRP = OFF               // General Segment Code Flash Write Protection bit (General segment may be written)
#pragma config GCP = OFF                // General Segment Code Flash Code Protection bit (No protection)

// FOSCSEL
#pragma config FNOSC = FRC              // Oscillator Select (Fast RC oscillator (FRC))
#pragma config IESO = OFF               // Internal External Switch Over bit (Internal External Switchover mode disabled (Two-Speed Start-up disabled))

// FOSC
#pragma config POSCMOD = NONE           // Primary Oscillator Configuration bits (Primary oscillator disabled)
#pragma config OSCIOFNC = ON            // CLKO Enable Configuration bit (CLKO output disabled; pin functions as port I/O)
#pragma config POSCFREQ = HS            // Primary Oscillator Frequency Range Configuration bits (Primary oscillator/external clock input frequency greater than 8 MHz)
#pragma config SOSCSEL = SOSCHP         // SOSC Power Selection Configuration bits (Secondary oscillator configured for high-power operation)
#pragma config FCKSM = CSECMD           // Clock Switching and Monitor Selection (Clock switching is enabled, Fail-Safe Clock Monitor is disabled)

// FWDT
#pragma config WDTPS = PS32768          // Watchdog Timer Postscale Select bits (1:32,768)
#pragma config FWPSA = PR128            // WDT Prescaler (WDT prescaler ratio of 1:128)
#pragma config WINDIS = OFF             // Windowed Watchdog Timer Disable bit (Standard WDT selected; windowed WDT disabled)
#pragma config FWDTEN = OFF             // Watchdog Timer Enable bit (WDT disabled (control is placed on the SWDTEN bit))

// FPOR
#pragma config BOREN = BOR3             // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware; SBOREN bit disabled)
#pragma config PWRTEN = ON              // Power-up Timer Enable bit (PWRT enabled)
#pragma config I2C1SEL = PRI            // Alternate I2C1 Pin Mapping bit (Default location for SCL1/SDA1 pins)
#pragma config BORV = V18               // Brown-out Reset Voltage bits (Brown-out Reset set to lowest voltage (1.8V))
#pragma config MCLRE = ON               // MCLR Pin Enable bit (MCLR pin enabled; RA5 input pin disabled)

// FICD
#pragma config ICS = PGx2               // ICD Pin Placement Select bits (PGC2/PGD2 are used for programming and debugging the device)

// FDS
#pragma config DSWDTPS = DSWDTPSF       // Deep Sleep Watchdog Timer Postscale Select bits (1:2,147,483,648 (25.7 Days))
#pragma config DSWDTOSC = LPRC          // DSWDT Reference Clock Select bit (DSWDT uses LPRC as reference clock)
#pragma config RTCOSC = SOSC            // RTCC Reference Clock Select bit (RTCC uses SOSC as reference clock)
#pragma config DSBOREN = ON             // Deep Sleep Zero-Power BOR Enable bit (Deep Sleep BOR enabled in Deep Sleep)
#pragma config DSWDTEN = ON             // Deep Sleep Watchdog Timer Enable bit (DSWDT enabled)

// #pragma config statements should precede project file includes.

#include <xc.h>



#include "xc.h"
#include "math.h"
#include "string.h"

uint16_t PB1_event;
uint16_t PB2_event;
uint16_t PB3_event;

uint16_t INDIV_PB_event;

uint8_t state;
uint16_t minutes;
uint16_t seconds;
uint8_t PB3_start;
uint8_t PB3_stop;
uint8_t PB3_pressed;
uint8_t PB2_pressed;
uint8_t PB1_pressed;
uint8_t PB2_long_press;// Flag for PB2 long press (increment by 5)
uint8_t timer_counter;// Counter to track button press duration
uint8_t long_PB3press;

/**
 * You might find it useful to add your own #defines to improve readability here
 */

//clkChange.c

void newClk(unsigned int clkval) {
    uint8_t COSCNOSC;
    switch(clkval) {
        case 8: // 8 MHz
            COSCNOSC = 0x00;
            break;
        case 500: // 500 kHz
            COSCNOSC = 0x66;
            break;
        case 32: // 32 kHz
            COSCNOSC = 0x55;
            break;
        default:
            COSCNOSC = 0x55;
    }
    SRbits.IPL = 7;
    CLKDIVbits.RCDIV = 0;
    __builtin_write_OSCCONH(COSCNOSC);
    __builtin_write_OSCCONL(0x01);
    while(OSCCONbits.OSWEN==1) {}
    SRbits.IPL = 0;
}


//UART2.c
unsigned int clkval;

///// Initialization of UART 2 module.

void InitUART2(void) 
{
	// configures UART2 module on pins RB0 (Tx) and RB1 (Rx) on PIC24F16KA101 
	// Enables UART2 
	//Set to Baud 4800 with 500kHz clk on PIC24F
	
	TRISBbits.TRISB0=0;
	TRISBbits.TRISB1=1;
	LATBbits.LATB0=1;

	// configure U2MODE
    U2MODE = 0b0000000000001000;
/*    
	U2MODEbits.UARTEN = 0;	// Bit15 TX, RX DISABLED, ENABLE at end of func
	U2MODEbits.USIDL = 0;	// Bit13 Continue in Idle
	U2MODEbits.IREN = 0;	// Bit12 No IR translation
	U2MODEbits.RTSMD = 0;	// Bit11 Simplex Mode
	U2MODEbits.UEN = 0;		// Bits8,9 TX,RX enabled, CTS,RTS not
	U2MODEbits.WAKE = 0;	// Bit7 No Wake up (since we don't sleep here)
	U2MODEbits.LPBACK = 0;	// Bit6 No Loop Back
	U2MODEbits.ABAUD = 0;	// Bit5 No Autobaud (would require sending '55')
	U2MODEbits.RXINV = 0;	// Bit4 IdleState = 1
	U2MODEbits.BRGH = 1;	// Bit3 16 clocks per bit period
	U2MODEbits.PDSEL = 0;	// Bits1,2 8bit, No Parity
	U2MODEbits.STSEL = 0;	// Bit0 One Stop Bit
 */
	if (OSCCONbits.COSC == 0b110)
	{
		U2BRG = 12;	// gives a baud rate of 4807.7 Baud with 500kHz clock; Set Baud to 4800 on realterm
	}
	else if (OSCCONbits.COSC == 0b101)
	{
		U2BRG = 12;	// gives a baud rate of 300 Baud with 32kHz clock; set Baud to 300 on realterm
	}
	else if (OSCCONbits.COSC == 0b000)
	{
		U2BRG=103;	// gives a baud rate of 9600 with 8MHz clock; set Baud to 9600 on real term
	}
	// Load all values in for U1STA SFR
	U2STA = 0b1010000000000000;
    /*
    U2STAbits.UTXISEL1 = 1;	//Bit15 Int when Char is transferred (1/2 config!)
    U2STAbits.UTXISEL0 = 1;	//Generate interrupt with last character shifted out of U2TXREG buffer
	U2STAbits.UTXINV = 0;	//Bit14 N/A, IRDA config
	U2STAbits.UTXBRK = 0;	//Bit11 Disabled
	U2STAbits.UTXEN = 0;	//Bit10 TX pins controlled by periph
	U2STAbits.UTXBF = 0;	//Bit9 *Read Only Bit*
	U2STAbits.TRMT = 0;		//Bit8 *Read Only bit*
	U2STAbits.URXISEL = 0;	//Bits6,7 Int. on character recieved
	U2STAbits.ADDEN = 0;	//Bit5 Address Detect Disabled
	U2STAbits.RIDLE = 0;	//Bit4 *Read Only Bit*
	U2STAbits.PERR = 0;		//Bit3 *Read Only Bit*
	U2STAbits.FERR = 0;		//Bit2 *Read Only Bit*
	U2STAbits.OERR = 0;		//Bit1 *Read Only Bit*
	U2STAbits.URXDA = 0;	//Bit0 *Read Only Bit*
    */
	IFS1bits.U2TXIF = 0;	// Clear the Transmit Interrupt Flag
    IPC7bits.U2TXIP = 3; // UART2 TX interrupt has interrupt priority 3-4th highest priority
    
	IEC1bits.U2TXIE = 1;	// Enable Transmit Interrupts
	IFS1bits.U2RXIF = 0;	// Clear the Recieve Interrupt Flag
	IPC7bits.U2RXIP = 4; //UART2 Rx interrupt has 2nd highest priority
    IEC1bits.U2RXIE = 0;	// Disable Recieve Interrupts

	U2MODEbits.UARTEN = 1;	// And turn the peripheral on

	U2STAbits.UTXEN = 1;
	return;
}



///// XmitUART2: 
///// Displays 'DispData' on realterm 'repeatNo' of times using UART to PC. 
///// Adjust Baud on real term as per clock: 32kHz clock - Baud=300 // 500kHz clock - Baud=4800 

void XmitUART2(char CharNum, unsigned int repeatNo)
{	
	
	InitUART2();	//Initialize UART2 module and turn it on
	while(repeatNo!=0) 
	{
		while(U2STAbits.UTXBF==1)	//Just loop here till the FIFO buffers have room for one more entry
		{
			// Idle();  //commented to try out serialplot app
		}	
		U2TXREG=CharNum;	//Move Data to be displayed in UART FIFO buffer
		repeatNo--;
	}
	while(U2STAbits.TRMT==0)	//Turn off UART2 upon transmission of last character; also can be Verified in interrupt subroutine U2TXInterrupt()
	{
		//Idle();
	}
	U2MODEbits.UARTEN = 0;	
	return;
}


void __attribute__ ((interrupt, no_auto_psv)) _U2RXInterrupt(void) {
//	LATA = U2RXREG;
	IFS1bits.U2RXIF = 0;
}
void __attribute__ ((interrupt, no_auto_psv)) _U2TXInterrupt(void) {
	IFS1bits.U2TXIF = 0;

}




// Displays 16 bit number in Hex form using UART2
void Disp2Hex(unsigned int DispData)   
{
    char i;
    char nib = 0x00;
    XmitUART2(' ',1);  // Disp Gap
    XmitUART2('0',1);  // Disp Hex notation 0x
    XmitUART2('x',1);
    
    for (i=3; i>=0; i--)
    {
        nib = ((DispData >> (4*i)) & 0x000F);
        if (nib >= 0x0A)
        {
            nib = nib +0x37;  //For Hex values A-F
        }
        else 
        {
            nib = nib+0x30;  //For hex values 0-9
        }
        XmitUART2(nib,1);
    }
    
    XmitUART2(' ',1);
    DispData = 0x0000;  // Clear DispData
    return;
}


void Disp2Hex32(unsigned long int DispData32)   // Displays 32 bit number in Hex form using UART2
{
    char i;
    char nib = 0x00;
    XmitUART2(' ',1);  // Disp Gap
    XmitUART2('0',1);  // Disp Hex notation 0x
    XmitUART2('x',1);
    
    for (i=7; i>=0; i--)
    {
        nib = ((DispData32 >> (4*i)) & 0x000F);
        if (nib >= 0x0A)
        {
            nib = nib +0x37;  //For Hex values A-F
        }
        else 
        {
            nib = nib+0x30;  //For hex values 0-9
        }
        XmitUART2(nib,1);
    }
    
    XmitUART2(' ',1);
    DispData32 = 0x00000000;  // Clear DispData
    return;
}

// Displays 16 bit unsigned in in decimal form
void Disp2Dec(uint16_t Dec_num)
{
    uint8_t rem;  //remainder in div by 10
    uint16_t quot; 
    uint8_t ctr = 0;  //counter
    XmitUART2(' ',1);  // Disp Gap
    while(ctr<5)
    {
        quot = Dec_num/(pow(10,(4-ctr)));
        rem = quot%10;
        XmitUART2(rem + 0x30 , 1);
        ctr = ctr + 1;
    }
    XmitUART2(' ',1);  // Disp Gap
    // XmitUART2('\n',1);  // new line
    // XmitUART2('\r',1);  // carriage return
   
    return;
}


void Disp2String(char *str) //Displays String of characters
{
    unsigned int i;
   // XmitUART2(0x0A,2);  //LF
   // XmitUART2(0x0D,1);  //CR 
    for (i=0; i<= strlen(str); i++)
    {
          
        XmitUART2(str[i],1);
    }
    // XmitUART2(0x0A,2);  //LF
    // XmitUART2(0x0D,1);  //CR 
    
    return;
}
void Timer3Init(void) {
    T3CON = 0; // Clear Timer3 control register
    TMR3 = 0;  // Clear Timer3 counter
    PR3 = 62500; // Set period for 3 seconds
    T3CONbits.TCKPS = 2; // set prescaler to 1:8
    T3CONbits.TCS = 0; // use internal clock
    T3CONbits.TSIDL = 0; //operate in idle mode
    IPC2bits.T3IP = 3; //7 is highest and 1 is lowest pri.
    IFS0bits.T3IF = 0;
    IEC0bits.T3IE = 1; //enable timer interrupt
    //T3CONbits.TCKPS = 2; // Set appropriate prescaler if needed
}

void Timer1Init(void) {
    T1CON = 0;            // Clear Timer1 control register
    TMR1 = 0;             // Clear Timer1 counter
    PR1 = 1953;          // Set period for 3 seconds
                          // Calculated as: PR1 = (3s) * (500kHz / 256) = 23437
    T1CONbits.TCKPS = 3;  // Set prescaler to 1:256
    T1CONbits.TCS = 0;    // Use internal clock (Fosc/2)
    T1CONbits.TSIDL = 0;  // Operate in idle mode
    IPC0bits.T1IP = 3;    // Set priority level (1 to 7)
    IFS0bits.T1IF = 0;    // Clear Timer1 interrupt flag
    IEC0bits.T1IE = 1;    // Enable Timer1 interrupt
}

int delay_ms(uint16_t time_ms){
    
    
   T2CONbits.T32 = 0; // operate timer 3 as 16 bit timer
    T3CONbits.TCKPS = 2; // set prescaler to 1:8
    T3CONbits.TCS = 0; // use internal clock
    T3CONbits.TSIDL = 0; //operate in idle mode
    IPC2bits.T3IP = 2; //7 is highest and 1 is lowest pri.
    IFS0bits.T3IF = 0;
    IEC0bits.T3IE = 1; //enable timer interrupt
    
    //add a switch case here for the different PR2 values
     if (time_ms == 500){
       PR3 = 1953;
    }
    
    if(time_ms == 1000){
        PR3 = 3906;
    }
    
    if(time_ms == 4000){
        PR3 = 15625;
    }
    
//    PR2 = time_ms * 250;
    TMR3 =0;
    
    T3CONbits.TON=1;//start the timer
    Idle();
    T3CONbits.TON=0;//stop the timer
    return 0; 
}

int delay2_ms(uint16_t time_ms){
    
    
   T2CONbits.T32 = 0; // operate timer 2 as 16 bit timer
    T2CONbits.TCKPS = 2; // set prescaler to 1:8
    T2CONbits.TCS = 0; // use internal clock
    T2CONbits.TSIDL = 0; //operate in idle mode
    IPC2bits.T3IP = 2; //7 is highest and 1 is lowest pri.
    IFS0bits.T3IF = 0;
    IEC0bits.T3IE = 1; //enable timer interrupt
    
    //add a switch case here for the different PR2 values
     if (time_ms == 500){
       PR2 = 1953;
    }
    
    if(time_ms == 1000){
        PR2 = 3906;
    }
    
    if(time_ms == 4000){
        PR2 = 15625;
    }
    
//    PR2 = time_ms * 250;
    TMR2 =0;
    
    T2CONbits.TON=1;//start the timer
    Idle();
    T2CONbits.TON=0;//stop the timer
    return 0; 
}


void IOinit(void){
    AD1PCFG = 0xFFFF; /* keep this line as it sets I/O pins that can also be analog to be digital */
    
    newClk(500);
    
    //T3CON config
    T2CONbits.T32 = 0; // operate timer 2 as 16 bit timer
    T2CONbits.TCKPS = 1; // set prescaler to 1:8
    T2CONbits.TCS = 0; // use internal clock
    T2CONbits.TSIDL = 0; //operate in idle mode
    IPC1bits.T2IP = 2; //7 is highest and 1 is lowest pri.
    IFS0bits.T2IF = 0;
    IEC0bits.T2IE = 1; //enable timer interrupt
    PR2 = 31250; // set the count value for approximately 1s
    TMR2 = 0;
    T2CONbits.TON = 1;

    /* Let's set up some I/O */
    TRISBbits.TRISB8 = 0;
    LATBbits.LATB8 = 0;
    
    TRISAbits.TRISA4 = 1;
    CNPU1bits.CN0PUE = 1;
    CNEN1bits.CN0IE = 1;
    
    TRISBbits.TRISB4 = 1;
    CNPU1bits.CN1PUE = 1;
    CNEN1bits.CN1IE = 1;
    
    TRISAbits.TRISA2 = 1;
    CNPU2bits.CN30PUE = 1;
    CNEN2bits.CN30IE = 1;
    
    /* Let's clear some flags */
    PB1_event = 0;
    PB2_event = 0;
    PB3_event = 0;
    INDIV_PB_event = 0;
    
    
    IPC4bits.CNIP = 6;
    IFS1bits.CNIF = 0;
    IEC1bits.CNIE = 1;
    
    /* Let's set up our UART */    
    InitUART2();
    
    
    //Defining states
    #define STATE_A 0//Idle() mode
    #define STATE_B 1//COUNTING MODE While program is in counting mode, if PB3 is pressed, switch to STOP/SET mode (STATE_C). In the background, if minutes >= 0 { if seconds=0: seconds=59, else if seconds>0: seconds -= 1}, then, we display the time stamp. This timp stamp will continue to be refreshed as long as we're in STATEB. We need to set a 1 second delay before the timestamp is displayed
    #define STATE_C 2//STOP/SETUP MODE. While program is in this state, if PB1 is preesed, increment minutes. If PB2 is pressed increment seconds. If PB3 is pressed, we change the state to STATEB for counting mode. If PB3 is pressed even after long delay, we set minutes and seconds to 0
    #define STATE_D 3// FIN MODE. here we just keep the LED on until
    #define STATE_E 4//RESET MODE
    //uint8_t state;
}





int main(void) {
    
    /** This is usually where you would add run-once code
     * e.g., peripheral initialization. For the first labs
     * you might be fine just having it here. For more complex
     * projects, you might consider having one or more initialize() functions
     */
    
    IOinit();
    //Timer3Init();
    Timer1Init();
    long_PB3press = 0;
    
    state = STATE_A;
    //increment_value = 1;
    while(1) {
        //how are the minutes and seconds getting back to 0??
        
        while (state == STATE_A){
            Idle();
        }
        
        while(state == STATE_B){
//            //delay to check for long press of PB3
//            if(PB3_pressed == 1){
//               //Timer3Init();
//               T3CONbits.TON = 1;  // Start Timer3
//                
//                
//            }
            //PB3_start = 1; //indicating that we are in COUNTING mode
           
            
            
            
            if (PB3_stop == 1){//that is, if PB3 is pressed to go back to setup mode
                //PB3_stop=0;
                state = STATE_C;
                break;
            }
            
            
                 //decrement time accordingly and display it here
            if (minutes>0 && minutes <=59){
                if(seconds>0){
                    seconds -= 1;
                }
                if (seconds == 0){
                    seconds = 59;
                    minutes -= 1;
                }
                
            }
             if (minutes == 0){
                 //Disp2String("\n\r hello");
                if(seconds>0){
                    seconds-=1;
                }
                else if(seconds == 0){
                    //here the timer would probably be finished counting, so we display the FIN message,and change the state to STATE_A(Idle()) and keep the LED on
                    LATBbits.LATB8 =1;
                    Disp2String("\r FIN 00m : 00s -- ALARM ");
                    //Idle();
                    state = STATE_A;
                    break;
                }
            }
            
            delay_ms(1000);
            LATBbits.LATB8 ^=1;
            Disp2String("\r CNT ");
            Disp2Dec(minutes);
            Disp2String("m : ");
            Disp2Dec(seconds);
            Disp2String("s");
       
           
        }
        
        while(state == STATE_C){
           
            //PB3_stop=1; //indicating that we are in stop/setup mode
            
//            delay_ms(1000);//for now we'll implement a long button press when switching to STATE E, which is RESET MODE. We still need to implement a long button press when switching from STATE_B (i.e while the countdown is running)
//            if(PB3_pressed == 1){\
//               //Timer3Init();
//               T3CONbits.TON = 1;  // Start Timer3
//                
//                
//            }
            delay_ms(1000);
            //delay_ms(500);
            //Timer3Init();
            
            if (PB3_start == 1){//that is, if PB3 is pressed to go back to countdown mode
                //PB3_stop=0;
                //PB3_start = 0;
                state = STATE_B;
                break;
            }
            //here, since we already updated the minutes and seconds in the ISR we can just display the current value of the timestamp
           
            //if(PORTAbits.RA2 == 0 || PORTBbits.RB4 == 1){
                Disp2String("\r SET ");
                Disp2Dec(minutes);
                Disp2String("m : ");
                Disp2Dec(seconds);
                Disp2String("s");
            //}
        }
        
        while(state == STATE_D){//FIN mode. To get out of this mode you have to press and hold PB3 to go into reset mode (STATE_E)
            LATBbits.LATB8 = 1;
            //delay_ms(1000);
            //Disp2String("\n\r in state D ");
             //Disp2Dec(minutes);
             //Disp2Dec(seconds);
//            if(PORTAbits.RA4 == 0){//Long button press takes us to STATE_E (reset mode)
//                state = STATE_E;
//                break;
//            }
            
        }
        
        if(state == STATE_E){//We get to this state after PB3 is pressed for a while
            LATBbits.LATB8 = 0;
            minutes = 0;
            seconds = 0;
            Disp2String("\r CLR 00m : 00s            ");
            delay_ms(4000);
            state = STATE_A;//setting it back to STATE_A (Idle mode) after a while to save power
            break;
        }
        
    }
    return 0;
}


// Timer 2 interrupt subroutine
void __attribute__((interrupt, no_auto_psv)) _T2Interrupt(void){
    //Don't forget to clear the timer 2 interrupt flag!
    IFS0bits.T2IF = 0;
    
    // Increment the counter when either PB1 or PB2 is pressed
    if (PB1_pressed || PB2_pressed) {
         
        timer_counter++;
        
        
        // Handle PB1 pressed (increment minutes)
        if (PB1_pressed) {// or maybe change this to PORTAbits
            // Increment minutes every second (based on Timer2 configuration)
            if (timer_counter % 1 == 0) {  // 1-second interval based on PR2=3906
                if (minutes < 59) {
                    minutes++;
                } else {
                    minutes = 0;
                }
 
            }
        }
        
         // Handle PB2 pressed (increment seconds)
        if (PB2_pressed) {
            // After holding PB2 for 2 seconds, start incrementing by 5
            if (timer_counter >= 2) {   // After 2 seconds
                PB2_long_press = 1;
            }

            // Increment seconds either by 1 or by 5 based on PB2_long_press
            if (timer_counter % 1 == 0) { // Every 1 second based on PR2=3906
                if (PB2_long_press) {
                    if (seconds + 5 <= 59) {
                        seconds += 5;
                    } else {
                        seconds = 0;
                    }
                } else {
                    if (seconds < 59) {
                        seconds++;
                    } else {
                        seconds = 0;
                    }
                }

            }
        }
    }
}

void __attribute__((interrupt, no_auto_psv)) _T1Interrupt(void){
    //Don't forget to clear the timer 2 interrupt flag!
    IFS0bits.T1IF = 0;
    //Disp2String("\n checking if it even gets here");
    TMR1 = 0;
    T1CONbits.TON = 0;//stop timer
   if(PORTAbits.RA4 == 0){
        //PB3_pressed = 0;
        state = STATE_E;
        long_PB3press = 1;
    }
   
   
    
   else{
       PB3_pressed = 0;
   }
    T1CONbits.TON = 0; // Stop Timer 3
    TMR1 = 0; // Clear Timer 3 counter
   
}

void __attribute__((interrupt, no_auto_psv)) _CNInterrupt(void){
    //Don't forget to clear the CN interrupt flag!
    IFS1bits.CNIF = 0;

    //In the background, if minutes >= 0 { if seconds=0: seconds=59, else if seconds>0: seconds -= 1}, then, we display the time stamp. This timp stamp will continue to be refreshed as long as we're in STATEB. We need to set a 1 second delay before the timestamp is displayed
    if(PORTAbits.RA2 == 0 && PORTBbits.RB4 == 1 && PORTAbits.RA4 == 1){//PB1 -minute button
        PB1_pressed = 1;
        timer_counter = 0; // Reset the timer counter
        //T2CONbits.TON = 1;
        
        if(state == STATE_A || state == STATE_C || state == STATE_D || state == STATE_E){
            state = STATE_C;//change to STATE_C (STOP/SETUP MODE). Doesn't matter what state we were in before, as long as the timer isnt running (STATE_B), we want to set the program to STATE_C (STOP/SETUP mode)
        }
        
        
       if(minutes>=0 && minutes<59){
           minutes += 1;
       }else{
           minutes = 0;
       }
       
    }else{
        PB1_pressed = 0;
    }
    
    if(PORTBbits.RB4 == 0 && PORTAbits.RA2 == 1 && PORTAbits.RA4 == 1){//PB2 -second button
        
         if(state == STATE_A || state == STATE_C || state == STATE_D || state == STATE_E){
            state = STATE_C;//change to STATE_C (STOP/SETUP MODE)
        }
        
         if(seconds >= 0 && seconds<59){
             seconds += 1;
         }
         else{
             seconds = 0;
         }
         
        //TMR2 = 0;           // Reset Timer2
        //T2CONbits.TON = 1;  // Start Timer2
        PB2_pressed = 1;
        timer_counter = 0;  // Reset the timer countE
        //PR2=15625;
    }else{
        PB2_pressed = 0;
    }
 
    if(PORTAbits.RA4 == 0 && PORTAbits.RA2 == 1 && PORTBbits.RB4 == 1){//PB3 for starting/stopping the countdown
        //state = STATE_C;
        
        if (state == STATE_E){
                state = STATE_C;
                PB3_start = 1;//to show that we've left/we're leaving counting mode
                PB3_stop = 0;//to show that we're entering stop/setup mode
        }
        TMR1 = 0;           // Reset Timer3
        T1CONbits.TON = 1;  // Start Timer3
        //Disp2String("\n checking if it even gets here");
        PB3_pressed = 1;//TO IMPLEMENT THE LONG BUTTON PRESS, WE FIRST NEED TO TURN THE PB3 FLAG ON AND THEN CHECK FOR A DELAY LATER in the main code
        
        if (!long_PB3press){
            if(state == STATE_B){
                //state = STATE_C;
                PB3_start = 0;//to show that we've left counting mode
                PB3_stop = 1;//to show that we're entering stop/setup mode
               //PB3_pressed = 0;
            }

            else if(state == STATE_C){
                //state = STATE_B;
                PB3_start = 1;//to show that we've left/we're leaving counting mode
                PB3_stop = 0;//to show that we're entering stop/setup mode
               //PB3_pressed = 0;
            }
            
             
        }
        //long_PB3press = 0;

    }else{
        PB3_pressed = 0;
        if (!long_PB3press) {
            // Short press detected (less than 3 seconds)
            T1CONbits.TON = 0;  // Stop Timer 3
            TMR1 = 0;  // Reset Timer 3
        }
    }

}


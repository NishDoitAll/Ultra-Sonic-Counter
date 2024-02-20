/*
 * File:   main.c(Ultra Sonic Sensor as a Counter)
 * Author: Nish
 *
 * Created on 14 February 2024, 14:10
 */


#include <xc.h>
#include <pic16f690.h>
#include "config.h"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <builtins.h>

#define _XTAL_FREQ 4000000
#define Trig_d TRISC1
#define Trig RC4
#define Echo RC6
#define trigger RC4
#define echo RC6
#define LEDL RB5 
#define LEDR RB6
#define LEDT RC5
#define BUTTONL RA2
#define BUTTONR RC3

uint8_t TMR1_C = 0, Edge=0;
uint32_t t1=0x00, t2=0x00, tp;
float Distance=0, Tp;
char Buffer[20];
int threshold;
int minnthreshold;
int minthreshold;
int maxthreshold;
int count = 0;
int calcount = 0;
int highcount = 0;
int calhighcount = 0;
int lowcount = 0;
int dist=0;
char thres[4];
int D = 0;
int i = 0;
uint8_t C = 0;

//FUNCTION  TO CALCULATE DISTANCE FROM THE ULTRASONIC SENSOR
int calc_dist(void)
{
  int distance=0;
  TMR1=0;
  // Send Trigger Pulse To The Sensor
  trigger=1;
  __delay_us(10);
  trigger=0;
  // Wait For The Echo Pulse From The Sensor
  while(!echo);
  // Turn ON Timer Module
  TMR1ON=1;
  // Wait Until The Pulse Ends
  while(echo);
  // Turn OFF The Timer
  TMR1ON=0;
  // Calculate The Distance Using The Equation
  distance=TMR1/58.82;
  distance = distance * 10;
  return distance;
}

//CALCULATE A THRESHOLD FOR COUNTING
void thresholdf(void) //Calculating Threshold
{ 
    LEDL = 1;
    for(i=0;i<5;i++)
    {
        thres[i] = calc_dist();
        __delay_ms(200);
    }
    threshold = thres[0]+thres[1]+thres[2]+thres[3]+thres[4];
    threshold = threshold/5;
    maxthreshold = threshold + 4;
    minthreshold = threshold - 4; 
    minnthreshold = threshold - 10;
    __delay_ms(200);
    LEDL = 0;
}

//DUTY CYCLE VARIATION FUNCTION
void PWM1_Set_Duty(uint16_t DC) 
{
  CCP1CONbits.DC1B1 = D & 2; //10 bit value for Duty Cycle
  CCP1CONbits.DC1B0 = D & 1;
  CCPR1L = D >> 2;
}
   
void main(void) {
    
    TRISAbits.TRISA0 = 0;
    TRISAbits.TRISA1 = 0;
    TRISAbits.TRISA2 = 1;
    TRISAbits.TRISA3 = 0;
    TRISAbits.TRISA4 = 0;
    TRISAbits.TRISA5 = 0;
    
    TRISBbits.TRISB4 = 0;
    TRISBbits.TRISB5 = 0;
    TRISBbits.TRISB6 = 0;
    TRISBbits.TRISB7 = 0;
    RB4 = 0; //Output signal
    
    TRISCbits.TRISC0 = 0;
    TRISCbits.TRISC1 = 0;
    TRISCbits.TRISC2 = 0;
    TRISCbits.TRISC3 = 1;
    TRISCbits.TRISC4 = 0;
    TRISCbits.TRISC5 = 0;
    TRISCbits.TRISC6 = 1;
    TRISCbits.TRISC7 = 0;

    //CLEARING ALL ANALOG BITS
    ANSELbits.ANS0 = 0;
    ANSELbits.ANS1 = 0;
    ANSELbits.ANS2 = 0;
    ANSELbits.ANS3 = 0;
    ANSELbits.ANS4 = 0;
    ANSELbits.ANS5 = 0;
    ANSELbits.ANS6 = 0;
    ANSELbits.ANS7 = 0;
    ANSELHbits.ANS8 = 0;
    ANSELHbits.ANS9 = 0;
    ANSELHbits.ANS10 = 0;
    ANSELHbits.ANS11 = 0;
         
    PEIE = 1;  // Peripherals Interrupt Enable Bit
    GIE = 1;   // Global Interrupts Enable Bit
    INTEDG = 1; //INTERRRUPT LOW-HIGH EDGE TRIGGER
    INTE = 1; //INTERRPT ENABLE
    INTF = 0; //CLEAR FLAG
    
    //TIMER 0 CONFIGURATIONS
    TMR0 = 0; //15 overflows is 1 second, 1 overflow takes 65ms or 0.065 seconds
    T0CS = 1; //TOGGLE TIMER0 ON/OFF BY CHANGING CLOCK SOURCE TO WATCH DOG TIMRER(WHICH IS DISABLED) OR TIMER0 
    OPTION_REGbits.PS0 = 1; //PRESCALER 1:256
    OPTION_REGbits.PS1 = 1;
    OPTION_REGbits.PS2 = 1;
    PSA = 0; //DONT CARE
    T0IE = 1; //ENABLING INTERRUPT
    T0IF = 0; //CLEARING FLAG
    
    
    //TIMER 1 CONFIGURATIONS (USED FOR ULTRASONIC SENSOR)
    T1CONbits.TMR1ON = 0; //TIMER1 OFF
    TMR1 = 0;  //CLEARING TIMER1 REGISTER TO START COUNT FROM ZERO
    TMR1CS = 0; //CLOCK SOURCE AS INTERNAL CLOCK
    T1CKPS0 = 0; //TIMER1 PRESCALER
    T1CKPS1 = 0;
    TMR1IE = 1; //Timer1 Interrupt Enable Bit
    TMR1IF = 0; //Clear The Interrupt Flag Bit
    
    //TIMER 2 CONFIGURATIONS
    T2CONbits.T2CKPS0 = 1; //1:4 PRESCALE RATIO 
    T2CONbits.T2CKPS1 = 0; //1:1 POSTSCALE RATIO
    T2CONbits.TOUTPS0 = 0;
    T2CONbits.TOUTPS1 = 0;
    T2CONbits.TOUTPS2 = 0;
    T2CONbits.TOUTPS3 = 0;
    T2CONbits.TMR2ON = 0; //TIMER2 OFF
    
    //PWM MODE CONFIGURATIONS
    CCP1M3 = 1; //PWM mode
    CCP1M2 = 1;
    CCP1M1 = 0;
    CCP1M0 = 0;
    PR2 = 124; //SET PR2 VALUE FOR 5MHZ
    T2CONbits.TMR2ON = 1; //START TIMER2
 

    __delay_ms(2000); //WAIT 2 SECONDS FOR START UP
    PWM1_Set_Duty(0);
    dist = 0;
    LEDR = 0;
    LEDL = 0;
    thresholdf();   
    LEDR = 0;
    LEDL = 0;
    
    
    while(1)
    {
        reset:
        dist = calc_dist();  //MEASURING DISTANCE CONTINOUSLY UNTIL THRESHOLD IS CROSSED  
        if(dist <= minthreshold )
        {          
            lowcount = 0;highcount = 0;count=5;
            TMR0 = 0;
            T0IF = 0;
            T0CS = 0; //STARTING TIMER0, HENCEFORTH TAKES CARE BY THE INTERRUPT ROUTINE
            while(1)       
            {
                if(count == 0) 
                {
                    lowcount = lowcount - 5;
                    calcount = highcount + lowcount;
                    calhighcount = calcount * 5;
                    calhighcount = calhighcount/10;  
                    if(highcount >= calhighcount) //IF 50% OF THE COUNTS ARE HIGH COUNTS, IT COUNTS AS ONE.
                    {
                        lowcount = 0;highcount = 0;count=5;
                        LEDR = 1;
                        RB4 = 1; //Output signal
                        __delay_ms(100);
                        LEDR = 0;
                        D = 0; //led dimming
                        CCP1CONbits.DC1B1 = D & 2;
                        CCP1CONbits.DC1B0 = D & 1;
                        CCPR1L = D >> 2;
                        RB4 = 0; //Output signal
                        goto reset;
                    }
                    else{lowcount = 0;highcount = 0;count=5;goto reset;} //ELSE IGNORED
                }
            }
        } 
        else if(BUTTONR == 1) //RESET THE THRESHOLD
        {
            thresholdf();
            goto reset;
        }
    } 
    return;
}

   void interrupt ISR ()
   {
        if(T0IF)      //FUNTION WILL BE TRIGGERED FOR EVERY 65MS
        {
            dist = calc_dist();         
            if(dist <= minthreshold){highcount++;count = 5;D = 200;}
            if(dist > minthreshold && dist <= threshold){lowcount++;count--;D = 50;}
            if(dist < minnthreshold){D = 400;}
            if(count == 0){T0CS = 1;T0IF = 0;} //EXITING THE FUNCTION WHEN CONTINOUS THRESHOLD VALUE IS DETECTED
            CCP1CONbits.DC1B1 = D & 2;
            CCP1CONbits.DC1B0 = D & 1;
            CCPR1L = D >> 2;
            T0IF = 0; //CLEAR FLAG TO START FUNCTION AGAIN
        }
   }
        /*
        if(TMR2IF)   //15 = 1 second //TIME OUT FOR STEPPER
        {
            
            dist = calc_dist();
            if(dist <= minthreshold){highcount++;count++;LEDR = 1;LEDL = 0;}
            if(dist > minthreshold && dist <= threshold){lowcount++;count--;LEDR = 0;LEDL = 1;}
            if(count == 0){T2CONbits.TMR2ON = 0;TMR2IF = 0;}
            TMR2IF = 0;   
        }*/
        
       /*if(TMR1IF)
        {
            TMR1_C++;
            TMR1IF = 0;
        }
        if(CCP1IF)
        {
            if(Edge == 0) // On The Rising Edge Of Echo Signal
            {
                t1 = TMR1; // Save t1
                CCP1CON = 0x04; // Re-configure CCP1 To Capture On Falling Edge
                Edge++; // Flip The State For Next Interrupt  
            }
            else if(Edge == 1) // On The Falling Edge Of Echo Signal
            {
                t2 = TMR1; // Save t2
                // Calculate The Echo Pulse Width (tp)
                if(TMR1_C)
                tp = (t2+65536*TMR1_C) - t1;
                else
                tp = t2 - t1;
                // Convert tp From TMR1 Ticks To Real-Time & Calculate Distance
                Tp = tp*0.0000001666;
                Distance = (Tp / 2)*34000;
                CCP1CON = 0x05; // Re-configure CCP1 To Capture On Rising Edge
                Edge = 0; // Reset The State Variable
            }
            // Clear Flags For Next Interrupt
            TMR1_C = 0;
            CCP1IF = 0;
        }*/
    


                /*for(int i = 0; i < 200; i++)
                {
                    if(T0IF)      
                    {
                        T0IF = 0;  
                        dist = calc_dist();         
                        if(dist <= minthreshold){highcount++;count++;D=400;}
                        if(dist > minthreshold && dist <= threshold){lowcount++;count--;D=50;}
                        if(dist == threshold){D = 0;}
                        if(count == 0){T0CS = 1;T0IF = 0;}         
                        PWM1_Set_Duty();                                      
                    }
                }*/




//void TMR1_Init();
//void CCP1_Capture_Init();
//void Trigger_Ultrasonic();
//void Update_Display();


/*
void TMR1_Init()
{
  //Configure Timer1 To Operate In Timer Mode
  //Clear The Timer1 Register. To start counting from 0
  TMR1 = 0;
  //Choose the local clock source (timer mode)
  TMR1CS = 0;
  //Choose the desired prescaler ratio (1:2)
  T1CKPS0 = 1;
  T1CKPS1 = 0;
  //Switch ON Timer1 Module!
  TMR1ON = 1;
  //Interrupts Configurations
  TMR1IE = 1; // Timer1 Interrupt Enable Bit
  TMR1IF = 0; // Clear The Interrupt Flag Bit
  PEIE = 1; // Peripherals Interrupts Enable Bit
  GIE = 1; // Global Interrupts Enable Bit
}

void CCP1_Capture_Init()
{
  //Configure The CCP1 Module To Operate in Capture Mode
  CCP1CON = 0x05;
  //Enable CCP1 Interrupt
  CCP1IF = 0;
  CCP1IE = 1;
}

void Update_Display()
{
  LCD_Set_Cursor(1,1);
  LCD_Write_String("Distance Is:");
  LCD_Set_Cursor(2,1);
  sprintf(Buffer, " %.3f [cm]", Distance);
  LCD_Write_String(Buffer);
}

void Trigger_Ultrasonic()
{
  Trig = 1;
  __delay_us(10);
  Trig = 0;
}
 * void calculate_count(void)
{
    LEDL = 1;LEDR = 1;
    T2CONbits.TMR2ON = 0;
    TMR2IF = 0;
    TMR2 = 0;
    while(1)
    {
        dist = calc_dist(); 
        if(dist <= minthreshold)
        {
            highcount = 0;
            lowcount = 0;
            count = 30;
            calcount = 0;
            calhighcount = 0;
            T2CONbits.TMR2ON = 1;
            while(1)
            {
                if(count == 0)
                {
                    T2CONbits.TMR2ON = 0;
                    LEDL = 0;LEDR = 0;
                    calcount = highcount;
                    calhighcount = calcount * 6;
                    calhighcount = calhighcount/10;                   
                    TMR2 = 0;  //Start counting from 0
                    TMR2IF = 0; // Clear The Interrupt Flag Bit
                    break;
                }
            }
            break;
        }
    }
    __delay_ms(1000);
    LEDL = 0;LEDR = 0;
}
*/   

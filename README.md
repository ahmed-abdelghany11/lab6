#include "lab06.h"

#include <xc.h>
//do not change the order of the following 2 definitions
#define FCY 12800000UL
#include <libpic30.h>
#include <stdint.h>

#include <math.h>

#include "types.h"
#include "lcd.h"
#include "led.h"


/*
 * Parameter
 */



/*
 * Common Definitions
 */
#define TCKPS_1   0x00
#define TCKPS_8   0x01
#define TCKPS_64  0x02
#define TCKPS_256 0x03

#define PWM_MIN_US 1000
#define PWM_MID_US 1500
#define PWM_MAX_US 2000
#define PWM_CYC_US 20000

#define TOUCH_DIM_X 0
#define TOUCH_DIM_Y 1

uint16_t x, y;
/*
 * Timer Code
 */
void timer_initialize(void)
{
    //configure timer 2 for pwm (10ms period)
    CLEARBIT(T2CONbits.TON);
    CLEARBIT(T2CONbits.TCS);
    CLEARBIT(T2CONbits.TGATE);
    T2CONbits.TCKPS = TCKPS_64;
    TMR2 = 0;
    PR2 = 2000;
    
    IFS0bits.T2IF = 0;
    IPC1bits.T2IP = 1;
    IEC0bits.T2IE = 1;
}


/*
 * Servo Code
 */
void servo_initialize(void){

    timer_initialize();
    //configure output compare 7 (servo y)and  8(servo X)
    CLEARBIT(TRISDbits.TRISD6);
    CLEARBIT(TRISDbits.TRISD7);

    OC7R = 0;
    OC7RS = 0;
    OC8R = 0;
    OC8RS = 0;
    //SET PWM, no fault, timer2
    OC7CON= 0x0006;
    OC8CON= 0x0006;
            // SET TIMER 2
    SETBIT(T2CONbits.TON);
}

void servo_setduty(uint8_t servo, uint16_t duty_us){
    if (duty_us < 1000) duty_us = 1000;
    if (duty_us > 2000) duty_us = 2000;
    
    uint16_t pulseTicks = duty_us / 5; //5us per tick (64/Fcy)
    
    uint16_t invertedTicks = 4000 - pulseTicks; //20ms/5us = period ticks
    if (servo == 0) OC7RS = invertedTicks;
    else if (servo == 1) OC8RS = invertedTicks;
   
}

/*
 * Touch screen code
 */
void touchscreen_initialize(void)
{
    // Control pins E1,E2,E3 as outputs
    CLEARBIT(TRISEbits.TRISE1);
    CLEARBIT(TRISEbits.TRISE2);
    CLEARBIT(TRISEbits.TRISE3);

    // Safe state
    CLEARBIT(PORTEbits.RE1);
    Nop();
    SETBIT(PORTEbits.RE2);
    Nop();
    SETBIT(PORTEbits.RE3);

    // ADC off during config
    CLEARBIT(AD1CON1bits.ADON);

    //Set input pins
    SETBIT(TRISBbits.TRISB15);
    SETBIT(TRISBbits.TRISB9);
    
    // AN15 (X) and AN9 (Y) as analog
    CLEARBIT(AD1PCFGLbits.PCFG15);
    CLEARBIT(AD1PCFGLbits.PCFG9);
    
    // ADC basic config
    CLEARBIT(AD1CON1bits.AD12B);
    AD1CON1bits.FORM = 0;
    AD1CON1bits.SSRC = 0x7;
    AD1CON2 = 0;
    CLEARBIT(AD1CON3bits.ADRC);
    AD1CON3bits.SAMC = 0x1F;
    AD1CON3bits.ADCS = 0x2;

    // ADC on
    SETBIT(AD1CON1bits.ADON);
}

void touchscreen_set_dimension(uint8_t dim)
{
    CLEARBIT(AD1CON1bits.ADON);

    if (dim == TOUCH_DIM_X)
    {
        CLEARBIT(PORTEbits.RE1);
        Nop();
        SETBIT(PORTEbits.RE2);
        Nop();
        SETBIT(PORTEbits.RE3);
        Nop();

        // Select AN15 for X read
        AD1CHS0bits.CH0SA = 15;
    }
    else if (dim == TOUCH_DIM_Y)
    {
        SETBIT(PORTEbits.RE1);
        Nop();
        CLEARBIT(PORTEbits.RE2);
        Nop();
        CLEARBIT(PORTEbits.RE3);
        Nop();

        // Select AN9 for Y read
        AD1CHS0bits.CH0SA = 9;
    }
    
    else
    {
        SETBIT(PORTEbits.RE1);
        Nop();
        SETBIT(PORTEbits.RE2);
        Nop();
        SETBIT(PORTEbits.RE3);
        Nop();
    }
    
    SETBIT(AD1CON1bits.ADON);
    __delay_ms(10);
}

uint16_t touchscreen_read(void)
{
    SETBIT(AD1CON1bits.SAMP);
    while (!AD1CON1bits.DONE);
    CLEARBIT(AD1CON1bits.DONE);
    return ADC1BUF0;
}

/*
 * PD Controller
 */



/*
 * Butterworth Filter N=1, Cutoff 3 Hz, sampling @ 50 Hz
 */

void __attribute__((__interrupt__, auto_psv)) _T2Interrupt(void)
{
    IFS0bits.T2IF = 0;
    touchscreen_set_dimension(TOUCH_DIM_X);
    x = touchscreen_read();
    touchscreen_set_dimension(TOUCH_DIM_Y);
    y = touchscreen_read();
    lcd_locate(0, 6);
    lcd_printf("X/Y = %u/%u  ", x, y);
}

/*
 * main loop
 */
void main_loop(void)
{
    // print assignment information
    lcd_printf("Lab06: Amazing Ball");
    lcd_locate(0, 1);
    lcd_printf("Group: 5");
    lcd_locate(0, 2);
    
    servo_initialize();
    touchscreen_initialize();
    
    //0
    servo_setduty(0, 1500);
    servo_setduty(1, 1800);
    __delay_ms(5000);

    while (TRUE)
    {
       
        //1
        servo_setduty(0, PWM_MIN_US);
        servo_setduty(1, PWM_MIN_US);
        __delay_ms(5000);

        //2
        servo_setduty(0, PWM_MAX_US);
        servo_setduty(1, PWM_MIN_US);
        __delay_ms(5000);

        //3
        servo_setduty(0, PWM_MAX_US);
        servo_setduty(1, PWM_MAX_US);
        __delay_ms(5000);

        //4
        servo_setduty(0, PWM_MIN_US);
        servo_setduty(1, PWM_MAX_US);
        __delay_ms(5000);
    }
}

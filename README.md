#include "lab06.h"
#include <xc.h>
#include <libpic30.h>
#include <stdint.h>
#include <math.h>
#include "types.h"
#include "lcd.h"
#include "led.h"

// --------------------------------------------------------------
// Hardware & Timer Configuration
// --------------------------------------------------------------
#define FCY 12800000UL

#define TCKPS_1   0x00
#define TCKPS_8   0x01
#define TCKPS_64  0x02
#define TCKPS_256 0x03

// --------------------------------------------------------------
// Servo & PWM Configuration
// --------------------------------------------------------------
// 50Hz Period = 20ms. Prescaler 64 (5us/tick) => 4000 ticks.
#define PWM_PERIOD_TICKS 4000 
#define PWM_MIN_US 900
#define PWM_MAX_US 2100
#define PWM_CENTER_US 1500

// --------------------------------------------------------------
// Control Parameters (Lab Requirements)
// --------------------------------------------------------------
#define TOUCH_DIM_X 0
#define TOUCH_DIM_Y 1

#define CIRCLE_RADIUS 300.0f
#define CIRCLE_SPEED  2.0f      // Speed in Radians/second
#define CENTER_X      1600.0f   // Board Center X (Calibrate this)
#define CENTER_Y      1600.0f   // Board Center Y (Calibrate this)

[span_0](start_span)// PD Controller Gains[span_0](end_span)
#define KP_X  0.6f
#define KD_X  18.0f
#define KP_Y  0.6f
#define KD_Y  18.0f

// --------------------------------------------------------------
// Global Variables
// --------------------------------------------------------------
volatile uint8_t run_logic = 0;       // 100Hz trigger flag
volatile uint8_t processing_busy = 0; // Deadline check flag
volatile uint16_t deadline_misses = 0;// Deadline miss counter

uint8_t current_dim = TOUCH_DIM_X;
uint16_t tick_counter = 0;
float time_seconds = 0;

// Filter & Control State
float x_raw_prev = 0, y_raw_prev = 0;
float x_filt = 0, x_filt_prev = 0;
float y_filt = 0, y_filt_prev = 0;
float error_x_prev = 0, error_y_prev = 0;

/*
 * [span_1](start_span)Timer 1: 100Hz Interrupt (10ms)[span_1](end_span)
 * Controls the System Heartbeat
 */
void timer1_initialize(void)
{
    CLEARBIT(T1CONbits.TON);
    CLEARBIT(T1CONbits.TCS);
    CLEARBIT(T1CONbits.TGATE);
    
    // Fcy = 12.8Mhz. Prescaler 64 => 5us per tick.
    // 10ms = 2000 ticks.
    T1CONbits.TCKPS = TCKPS_64;
    TMR1 = 0;
    PR1 = 2000; 

    IPC0bits.T1IP = 4; // Priority
    IFS0bits.T1IF = 0; // Clear Flag
    IEC0bits.T1IE = 1; // Enable Interrupt
    
    SETBIT(T1CONbits.TON);
}

/*
 * [span_2](start_span)Servo Initialize: PWM Generation via Timer 2 (50Hz)[span_2](end_span)
 */
void servo_initialize(void)
{
    // Configure Timer 2 for PWM (20ms period)
    CLEARBIT(T2CONbits.TON);
    CLEARBIT(T2CONbits.TCS);
    CLEARBIT(T2CONbits.TGATE);
    T2CONbits.TCKPS = TCKPS_64;
    TMR2 = 0;
    PR2 = PWM_PERIOD_TICKS; // 4000 ticks
    
    // Configure Output Compare 7 (Y) and 8 (X)
    CLEARBIT(TRISDbits.TRISD6);
    CLEARBIT(TRISDbits.TRISD7);

    OC7R = 0; OC7RS = 0;
    OC8R = 0; OC8RS = 0;
    
    // PWM Mode
    OC7CON = 0x0006;
    OC8CON = 0x0006;
    
    SETBIT(T2CONbits.TON);
}

void servo_setduty(uint8_t servo, uint16_t duty_us)
{
    if (duty_us < PWM_MIN_US) duty_us = PWM_MIN_US;
    if (duty_us > PWM_MAX_US) duty_us = PWM_MAX_US;
    
    uint16_t pulseTicks = duty_us / 5; // 5us per tick
    
    [span_3](start_span)// Hardware Inversion Logic[span_3](end_span)
    uint16_t invertedTicks = PWM_PERIOD_TICKS - pulseTicks; 
    
    if (servo == TOUCH_DIM_X) OC8RS = invertedTicks;
    else if (servo == TOUCH_DIM_Y) OC7RS = invertedTicks;
}

/*
 * [span_4](start_span)Touchscreen Initialize[span_4](end_span)
 */
void touchscreen_initialize(void)
{
    // Control Pins
    CLEARBIT(TRISEbits.TRISE1);
    CLEARBIT(TRISEbits.TRISE2);
    CLEARBIT(TRISEbits.TRISE3);

    // Initial State
    SETBIT(PORTEbits.RE1);
    SETBIT(PORTEbits.RE2);
    SETBIT(PORTEbits.RE3);

    // ADC Config
    CLEARBIT(AD1CON1bits.ADON);
    SETBIT(TRISBbits.TRISB15); // X Input
    SETBIT(TRISBbits.TRISB9);  // Y Input
    
    CLEARBIT(AD1PCFGLbits.PCFG15);
    CLEARBIT(AD1PCFGLbits.PCFG9);
    
    CLEARBIT(AD1CON1bits.AD12B);
    AD1CON1bits.FORM = 0;
    AD1CON1bits.SSRC = 0x7; // Auto convert
    AD1CON2 = 0;            [span_5](start_span)// No Scanning[span_5](end_span)
    CLEARBIT(AD1CON3bits.ADRC);
    AD1CON3bits.SAMC = 0x1F;
    AD1CON3bits.ADCS = 0x2;

    SETBIT(AD1CON1bits.ADON);
}

/*
 * [span_6](start_span)Switch Dimension Configuration[span_6](end_span)
 * Does not delay; prepares MUX for the NEXT 10ms cycle.
 */
void touchscreen_switch_conf(uint8_t dim)
{
    if (dim == TOUCH_DIM_X)
    {
        // Setup for X
        CLEARBIT(PORTEbits.RE1); Nop();
        SETBIT(PORTEbits.RE2);   Nop();
        SETBIT(PORTEbits.RE3);   Nop();
        AD1CHS0bits.CH0SA = 15;
    }
    else
    {
        // Setup for Y
        SETBIT(PORTEbits.RE1);   Nop();
        CLEARBIT(PORTEbits.RE2); Nop();
        CLEARBIT(PORTEbits.RE3); Nop();
        AD1CHS0bits.CH0SA = 9;
    }
}

uint16_t touchscreen_read_now(void)
{
    SETBIT(AD1CON1bits.SAMP);
    while (!AD1CON1bits.DONE);
    CLEARBIT(AD1CON1bits.DONE);
    return ADC1BUF0;
}

/*
 * [span_7](start_span)Butterworth Filter 1st Order (3Hz cutoff @ 50Hz)[span_7](end_span)
 */
float butterworth_filter(float raw_new, float *raw_prev, float *y_prev) {
    float b = 0.1602f;
    float a = 0.6795f;
    float y_new = (b * raw_new) + (b * (*raw_prev)) + (a * (*y_prev));
    *raw_prev = raw_new;
    *y_prev = y_new;
    return y_new;
}

/*
 * [span_8](start_span)Timer 1 Interrupt (100Hz)[span_8](end_span)
 */
void __attribute__((__interrupt__, auto_psv)) _T1Interrupt(void)
{
    IFS0bits.T1IF = 0;
    
    if (processing_busy) {
        deadline_misses++;
    }
    
    run_logic = 1; // Trigger main loop
}

/*
 * [span_9](start_span)Main Loop[span_9](end_span)
 */
void main_loop(void)
{
    // Setup
    servo_initialize();
    touchscreen_initialize();
    lcd_initialize();
    lcd_clear();
    
    // Initial Config
    current_dim = TOUCH_DIM_X;
    touchscreen_switch_conf(TOUCH_DIM_X);
    __delay_ms(20); 
    
    timer1_initialize();

    while (1)
    {
        [span_10](start_span)// Wait for 10ms Tick[span_10](end_span)
        while (!run_logic);
        run_logic = 0;
        
        processing_busy = 1; // Start Deadline Check
        
        // --- 1. Read (100Hz) ---
        uint16_t adc_val = touchscreen_read_now();
        uint8_t just_read_dim = current_dim;
        
        [span_11](start_span)// --- 2. Switch for NEXT 10ms[span_11](end_span) ---
        if (current_dim == TOUCH_DIM_X) {
            touchscreen_switch_conf(TOUCH_DIM_Y);
            current_dim = TOUCH_DIM_Y;
        } else {
            touchscreen_switch_conf(TOUCH_DIM_X);
            current_dim = TOUCH_DIM_X;
        }

        [span_12](start_span)// --- 3. Control (50Hz - Every 2nd Tick)[span_12](end_span) ---
        if (tick_counter % 2 == 0)
        {
            if (just_read_dim == TOUCH_DIM_X) {
                x_filt = butterworth_filter((float)adc_val, &x_raw_prev, &x_filt_prev);
            } else {
                y_filt = butterworth_filter((float)adc_val, &y_raw_prev, &y_filt_prev);
            }
            
            [span_13](start_span)// Circle Setpoint[span_13](end_span)
            time_seconds += 0.02f;
            float target_x = CENTER_X + CIRCLE_RADIUS * cos(CIRCLE_SPEED * time_seconds);
            float target_y = CENTER_Y + CIRCLE_RADIUS * sin(CIRCLE_SPEED * time_seconds);
            
            // PD Control X
            float error_x = target_x - x_filt;
            float deriv_x = error_x - error_x_prev;
            error_x_prev = error_x;
            float out_x = (KP_X * error_x) + (KD_X * deriv_x);
            servo_setduty(TOUCH_DIM_X, (uint16_t)(PWM_CENTER_US + out_x));
            
            // PD Control Y
            float error_y = target_y - y_filt;
            float deriv_y = error_y - error_y_prev;
            error_y_prev = error_y;
            float out_y = (KP_Y * error_y) + (KD_Y * deriv_y);
            servo_setduty(TOUCH_DIM_Y, (uint16_t)(PWM_CENTER_US + out_y));
        }

        [span_14](start_span)// --- 4. Reporting (5Hz - Every 20th Tick)[span_14](end_span) ---
        if (tick_counter % 20 == 0)
        {
            lcd_locate(0, 0);
            lcd_printf("Lab 6 - Circle");
            lcd_locate(0, 1);
            lcd_printf("Misses: %u", deadline_misses);
            lcd_locate(0, 2);
            lcd_printf("X:%4.0f Y:%4.0f", (double)x_filt, (double)y_filt);
        }
        
        tick_counter++;
        processing_busy = 0; // End Deadline Check
    }
}

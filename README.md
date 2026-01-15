#include "lab06.h"
#include <xc.h>
#include <libpic30.h>
#include <stdint.h>
#include <math.h>
#include "types.h"
#include "lcd.h"
#include "led.h"

// --------------------------------------------------------------
// Hardware & Timer Config
// --------------------------------------------------------------
#define FCY 12800000UL

#define TCKPS_1   0x00
#define TCKPS_8   0x01
#define TCKPS_64  0x02
#define TCKPS_256 0x03

// --------------------------------------------------------------
// Servo & PWM Config
// --------------------------------------------------------------
// Servos operate best at 50Hz (20ms period). 
// With Prescaler 64, 1 tick = 5us. 
// 20ms = 4000 ticks. Center (1.5ms) = 300 ticks.
#define PWM_PERIOD_TICKS 4000 
#define PWM_MIN_US 900
#define PWM_MAX_US 2100
#define PWM_CENTER_US 1500

// --------------------------------------------------------------
[span_1](start_span)// Control Constants (Configurable)[span_1](end_span)
// --------------------------------------------------------------
#define TOUCH_DIM_X 0
#define TOUCH_DIM_Y 1

#define CIRCLE_RADIUS 300.0f
#define CIRCLE_SPEED  2.0f      // Speed of circle in Radians/sec
#define CENTER_X      1600.0f   // Calibrate this to your board's center!
#define CENTER_Y      1600.0f   // Calibrate this to your board's center!

[span_2](start_span)// PID Gains - TUNING REQUIRED[span_2](end_span)
#define KP_X  0.6f
#define KD_X  18.0f
#define KP_Y  0.6f
#define KD_Y  18.0f

// --------------------------------------------------------------
// Global Variables
// --------------------------------------------------------------
volatile uint8_t run_logic = 0;       // Flag to trigger main loop
volatile uint8_t processing_busy = 0; // Flag for deadline monitoring
[span_3](start_span)volatile uint16_t deadline_misses = 0;// Counter[span_3](end_span)

uint8_t current_dim = TOUCH_DIM_X;
uint16_t tick_counter = 0;
float time_seconds = 0;

// Filter & Control Variables
float x_raw_prev = 0, y_raw_prev = 0;
float x_filt = 0, x_filt_prev = 0;
float y_filt = 0, y_filt_prev = 0;
float error_x_prev = 0, error_y_prev = 0;

/*
 * Timer 1 Initialize: System Heartbeat (100Hz)
 * [span_4](start_span)Reads touchscreen every 10ms[span_4](end_span)
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
 * Servo Initialize: PWM Generation (Timer 2)
 */
void servo_initialize(void)
{
    // Configure Timer 2 for PWM (20ms period / 50Hz)
    CLEARBIT(T2CONbits.TON);
    CLEARBIT(T2CONbits.TCS);
    CLEARBIT(T2CONbits.TGATE);
    T2CONbits.TCKPS = TCKPS_64;
    TMR2 = 0;
    PR2 = PWM_PERIOD_TICKS; // 4000 ticks = 20ms
    
    // Configure Output Compare 7 (Y) and 8 (X)
    CLEARBIT(TRISDbits.TRISD6);
    CLEARBIT(TRISDbits.TRISD7);

    OC7R = 0; OC7RS = 0;
    OC8R = 0; OC8RS = 0;
    
    // PWM Mode, no fault protection
    OC7CON = 0x0006;
    OC8CON = 0x0006;
    
    SETBIT(T2CONbits.TON);
}

void servo_setduty(uint8_t servo, uint16_t duty_us)
{
    // Safety Limits
    if (duty_us < PWM_MIN_US) duty_us = PWM_MIN_US;
    if (duty_us > PWM_MAX_US) duty_us = PWM_MAX_US;
    
    uint16_t pulseTicks = duty_us / 5; // 5us per tick
    
    // Hardware Inversion: 
    // The hardware inverts the signal, so we want the output to be LOW 
    // for the duration of the pulse.
    // In PWM mode, output is High when TMR < OCxRS. 
    // We want LOW for `pulseTicks`.
    uint16_t invertedTicks = PWM_PERIOD_TICKS - pulseTicks; 
    
    if (servo == TOUCH_DIM_X) OC8RS = invertedTicks;
    else if (servo == TOUCH_DIM_Y) OC7RS = invertedTicks;
}

/*
 * Touchscreen Initialize
 */
void touchscreen_initialize(void)
{
    // Logic Pins E1, E2, E3
    CLEARBIT(TRISEbits.TRISE1);
    CLEARBIT(TRISEbits.TRISE2);
    CLEARBIT(TRISEbits.TRISE3);

    // Initial Safe State
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
    AD1CON2 = 0;
    CLEARBIT(AD1CON3bits.ADRC);
    AD1CON3bits.SAMC = 0x1F;
    AD1CON3bits.ADCS = 0x2;

    SETBIT(AD1CON1bits.ADON);
}

/*
 * Configure MUX for the NEXT reading
 * [span_5](start_span)Allows the hardware 10ms to settle before we read it[span_5](end_span)
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
 * Butterworth Filter 1st Order
 * Cutoff: 3Hz, Sample Rate: 50Hz
 * y[n] = 0.160*x[n] + 0.160*x[n-1] + 0.679*y[n-1]
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
 * Timer 1 Interrupt (100Hz)
 * Checks deadlines and triggers main loop
 */
void __attribute__((__interrupt__, auto_psv)) _T1Interrupt(void)
{
    IFS0bits.T1IF = 0;
    
    [span_6](start_span)// Check if the main loop finished the previous job[span_6](end_span)
    if (processing_busy) {
        deadline_misses++;
    }
    
    run_logic = 1; // Unblock main loop
}

/*
 * Main Function
 */
int main(void)
{
    // Initialize Hardware
    servo_initialize();
    touchscreen_initialize();
    lcd_initialize();
    lcd_clear();
    
    // Initial State: Setup X and wait for settle
    current_dim = TOUCH_DIM_X;
    touchscreen_switch_conf(TOUCH_DIM_X);
    __delay_ms(20); 
    
    // Start System Timer (100Hz)
    timer1_initialize();

    while (1)
    {
        [span_7](start_span)// Wait for 10ms (100Hz) trigger[span_7](end_span)
        while (!run_logic);
        run_logic = 0;
        
        processing_busy = 1; // Start processing time (for deadline check)
        
        // ----------------------------------------------------
        // 100 Hz Zone: Read & Switch
        // ----------------------------------------------------
        
        // 1. READ: The signal is stable because we waited 10ms since the last switch
        uint16_t adc_val = touchscreen_read_now();
        
        // Store which dimension we JUST read for the filter later
        uint8_t just_read_dim = current_dim;
        
        [span_8](start_span)// 2. SWITCH: Change MUX now so it settles for the NEXT 10ms[span_8](end_span)
        if (current_dim == TOUCH_DIM_X) {
            touchscreen_switch_conf(TOUCH_DIM_Y);
            current_dim = TOUCH_DIM_Y;
        } else {
            touchscreen_switch_conf(TOUCH_DIM_X);
            current_dim = TOUCH_DIM_X;
        }

        // ----------------------------------------------------
        // 50 Hz Zone: Control Loop (Runs every 2nd tick)
        // ----------------------------------------------------
        if (tick_counter % 2 == 0)
        {
            // Update Filter for the dimension we just read
            if (just_read_dim == TOUCH_DIM_X) {
                x_filt = butterworth_filter((float)adc_val, &x_raw_prev, &x_filt_prev);
            } else {
                y_filt = butterworth_filter((float)adc_val, &y_raw_prev, &y_filt_prev);
            }
            
            [span_9](start_span)// 1. Generate Circle Setpoint[span_9](end_span)
            time_seconds += 0.02f; // 1/50Hz = 0.02s
            float target_x = CENTER_X + CIRCLE_RADIUS * cos(CIRCLE_SPEED * time_seconds);
            float target_y = CENTER_Y + CIRCLE_RADIUS * sin(CIRCLE_SPEED * time_seconds);
            
            [span_10](start_span)// 2. PD Control X[span_10](end_span)
            float error_x = target_x - x_filt;
            float deriv_x = error_x - error_x_prev;
            error_x_prev = error_x;
            float out_x = (KP_X * error_x) + (KD_X * deriv_x);
            servo_setduty(TOUCH_DIM_X, (uint16_t)(PWM_CENTER_US + out_x));
            
            [span_11](start_span)// 3. PD Control Y[span_11](end_span)
            float error_y = target_y - y_filt;
            float deriv_y = error_y - error_y_prev;
            error_y_prev = error_y;
            float out_y = (KP_Y * error_y) + (KD_Y * deriv_y);
            servo_setduty(TOUCH_DIM_Y, (uint16_t)(PWM_CENTER_US + out_y));
        }

        // ----------------------------------------------------
        // 5 Hz Zone: LCD Printing (Runs every 20th tick)
        // ----------------------------------------------------
        if (tick_counter % 20 == 0)
        {
            lcd_locate(0, 0);
            lcd_printf("Lab 6 - Circle");
            lcd_locate(0, 1);
            lcd_printf("Misses: %u", deadline_misses); [span_12](start_span)//[span_12](end_span)
            lcd_locate(0, 2);
            lcd_printf("X:%4.0f Y:%4.0f", (double)x_filt, (double)y_filt);
        }
        
        tick_counter++;
        processing_busy = 0; // Finished processing
    }
    
    return 0;
}

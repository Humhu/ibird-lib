/*********************************************
* Name: motor_ctrl.c
* Desc: Motor Controller (PWM)
* Date: 2010-05-30
* Author: stanbaek
*********************************************/

#include "motor_ctrl.h"
#include "pwm.h"
#include "ports.h"
#include "led.h"

//#include "lcd.h"
//#include <stdio.h>


#define LEFT_TURN           _LATE2 = 1; _LATE4 = 0
#define RIGHT_TURN          _LATE2 = 0; _LATE4 = 1
#define NO_TURN             _LATE2 = 0; _LATE4 = 0

#define LEFT_TURN_CHANNEL   2
#define RIGHT_TURN_CHANNEL  3

#define LED_LEFT_TURN       LED_GREEN = 1; LED_RED = 0
#define LED_RIGHT_TURN      LED_GREEN = 0; LED_RED = 1
#define LED_NO_TURN         LED_GREEN = 0; LED_RED = 0


static unsigned int pwmPeriod;
static unsigned char steerMode = MC_STEER_MODE_CONT;

static void mcSetupPeripheral(void);


void mcSetup(void) {

    mcSetupPeripheral();

}

void mcSetDutyCycle(unsigned char channel, float duty_cycle) {

    unsigned int pdc_value;
    pdc_value = (unsigned int)(2*duty_cycle/100*pwmPeriod);
    SetDCMCPWM(channel, pdc_value, 0);

}

void mcThrust(float value) {

    mcSetDutyCycle(MC_CHANNEL_PWM1, value);

}

void mcSteer(float value) {

    //char lcdstr[80];

    if (steerMode == MC_STEER_MODE_CONT) {
        if (value > 0) {
            mcSetDutyCycle(RIGHT_TURN_CHANNEL, 0);
            mcSetDutyCycle(LEFT_TURN_CHANNEL, value);
        } else if (value < 0) {
            mcSetDutyCycle(LEFT_TURN_CHANNEL, 0);
            mcSetDutyCycle(RIGHT_TURN_CHANNEL, -value);
        } else {
            PDC2 = 0;
            PDC3 = 0;
        }

    } else {
        if (value > 0) { 
            NO_TURN;
            RIGHT_TURN;
        } else if (value < 0) { 
            NO_TURN;
            LEFT_TURN;
        } else { 
            NO_TURN;
        }
    }

}


void mcSetSteerMode(unsigned char mode) {

    if (mode == MC_STEER_MODE_DISC) {
        steerMode = MC_STEER_MODE_DISC;
        // PWM2L & PWM3L pins are general I/O
        PWMCON1bits.PEN2L = 0;
        PWMCON1bits.PEN3L = 0;
    } else {
        steerMode = MC_STEER_MODE_CONT;
        // PWM2L & PWM3L pins are enabled for PWM output
        PWMCON1bits.PEN2L = 1;
        PWMCON1bits.PEN3L = 1;
    }

}

static void mcSetupPeripheral(void) {

    // For 1KHz at MIPS == 40
    pwmPeriod = 624; 
    
    ConfigIntMCPWM(PWM_INT_DIS & PWM_FLTA_DIS_INT & PWM_FLTB_DIS_INT);
    
    PDC1 = 0;   // duty cycle = 0
    PDC2 = 0;   // duty cycle = 0
    PDC3 = 0;   // duty cycle = 0
    PDC4 = 0;   // duty cycle = 0

    PTPER = pwmPeriod;

    SEVTCMP = 1; // Special Event Trigger Compare Value for ADC in phase with PWM

    PWMCON1bits.PMOD1 = 1;
    PWMCON1bits.PMOD2 = 1;
    PWMCON1bits.PMOD3 = 1;
    PWMCON1bits.PMOD4 = 0;

    PWMCON1bits.PEN1H = 0;
    PWMCON1bits.PEN2H = 0;
    PWMCON1bits.PEN3H = 0;
    PWMCON1bits.PEN4H = 0;
    PWMCON1bits.PEN1L = 1;
    PWMCON1bits.PEN2L = 1;
    PWMCON1bits.PEN3L = 1;
    PWMCON1bits.PEN4L = 0;

    PWMCON2bits.SEVOPS = 0; // postscale 1:1
    PWMCON2bits.OSYNC = 0;
    PWMCON2bits.IUE = 0;

    PTCONbits.PTMOD = 0; // Free running mode
    PTCONbits.PTOPS = 0; // postscale 1:1
    PTCONbits.PTCKPS = 0b11; // postscale 1:64
    PTCONbits.PTSIDL = 0; // runs in CPU idle mode
    PTCONbits.PTEN = 1;

}




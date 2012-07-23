/*
* Copyright (c) 2010 - 2012, Regents of the University of California
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* - Redistributions of source code must retain the above copyright notice,
*   this list of conditions and the following disclaimer.
* - Redistributions in binary form must reproduce the above copyright notice,
*   this list of conditions and the following disclaimer in the documentation
*   and/or other materials provided with the distribution.
* - Neither the name of the University of California, Berkeley nor the names
*   of its contributors may be used to endorse or promote products derived
*   from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
*
* PWM Motor Controller Drive
*
* by Stanley S. Baek
*
* v 0.2
*
* Revisions:
*   Stanley S. Baek     2010-05-30      Initial release
*   Humphrey Hu         2012-06-30      Switched to +- 1.0 scale
*/

#include "motor_ctrl.h"
#include "pwm.h"
#include "ports.h"
#include "led.h"

#define LEFT_TURN           _LATE2 = 1; _LATE4 = 0;
#define RIGHT_TURN          _LATE2 = 0; _LATE4 = 1;
#define NO_TURN             _LATE2 = 0; _LATE4 = 0;

#define LEFT_TURN_CHANNEL   (2)
#define RIGHT_TURN_CHANNEL  (3)

#define DC_UPPER_LIMIT      (1.0)
#define DC_LOWER_LIMIT      (0.0)

// =========== Static Variables ================================================
static unsigned char is_ready = 0;
static unsigned int pwmPeriod;
static McSteerMode steerMode = MC_STEER_MODE_CONT;

// =========== Function Stubs ==================================================
static void mcSetupPeripheral(void);

// =========== Public Functions ================================================
void mcSetup(void) {
    
    pwmPeriod = 624;    // For 1KHz at MIPS == 40    
    mcSetupPeripheral();
    is_ready = 1;
    
}

void mcStop(void) {   
    
    mcSetDutyCycle(MC_CHANNEL_PWM1, 0.0);
    mcSetDutyCycle(MC_CHANNEL_PWM2, 0.0);
    mcSetDutyCycle(MC_CHANNEL_PWM3, 0.0);
    mcSetDutyCycle(MC_CHANNEL_PWM4, 0.0);

}

void mcSetDutyCycle(unsigned char channel, float duty_cycle) {

    unsigned int pdc_value;

    if(!is_ready) { return; }

    // Check input range
    if(duty_cycle > DC_UPPER_LIMIT) {
        duty_cycle = DC_UPPER_LIMIT;
    } else if(duty_cycle < DC_LOWER_LIMIT) {
        duty_cycle = DC_LOWER_LIMIT;
    }
    
    pdc_value = (unsigned int)(2*duty_cycle*pwmPeriod);
    SetDCMCPWM(channel, pdc_value, 0);

}

void mcThrust(float value) {

    mcSetDutyCycle(MC_CHANNEL_PWM1, value);

}

void mcSteer(float value) {

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


void mcSetSteerMode(McSteerMode mode) {

    if (mode == MC_STEER_DISC) {
        steerMode = MC_STEER_DISC;
        // PWM2L & PWM3L pins are general I/O
        PWMCON1bits.PEN2L = 0;
        PWMCON1bits.PEN3L = 0;
    } else {
        steerMode = MC_STEER_CONT;
        // PWM2L & PWM3L pins are enabled for PWM output
        PWMCON1bits.PEN2L = 1;
        PWMCON1bits.PEN3L = 1;
    }

}

static void mcSetupPeripheral(void) {    
    
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

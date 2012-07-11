/*
* Copyright (c) 2009 - 2010, Regents of the University of California
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
* Synchronous Software Servo (RC PWM) Module
*
* by Humphrey Hu
*
* v.alpha
*
* Revisions:
*  Humphrey Hu             2011-9-23      Initial implementation
*  Humphrey Hu             2012-6-21      Switched to normalized float inputs
*
*/

#include "sync_servo.h"
#include "utils.h"
#include "timer.h"

#define SERVO_A                 _LATE1

#define FCY                     (40000000)
#define PWM_FREQUENCY           (50)
#define PWM_PERIOD              (12500) // 40 MIPS/(prescale * frequency)

#define PULSE_MAX_LENGTH        (1250)
#define PULSE_MIN_LENGTH        (625)
#define PULSE_AMPLITUDE         ((PULSE_MAX_LENGTH - PULSE_MIN_LENGTH)/2) // 312
#define PULSE_ZERO_LENGTH       ((PULSE_MAX_LENGTH + PULSE_MIN_LENGTH)/2) // 937

// =========== Function Prototypes ============================================ 
static void setupTimer4(unsigned int frequency);

// =========== Static Variables ===============================================
static unsigned char pin_is_high;
static unsigned int pulse_length, setpoint;
static int current_setpoint;

// =========== Public Methods =================================================
void servoSetup(void) {

    setupTimer4(PWM_FREQUENCY);

    pin_is_high = 0;
    SERVO_A = 0;
    current_setpoint = 0;
    setpoint = PULSE_ZERO_LENGTH;
    pulse_length = PULSE_ZERO_LENGTH;

    servoStop();

}

// Set is in +- 1.0, Track is in +- 312
void servoSet(float set) {

    if(set > 1.0) { set = 1.0; }
    else if(set < -1.0) { set = -1.0; }
    
    setpoint = (unsigned int)(PULSE_ZERO_LENGTH + (int)(set*PULSE_AMPLITUDE));

}

void servoStart(void) {

    WriteTimer4(0);
    _T4IE = 1;

}

void servoStop(void) {

    _T4IE = 0;
}

// =========== Private Functions ===============================================

void __attribute__((interrupt, no_auto_psv)) _T4Interrupt(void) {

    if(pin_is_high) {
        
        SERVO_A = 0; // End pulse
        pin_is_high = 0;
        
        WriteTimer4(0); // Wait until next pulse start
        PR4 = PWM_PERIOD - pulse_length;
    } else {
                
        pulse_length = setpoint; // Buffer pulse length
        WriteTimer4(0);
        PR4 = pulse_length;
        
        SERVO_A = 1; // Begin pulse
        pin_is_high = 1;
    }

    _T4IF = 0;
    
}

void setupTimer4(unsigned int frequency) {

    unsigned int con_reg, period;

    // prescale 1:64
    con_reg =     T4_ON & 
    T4_IDLE_STOP & 
    T4_GATE_OFF & 
    T4_PS_1_64 & 
    T4_SOURCE_INT &
    T4_32BIT_MODE_OFF;

    // period value = Fcy/(prescale*Ftimer)
    period = FCY/(64*frequency); 

    OpenTimer4(con_reg, period);
    ConfigIntTimer4(T4_INT_PRIOR_5 & T4_INT_ON);

}

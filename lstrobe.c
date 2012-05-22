/**
 * Copyright (c) 2012, Regents of the University of California
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
 * LED Strobe Timer
 *
 * by Humphrey Hu
 *
 * v.beta
 *
 * Revisions:
 *  Humphrey Hu		2012-04-25		Initial implementation 
 *                      
 */

#include "lstrobe.h"
#include "led.h"
#include "timer.h"
#include "sys_clock.h"
#include "cam.h"

#define DEFAULT_ON_TIME         (0)
#define DEFAULT_OFF_TIME        (0)
 
#define STROBE_ON               (1)
#define STROBE_OFF              (0)
#define STROBE                  (LED_IR)

#define RUNS_BEFORE_CALIB       (50)

typedef enum {
    LT_ON = 0,
    LT_OFF,
    LT_PL,
}LTimerState; 
 
static LTimerState state;
static unsigned char is_ready = 0;
static LStrobeParamStruct target;
static unsigned int runs;

static void phaseLock(LStrobeParam param);
static void setupTimer3(void);

void lstrobeSetup(void) {

    setupTimer3();

    STROBE = STROBE_ON;
    state = LT_OFF;

    runs = 0;
    is_ready = 1;
    
} 
 
void lstrobeSetParam(LStrobeParam param) {

    target.period = param->period;
    target.period_offset = param->period_offset;
    target.on_time = param->on_time;
    target.off_time = param->off_time;

}

void lstrobeGetParam(LStrobeParam param) {

    param->period = target.period;
    param->period_offset = target.period_offset;
    param->on_time = target.on_time;
    param->off_time = param->off_time;

}

void lstrobeStart(void) {

    if(!is_ready) { return; }
    phaseLock(&target);
    
    
}
 
// ====== Private Functions ===================================================
 
void __attribute__((interrupt, no_auto_psv)) _T3Interrupt(void) {

    if(state == LT_PL) {
        state = LT_OFF; // Once synchronized, proceed to OFF state immediately
    }

    if(state == LT_ON) {
        STROBE = STROBE_OFF;
        LED_RED = 0;
        PR3 = target.off_time;
        state = LT_OFF;

    } else if(state == LT_OFF) {
        STROBE = STROBE_ON;
        LED_RED = 1;
        PR3 = target.on_time;
        state = LT_ON;
    } 

    _T3IF = 0;

}
 
static void phaseLock(LStrobeParam params) {      
    
    DisableIntT3;
    
    PR3 = params->period + params->period_offset - 
            sclockGetGlobalTicks() % params->period;
    state = LT_PL;

    EnableIntT3;   
    
}
 
/**
 * LED strobe timer setup
 */
static void setupTimer3(void) {

    unsigned int con_reg;

    con_reg =   T3_OFF &            // Disable module until setup complete
                T3_IDLE_CON &       // Continue running when idle
                T3_GATE_OFF &       // Time accumulation disable
                T3_PS_1_256 &        // Prescale 1:64
                T3_SOURCE_INT;      // Internal clock

    _T3IF = 0;
    OpenTimer3(con_reg, 0);         // Configure timer
    ConfigIntTimer3(T3_INT_PRIOR_6 & T3_INT_OFF);
    T3CONbits.TON = 1;              // Enable module

}



/*
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
* Rate-Based Reference Generator
*
* by Humphrey Hu
*
* v.0.1
*
* Revisions: 
*  Humphrey Hu		    2012-07-09       Initial implementation
*/

#include "rate.h"
#include "quat.h"
#include "regulator.h"
#include "bams.h"
#include "attitude.h"

#include <stdlib.h>
#include <string.h>
#include <math.h>

// =========== Static Variables ================================================
static unsigned char is_ready = 0, is_running = 0;
static float period;

static RateStruct global_rate, body_rate;
static Quaternion global_displacement, body_displacement;

// =========== Function Stubs ==================================================
static void generateDisplacement(Rate rate, Quaternion *q);

// =========== Public Methods ==================================================
void rateSetup(float ts) {

    period = ts;
    
    global_rate.yaw_rate = 0.0;
    global_rate.pitch_rate = 0.0;
    global_rate.roll_rate = 0.0;   
    generateDisplacement(&global_rate, &global_displacement);
    
    body_rate.yaw_rate = 0.0;
    body_rate.pitch_rate = 0.0;
    body_rate.roll_rate = 0.0;
    generateDisplacement(&body_rate, &body_displacement);
    
    is_running = 0;
    is_ready = 1;
    
}

void rateSetGlobalSlew(Rate rate) {

    if(rate == NULL) { return; }
    
    memcpy(&global_rate, rate, sizeof(RateStruct));    
    generateDisplacement(&global_rate, &global_displacement);
    
}

void rateSetBodySlew(Rate rate) {

    if(rate == NULL) { return; }
    
    memcpy(&body_rate, rate, sizeof(RateStruct));
    generateDisplacement(&body_rate, &body_displacement);

}

void rateEnable(void) {
    is_running = 1;
}

void rateDisable(void) {
    is_running = 0;
}

void rateApplyGlobalRotation(Quaternion *rot) {

    Quaternion current_ref;
    
    rgltrGetQuatRef(&current_ref);
    quatMult(rot, &current_ref, &current_ref);
    quatNormalize(&current_ref);
    rgltrSetQuatRef(&current_ref);

}

void rateApplyLocalRotation(Quaternion *rot) {

    Quaternion current_ref;
    
    rgltrGetQuatRef(&current_ref);
    quatMult(&current_ref, rot, &current_ref);
    quatNormalize(&current_ref);
    rgltrSetQuatRef(&current_ref);

}

void rateProcess(void) {

    Quaternion current_ref;

    if(!is_ready || !is_running) { return; }
    
    rgltrGetQuatRef(&current_ref);    

    // q_body*q_current*q_global    
    quatMult(&global_displacement, &current_ref, &current_ref);
    quatMult(&current_ref, &body_displacement, &current_ref);

    quatNormalize(&current_ref);

    rgltrSetQuatRef(&current_ref);

}

// =========== Private Methods =================================================

void generateDisplacement(Rate rate, Quaternion *q) {
    
    float norm, sina_2;
    bams32_t a_2;
    
    norm = sqrtf(   rate->yaw_rate*rate->yaw_rate +
                    rate->pitch_rate*rate->pitch_rate +
                    rate->roll_rate*rate->roll_rate   );

    if(norm == 0.0) {
        q->w = 1.0;
        q->x = 0.0;
        q->y = 0.0;
        q->z = 0.0;
    } else {
        a_2 = floatToBams32Rad(norm*period)/2;
        sina_2 = bams32SinFine(a_2);

        q->w = bams32CosFine(a_2)*norm;
        q->x = sina_2*rate->roll_rate;
        q->y = sina_2*rate->pitch_rate;
        q->z = sina_2*rate->yaw_rate;
    }
    quatNormalize(q);    
    
}
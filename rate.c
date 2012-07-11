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
* I-Bird Rate-Based Reference Generator
*
* by Humphrey Hu
*
* v.beta
*
* Revisions: 
*  Humphrey Hu		    2012-07-09       Initial implementation
*
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

static RateStruct current_rate;
static Quaternion displacement_quat;

// =========== Function Stubs ==================================================

// =========== Public Methods ==================================================
void rateSetup(float ts) {

    period = ts;
    current_rate.yaw_rate = 0.0;
    current_rate.pitch_rate = 0.0;
    current_rate.roll_rate = 0.0;   
    
    is_running = 0;
    is_ready = 1;
    
}

void rateSet(Rate rate) {

    float norm, a_2, sina_2;

    if(rate == NULL) { return; }
    
    memcpy(&current_rate, rate, sizeof(RateStruct));    

    norm = sqrtf(   rate->yaw_rate*rate->yaw_rate +
                    rate->pitch_rate*rate->pitch_rate +
                    rate->roll_rate*rate->roll_rate   );

    if(norm == 0.0) {
        displacement_quat.w = 1.0;
        displacement_quat.x = 0.0;
        displacement_quat.y = 0.0;
        displacement_quat.z = 0.0;
    } else {
        a_2 = floatToBams32Rad(norm*period)/2;
        sina_2 = bams32SinFine(a_2);

        displacement_quat.w = bams32CosFine(a_2)*norm;
        displacement_quat.x = sina_2*rate->roll_rate;
        displacement_quat.y = sina_2*rate->pitch_rate;
        displacement_quat.z = sina_2*rate->yaw_rate;
    }
    quatNormalize(&displacement_quat);
    
}

void rateEnable(void) {
    is_running = 1;
}

void rateDisable(void) {
    is_running = 0;
}

void rateProcess(void) {

    Quaternion current_ref;

    if(!is_ready || !is_running) { return; }
    
    rgltrGetQuatRef(&current_ref);
    quatMult(&current_ref, &displacement_quat, &current_ref);
    quatNormalize(&current_ref);
    rgltrSetQuatRef(&current_ref);

}

// =========== Private Methods =================================================

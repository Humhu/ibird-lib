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
* Reference Slew Rate Limiter
*
* by Humphrey Hu
*
* v.0.1
*
* Revisions: 
*  Humphrey Hu		    2012-07-22       Initial implementation
*/

#include "slew.h"
#include "quat.h"
#include "bams.h"
#include <stdlib.h>

// =========== Static Variables ===============================================
static unsigned char is_ready = 0, is_running;
static Quaternion prev_ref;
static float max_slew_rate, period;
static bams32_t max_angular_displacement;

// =========== Public Methods ===============================================
void slewSetup(float ts) {
    
    prev_ref.w = 1.0;
    prev_ref.x = 0.0;
    prev_ref.y = 0.0;
    prev_ref.z = 0.0;
    
    period = ts;
    
    is_running = 0; // TODO: Should be changed to off by default
    is_ready = 1;
    
}

void slewEnable(void) {
    is_running = 1;
}

void slewDisable(void) {
    is_running = 0;
}

// qtarget = qcurr*qdisp
// qcurr'*qtarget = qdisp
void slewProcess(Quaternion *input, Quaternion *output) {

    Quaternion conjugate, displacement;    
    bams32_t input_angle, limited_angle;
    float sin_input, sin_limited, scale;
    
    if(!is_ready || !is_running || max_angular_displacement == 0) {
        quatCopy(output, input); 
        return;
    }    
    
    // Calculate displacement
    quatConj(&prev_ref, &conjugate);
    quatMult(&conjugate, input, &displacement);

    // Calculate displacement magnitude
    input_angle = bams16ToBams32(bams16Acos(displacement.w)*2);
    if(input_angle == 0) { // Check for no displacement case
        quatCopy(output, input);
        return;
    }    
    sin_input = bams32Sin(input_angle/2);
    
    // Apply displacement limits
    if(input_angle > max_angular_displacement) {
        limited_angle = max_angular_displacement;
    } else if(input_angle < -max_angular_displacement) {
        limited_angle = -max_angular_displacement;
    } else {
        limited_angle = input_angle;
    }
    
    sin_limited = bams32SinFine(limited_angle/2);
    scale = sin_limited/sin_input;
    
    displacement.w = bams32CosFine(limited_angle/2);
    displacement.x = displacement.x*scale;
    displacement.y = displacement.y*scale;
    displacement.z = displacement.z*scale;
    
    // Apply limited displacement
    quatMult(&prev_ref, &displacement, output);
    quatNormalize(output);
    quatCopy(&prev_ref, output);

}

void slewSetLimit(float rate) {

    max_slew_rate = rate;
    max_angular_displacement = floatToBams32Rad(rate*period);
    
}

/**
 * Copyright (c) 2009 - 2012, Regents of the University of California
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
 * I-Bird Attitude Regulation Module
 *
 * by Stanley S. Baek
 *
 * v.beta
 *
 * Revisions:
 *  Stanley S. Baek      2009-10-30     Initial release
 *  Humphrey Hu		 2011-7-20      Changed to fixed point
 *  Humphrey Hu          2012-2-20      Returned to floating point, restructured
 *                                      and cleaned up states
 */

#include "controller.h"
#include "dsp.h"
#include "attitude.h"
#include "regulator.h"
#include "utils.h"
#include "motor_ctrl.h"
#include "led.h"
#include "sys_clock.h"
#include "gyro.h"
#include "xl.h"
#include "telemetry.h"
#include <stdlib.h>
#include "bams.h"
#include "cv.h"

// ==== Constants ==============================================================

// ==== Static Variables =======================================================
typedef struct {
    float thrust;
    float steer;
} ControlOutput;

// Lower level components
CtrlPidParam yawPid, pitchPid, rollPid;
DigitalFilter yawRateFilter, pitchRateFilter, rollRateFilter;

// State info
static unsigned char is_ready = 0;
static RegulatorState reg_state = REG_OFF;
static ControlOutput rc_outputs;
/*-----------------------------------------------------------------------------
 *          Declaration of static functions
-----------------------------------------------------------------------------*/

static float runYawControl(float yaw);
static float runPitchControl(float pitch);

/*-----------------------------------------------------------------------------
 *          Public functions
-----------------------------------------------------------------------------*/

void rgltrSetup(float ts) {

    is_ready = 0;
    reg_state = REG_OFF;
        
    mcSetup();  // Set up motor driver
    
    yawPid = ctrlCreatePidParams(ts);
    if(yawPid == NULL) { return; }
    pitchPid = ctrlCreatePidParams(ts);
    if(pitchPid == NULL) { return; }
    rollPid = ctrlCreatePidParams(ts);
    if(rollPid == NULL) { return; }

    yawRateFilter = NULL;
    pitchRateFilter = NULL;
    rollRateFilter = NULL;

    is_ready = 1;
}

void rgltrSetState(unsigned char flag) {

    if(flag == REG_OFF) {
        reg_state = flag;
        ctrlStop(yawPid);
        ctrlStop(pitchPid);
    } else if(flag == REG_TRACK) {
        reg_state = flag;
        ctrlStart(yawPid);
        ctrlStart(pitchPid);
    } else if(flag == REG_REMOTE_CONTROL) {
        reg_state = flag;
        ctrlStop(yawPid);
        ctrlStop(pitchPid);
    }
        
}

void rgltrSetYawRateFilter(RateFilterParams params) {

    yawRateFilter = dspCreateFilter(params->order, params->type,
                                    params->xcoeffs, params->ycoeffs);
    
} 


void rgltrSetPitchRateFilter(RateFilterParams params) {

    pitchRateFilter = dspCreateFilter(params->order, params->type,
                                    params->xcoeffs, params->ycoeffs);

} 

void rgltrSetRollRateFilter(RateFilterParams params) {

    rollRateFilter = dspCreateFilter(params->order, params->type,
                                    params->xcoeffs, params->ycoeffs);

}

void rgltrSetYawPid(PidParams params) {
    
    ctrlSetPidParams(yawPid, params->ref, params->kp, params->ki, params->kd);
    ctrlSetPidOffset(yawPid, params->offset);
    ctrlSetRefWeigts(yawPid, params->beta, params->gamma);
    ctrlSetSaturation(yawPid, 100, -100);

}

void rgltrSetPitchPid(PidParams params) {
    
    ctrlSetPidParams(pitchPid, params->ref, params->kp, params->ki, params->kd);
    ctrlSetPidOffset(pitchPid, params->offset);
    ctrlSetRefWeigts(pitchPid, params->beta, params->gamma);
    ctrlSetSaturation(pitchPid, 100, 0);

}

void rgltrSetRollPid(PidParams params) {

    ctrlSetPidParams(rollPid, params->ref, params->kp, params->ki, params->kd);
    ctrlSetPidOffset(rollPid, params->offset);
    ctrlSetRefWeigts(rollPid, params->beta, params->gamma);
    ctrlSetSaturation(rollPid, 100, -100);

}

void rgltrSetYawRef(float ref) {
    ctrlSetRef(yawPid, ref);
}

void rgltrSetPitchRef(float ref) {
    ctrlSetRef(pitchPid, ref);	
}

void rgltrSetRollRef(float ref) {
    ctrlSetRef(rollPid, ref);
}

void rgltrSetRemoteControlValues(float thrust, float steer) {

    rc_outputs.thrust = thrust;
    rc_outputs.steer = steer;
    
}

void rgltrRunController(void) {

    unsigned long time;
    float steer, thrust;
    PoseEstimateStruct pose;    

    if(!is_ready) { return; }   // Don't run if not ready
    if(reg_state == REG_OFF) { return; } // Don't run if not running

    time = sclockGetGlobalTicks(); // Record system time

    steer = 0.0;
    thrust = 0.0;
    pose.yaw = 0.0; // Initialize to make optimizer happy
    pose.pitch = 0.0;

    if(reg_state == REG_REMOTE_CONTROL) {

        steer = rc_outputs.steer;
        thrust = rc_outputs.thrust;

    } else if(reg_state == REG_TRACK){

        steer = runYawControl(pose.yaw);
        thrust = runPitchControl(pose.pitch);
        // ? = runRollControl(roll); // No roll actuator yet

    }

    // TODO: Roll control mixing
    mcSteer(steer);
    mcThrust(thrust);
    
}


// ==== Private Methods ========================================================

static float runYawControl(float yaw) {

    float u;

    if (ctrlIsRunning(yawPid) == 0) {
        return 0;	    
    } else {
        // take care of motor backlash by scaling u.
        // the dead zone of tail prop is +-15% duty cycle.
        u = ctrlRunPid(yawPid, yaw, yawRateFilter);        
        // TODO: Generalize!
        if (u > 5) {
            return u*0.85 + 15;
        } else if (u < -5) {		
            return u*0.85 - 15;
        } else {
            return u;
        }
    }    

}


static float runPitchControl(float pitch) {

    if (!ctrlIsRunning(pitchPid)) {
        return 0.0;
    } else {
        return ctrlRunPid(pitchPid, pitch, pitchRateFilter);
    }
}

//static float runRollControl(float roll) {
//
//    if(!ctrlIsRunning(rollPid)) {
//        return 0.0;
//    } else {
//        return ctrlRunPid(rollPid, roll, rollRateFilter);
//    }
//
//}

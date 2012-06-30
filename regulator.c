/*
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
 *  Stanley S. Baek     2009-10-30      Initial release
 *  Humphrey Hu		    2011-07-20       Changed to fixed point
 *  Humphrey Hu         2012-02-20       Returned to floating point, restructured
 *  Humphrey Hu         2012-06-30      Switched to using quaternions
 *
 * Notes:
 *  I-Bird body axes are: (check these)
 *      x - Forward along anteroposterior axis
 *      y - Left along left-right axis
 *      z - Up along dorsoventral axis
 *  Rotations in body axes are:
 *      yaw - Positive z direction
 *      pitch - Positive y direction
 *      roll - Positive x direction
 */

// Software modules
#include "regulator.h"
#include "controller.h"
#include "dfilter.h"
#include "attitude.h"
#include "cv.h"

// Hardware/actuator interface
#include "motor_ctrl.h"
#include "sync_servo.h"
#include "led.h"

// Other
#include "quat.h"
#include "sys_clock.h"
#include "bams.h"
#include "utils.h"
#include "pbuff.h"
#include <stdlib.h>
#include <string.h>

typedef struct {
    float thrust;
    float steer;
    float elevator;
} RegulatorOutput;

#define REG_BUFF_SIZE       (25)

// =========== Static Variables ================================================

// Lower level components
CtrlPidParamStruct yawPid, pitchPid, rollPid;
DigitalFilter yawRateFilter, pitchRateFilter, rollRateFilter;

// State info
static unsigned char is_ready = 0;
static RegulatorMode reg_mode;
static RegulatorOutput rc_outputs;
static Quaternion reference;

// Telemetry buffering
static RegulatorStateStruct reg_state[REG_BUFF_SIZE];
static PoolBuffStruct reg_state_buff;

// =========== Function Stubs =================================================
static float runYawControl(float yaw);
static float runPitchControl(float pitch);
static float runRollControl(float roll);

// =========== Public Functions ===============================================

void rgltrSetup(float ts) {

    unsigned int i;
    RegulatorState states[REG_BUFF_SIZE];

    is_ready = 0;
    reg_mode = REG_OFF;
        
    mcSetup();  // Set up motor driver

    ctrlInitPidParams(&yawPid, ts);
    ctrlInitPidParams(&pitchPid, ts);
    ctrlInitPidParams(&rollPid, ts);

    yawRateFilter = NULL;
    pitchRateFilter = NULL;
    rollRateFilter = NULL;

    for(i = 0; i < REG_BUFF_SIZE; i++) {
        states[i] = &reg_state[i];
    }
    pbuffInit(&reg_state_buff, REG_BUFF_SIZE, states);
    
    reference.w = 1.0;
    reference.x = 0.0;
    reference.y = 0.0;
    reference.z = 0.0;
    
    is_ready = 1;
}

void rgltrSetMode(unsigned char flag) {

    if(flag == REG_OFF) {
        reg_mode = flag;
        ctrlStop(&yawPid);
        ctrlStop(&pitchPid);
        ctrlStop(&rollPid);
    } else if(flag == REG_TRACK) {
        reg_mode = flag;
        ctrlStart(&yawPid);
        ctrlStart(&pitchPid);
        ctrlStart(&rollPid);
    } else if(flag == REG_REMOTE_CONTROL) {
        reg_mode = flag;
        ctrlStop(&yawPid);
        ctrlStop(&pitchPid);
        ctrlStop(&rollPid);
    }
        
}

void rgltrSetYawRateFilter(RateFilterParams params) {

    yawRateFilter = dfilterCreate(params->order, params->type,
                                    params->xcoeffs, params->ycoeffs);    
} 


void rgltrSetPitchRateFilter(RateFilterParams params) {

    pitchRateFilter = dfilterCreate(params->order, params->type,
                                    params->xcoeffs, params->ycoeffs);

} 

void rgltrSetRollRateFilter(RateFilterParams params) {

    rollRateFilter = dfilterCreate(params->order, params->type,
                                    params->xcoeffs, params->ycoeffs);

}

void rgltrSetYawPid(PidParams params) {
    
    ctrlSetPidParams(&yawPid, params->ref, params->kp, params->ki, params->kd);
    ctrlSetPidOffset(&yawPid, params->offset);
    ctrlSetRefWeigts(&yawPid, params->beta, params->gamma);
    ctrlSetSaturation(&yawPid, 100, -100);

}

void rgltrSetPitchPid(PidParams params) {
    
    ctrlSetPidParams(&pitchPid, params->ref, params->kp, params->ki, params->kd);
    ctrlSetPidOffset(&pitchPid, params->offset);
    ctrlSetRefWeigts(&pitchPid, params->beta, params->gamma);
    ctrlSetSaturation(&pitchPid, 100, 0);

}

void rgltrSetRollPid(PidParams params) {

    ctrlSetPidParams(&rollPid, params->ref, params->kp, params->ki, params->kd);
    ctrlSetPidOffset(&rollPid, params->offset);
    ctrlSetRefWeigts(&rollPid, params->beta, params->gamma);
    ctrlSetSaturation(&rollPid, 100, -100);

}

void rgltrSetYawRef(float ref) {
 
    ctrlSetRef(&yawPid, ref);

}

void rgltrSetPitchRef(float ref) {

    ctrlSetRef(&pitchPid, ref);

}

void rgltrSetRollRef(float ref) {

    ctrlSetRef(&rollPid, ref);

}

void rgltrSetRemoteControlValues(float thrust, float steer, float elevator) {

    rc_outputs.thrust = thrust;
    rc_outputs.steer = steer;
    rc_outputs.elevator = elevator;

}

void rgltrGetState(RegulatorState dst) {

    RegulatorState src;

    src = pbuffGetOldestActive(&reg_state_buff);
    if(src == NULL) { 
        memset(dst, 0, sizeof(RegulatorStateStruct));
        return; 
    }
    
    memcpy(dst, src, sizeof(RegulatorStateStruct));

    pbuffReturn(&reg_state_buff, src);
    
}

void rgltrRunController(void) {
    
    float steer, thrust, elevator;
    float yaw_err, pitch_err, roll_err, rot_mag;
    Quaternion pose, error, conj;
    RegulatorState state;
    
    if(!is_ready) { return; }

    attgetQuat(&pose);      // Retrieve pose estimate
    
    // qref = qerr*qpose
    // qref*qpose' = qerr    
    quatConj(&pose, &conj);
    quatMult(&reference, &conj, &error);    
    
    rot_mag = bams16ToFloatRad(bams16Acos(error.w)*2);  // w = cos(rot_mag/2)
    yaw_err = error.z*rot_mag;
    pitch_err = error.y*rot_mag;
    roll_err = error.x*rot_mag;
    
    if(reg_mode == REG_OFF) { 
        
        steer = 0.0;
        thrust = 0.0;
        elevator = 0.0;
        
    } else if(reg_mode == REG_REMOTE_CONTROL) {

        steer = rc_outputs.steer;
        thrust = rc_outputs.thrust;
        elevator = rc_outputs.elevator;

    } else if(reg_mode == REG_TRACK){

        steer = runYawControl(yaw_err);
        thrust = runPitchControl(pitch_err);
        elevator = rc_outputs.elevator;
        // ? = runRollControl(roll); // No roll actuator yet

    }

    state = pbuffForceGetIdleOldest(&reg_state_buff);
    if(state != NULL) {
        state->time = sclockGetLocalTicks();
        state->ref[0] = ctrlGetRef(&yawPid);
        state->ref[1] = ctrlGetRef(&pitchPid);
        state->ref[2] = ctrlGetRef(&rollPid);
        state->x[0] = yaw_err; //pose.yaw;
        state->x[1] = pitch_err; //pose.pitch;
        state->x[2] = roll_err; //pose.roll;
        state->u[0] = thrust;
        state->u[1] = steer;
        state->u[2] = elevator;
        pbuffAddActive(&reg_state_buff, (void*) state);
    }
    
    
    mcSteer(steer);
    mcThrust(thrust);
    servoSet(elevator);
    
}


// =========== Private Functions ===============================================

static float runYawControl(float yaw) {

    float u;

    if (!ctrlIsRunning(&yawPid)) {
        u = 0.0;
    } else {        
        u = ctrlRunPid(&yawPid, yaw, yawRateFilter);                
    }    

    return u;

}


static float runPitchControl(float pitch) {

    float u;

    if (!ctrlIsRunning(&pitchPid)) {
        u = 0.0;
    } else {
        u = ctrlRunPid(&pitchPid, pitch, pitchRateFilter);
    }

    return u;
    
}

//static float runRollControl(float roll) {
//
//    if(!ctrlIsRunning(&rollPid)) {
//        return 0.0;
//    } else {
//        return ctrlRunPid(&rollPid, roll, rollRateFilter);
//    }
//
//}

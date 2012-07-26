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
 * v.0.4
 *
 * Revisions:
 *  Stanley S. Baek     2009-10-30      Initial release
 *  Humphrey Hu		    2011-07-20      Changed to fixed point
 *  Humphrey Hu         2012-02-20      Returned to floating point, restructured
 *  Humphrey Hu         2012-06-30      Switched to using quaternion representation
 *
 * Notes:
 *  I-Bird body axes are:
 *      x - Forward along anteroposterior axis
 *      y - Left along left-right axis
 *      z - Up along dorsoventral axis
 *  Rotations in body axes are:
 *      yaw - Positive z direction
 *      pitch - Positive y direction
 *      roll - Positive x direction
 *  PID loops should have references set to 0, since they take the externally
 *      calculated error as an input.
 */

// Software modules
#include "regulator.h"
#include "controller.h"
#include "dfilter.h"
#include "attitude.h"
#include "rate.h"
#include "slew.h"

// Hardware/actuator interface
#include "motor_ctrl.h"
#include "sync_servo.h"

// Other
#include "quat.h"
#include "sys_clock.h"
#include "bams.h"
#include "utils.h"
#include "ppbuff.h"
#include <stdlib.h>
#include <string.h>

typedef struct {
    float thrust;
    float steer;
    float elevator;
} RegulatorOutput;

typedef struct {
    float yaw_err;
    float pitch_err;
    float roll_err;
} RegulatorError;

#define YAW_SAT_MAX         (1.0)
#define YAW_SAT_MIN         (-1.0)
#define PITCH_SAT_MAX       (1.0)
#define PITCH_SAT_MIN       (-1.0)
#define ROLL_SAT_MAX        (1.0)
#define ROLL_SAT_MIN        (0.0) // Doubling as thrust right now

#define DEFAULT_SLEW_LIMIT  (1.0)

// =========== Static Variables ================================================
// Control loop objects
CtrlPidParamStruct yawPid, pitchPid, thrustPid;
DigitalFilterStruct yawRateFilter, pitchRateFilter, rollRateFilter;

// State info
static unsigned char is_ready = 0, is_logging = 0; 
static unsigned char yaw_filter_ready = 0, pitch_filter_ready = 0, roll_filter_ready = 0;
static RegulatorMode reg_mode;
static RegulatorOutput rc_outputs;
static Quaternion reference, limited_reference, pose;

// Telemetry buffering
static RegulatorStateStruct reg_states[2];
static PingPongBuffer reg_state_buff;

// =========== Function Stubs =================================================
static float runYawControl(float yaw);
static float runPitchControl(float pitch);
static float runRollControl(float roll);

static void calculateError(RegulatorError *error);
static void filterError(RegulatorError *error);
static void calculateOutputs(RegulatorError *error, RegulatorOutput *output);
static void applyOutputs(RegulatorOutput *output);
static void logTrace(RegulatorError *error, RegulatorOutput *output);
// =========== Public Functions ===============================================

void rgltrSetup(float ts) {
        
    // Set up drivers
    servoSetup();
    mcSetup();
    
    // Set up dependent modules
    attSetup(ts);
    
    
    rateSetup(ts);
    
    
    slewSetup(ts);    
    slewSetLimit(DEFAULT_SLEW_LIMIT);
    slewEnable();
    
    reg_mode = REG_OFF;  

    ctrlInitPidParams(&yawPid, ts);
    ctrlInitPidParams(&pitchPid, ts);
    ctrlInitPidParams(&thrustPid, ts);    

    ppbuffInit(&reg_state_buff);
    ppbuffWriteActive(&reg_state_buff, &reg_states[0]);
    ppbuffWriteInactive(&reg_state_buff, &reg_states[1]);
        
    reference.w = 1.0;
    reference.x = 0.0;
    reference.y = 0.0;
    reference.z = 0.0;   
    
    limited_reference.w = 1.0;
    limited_reference.x = 0.0;
    limited_reference.y = 0.0;
    limited_reference.z = 0.0;   
    
    is_logging = 0;
    is_ready = 1;    
    
}

void rgltrSetMode(unsigned char flag) {

    if(flag == REG_OFF) {
        rgltrSetOff();
    } else if(flag == REG_TRACK) {
        rgltrSetTrack();
    } else if(flag == REG_REMOTE_CONTROL) {
        rgltrSetRemote();
    }
        
}

void rgltrSetOff(void) {
    reg_mode = REG_OFF;
    ctrlStop(&yawPid);
    ctrlStop(&pitchPid);
    ctrlStop(&thrustPid);
    servoStop();
}

void rgltrSetTrack(void) {
    reg_mode = REG_TRACK;
    ctrlStart(&yawPid);
    ctrlStart(&pitchPid);
    ctrlStart(&thrustPid);
    servoStart();
}

void rgltrSetRemote(void) {
    reg_mode = REG_REMOTE_CONTROL;
    ctrlStop(&yawPid);
    ctrlStop(&pitchPid);
    ctrlStop(&thrustPid);
    servoStart();
}    

void rgltrSetYawRateFilter(RateFilterParams params) {

    dfilterInit(&yawRateFilter, params->order, params->type, 
                params->xcoeffs, params->ycoeffs);
    yaw_filter_ready = 1;
    
} 


void rgltrSetPitchRateFilter(RateFilterParams params) {

    dfilterInit(&pitchRateFilter, params->order, params->type,
                params->xcoeffs, params->ycoeffs);
    pitch_filter_ready = 1;
                
} 

void rgltrSetRollRateFilter(RateFilterParams params) {

    dfilterInit(&rollRateFilter, params->order, params->type,
                params->xcoeffs, params->ycoeffs);
    roll_filter_ready = 1;
    
}

void rgltrSetOffsets(float *offsets) {

    ctrlSetPidOffset(&yawPid, offsets[0]);
    ctrlSetPidOffset(&pitchPid, offsets[1]);
    ctrlSetPidOffset(&thrustPid, offsets[2]);

}

void rgltrSetYawPid(PidParams params) {
    
    ctrlSetPidParams(&yawPid, params->ref, params->kp, params->ki, params->kd);
    ctrlSetPidOffset(&yawPid, params->offset);
    ctrlSetRefWeigts(&yawPid, params->beta, params->gamma);
    ctrlSetSaturation(&yawPid, YAW_SAT_MAX, YAW_SAT_MIN);

}

void rgltrSetPitchPid(PidParams params) {
    
    ctrlSetPidParams(&pitchPid, params->ref, params->kp, params->ki, params->kd);
    ctrlSetPidOffset(&pitchPid, params->offset);
    ctrlSetRefWeigts(&pitchPid, params->beta, params->gamma);
    ctrlSetSaturation(&pitchPid, PITCH_SAT_MAX, PITCH_SAT_MIN);

}

void rgltrSetRollPid(PidParams params) {

    ctrlSetPidParams(&thrustPid, params->ref, params->kp, params->ki, params->kd);
    ctrlSetPidOffset(&thrustPid, params->offset);
    ctrlSetRefWeigts(&thrustPid, params->beta, params->gamma);
    ctrlSetSaturation(&thrustPid, ROLL_SAT_MAX, ROLL_SAT_MIN);

}

void rgltrSetYawRef(float ref) {
    ctrlSetRef(&yawPid, ref);
}

void rgltrSetPitchRef(float ref) {
    ctrlSetRef(&pitchPid, ref);
}

void rgltrSetRollRef(float ref) {
    ctrlSetRef(&thrustPid, ref);
}

void rgltrGetQuatRef(Quaternion *ref) {
    if(ref == NULL) { return; }
    quatCopy(ref, &reference);
}

void rgltrSetQuatRef(Quaternion *ref) {
    if(ref == NULL) { return; }
    quatCopy(&reference, ref);
}

void rgltrSetRemoteControlValues(float thrust, float steer, float elevator) {
    rc_outputs.thrust = thrust;
    rc_outputs.steer = steer;
    rc_outputs.elevator = elevator;
}

void rgltrStartLogging(void) {
    is_logging = 1;
}

void rgltrStopLogging(void) {
    is_logging = 0;
}

void rgltrGetState(RegulatorState dst) {

    RegulatorState src;

    src = ppbuffReadActive(&reg_state_buff);
    if(src == NULL) { // Return 0's if no unread data
        memset(dst, 0, sizeof(RegulatorStateStruct));
        return; 
    }
    
    memcpy(dst, src, sizeof(RegulatorStateStruct));    
    
}

void rgltrRunController(void) {
    
    RegulatorError error;        
    RegulatorOutput output;    

    if(!is_ready) { return; }    

    attEstimatePose();  // Update attitude estimate
    rateProcess();      // Update limited_reference
    slewProcess(&reference, &limited_reference); // Apply slew rate limiting

    attGetQuat(&pose);
    calculateError(&error);    
    calculateOutputs(&error, &output);
    applyOutputs(&output);        
    
    if(is_logging) {
        logTrace(&error, &output);
    }

}


// =========== Private Functions ===============================================

static float runYawControl(float yaw) {

    if(yaw_filter_ready) {
        return ctrlRunPid(&yawPid, yaw, &yawRateFilter);
    } else {
        return ctrlRunPid(&yawPid, yaw, NULL);
    }

}


static float runPitchControl(float pitch) {   

    if(pitch_filter_ready) {
        return ctrlRunPid(&pitchPid, pitch, &pitchRateFilter);
    } else {
        return ctrlRunPid(&pitchPid, pitch, NULL);
    }        

}

static float runRollControl(float roll) {

    if(roll_filter_ready) {
        return ctrlRunPid(&thrustPid, roll, &rollRateFilter);
    } else {
        return ctrlRunPid(&thrustPid, roll, NULL);
    }

}

static void calculateError(RegulatorError *error) {

    Quaternion conj_quat, err_quat;
    bams16_t a_2;
    float scale;
    
    // qref = qpose*qerr
    // qpose'*qref = qerr
    quatConj(&pose, &conj_quat);
    //quatMult(&limited_reference, &conj_quat, &err_quat);
    quatMult(&conj_quat, &limited_reference, &err_quat);

    // q = [cos(a/2), sin(a/2)*[x, y, z]]
    // d[x, y, z] = [q]*a/sin(a/2)        
    if(err_quat.w == 1.0) { // a = 0 case
        error->yaw_err = 0.0;
        error->pitch_err = 0.0;
        error->roll_err = 0.0;
    } else {
        a_2 = bams16Acos(err_quat.w); // w = cos(a/2)             
        scale = bams16ToFloatRad(a_2*2)/bams16Sin(a_2); // a/sin(a/2)
        error->yaw_err = err_quat.z*scale;
        error->pitch_err = err_quat.y*scale;
        error->roll_err = err_quat.x*scale;
    }
    
}

static void filterError(RegulatorError *error) {

    if(yaw_filter_ready) {
        error->yaw_err = dfilterApply(&yawRateFilter, error->yaw_err);
    }
    if(pitch_filter_ready) {
        error->pitch_err = dfilterApply(&pitchRateFilter, error->pitch_err);
    }
    if(roll_filter_ready) {
        error->roll_err = dfilterApply(&rollRateFilter, error->roll_err);
    }
    
}

static void calculateOutputs(RegulatorError *error, RegulatorOutput *output) {

    if(reg_mode == REG_REMOTE_CONTROL) {

        output->steer = rc_outputs.steer;
        output->thrust = rc_outputs.thrust;
        output->elevator = rc_outputs.elevator;

    } else if(reg_mode == REG_TRACK){

        output->steer = runYawControl(error->yaw_err);
        output->elevator = runPitchControl(error->pitch_err);        
        output->thrust = runRollControl(error->pitch_err);

    } else {

        output->steer = 0.0;
        output->thrust = 0.0;
        output->elevator = 0.0;

    }

}

static void applyOutputs(RegulatorOutput *output) {

    mcSteer(output->steer);
    mcThrust(output->thrust);
    servoSet(output->elevator);

}

static void logTrace(RegulatorError *error, RegulatorOutput *output) {

    RegulatorStateStruct *storage;
    
    storage = ppbuffReadInactive(&reg_state_buff);
    
    if(storage != NULL) {
        quatCopy(&storage->ref, &limited_reference);
        quatCopy(&storage->pose, &pose);        
        storage->error.w = 0.0;
        storage->error.x = error->roll_err;
        storage->error.y = error->pitch_err;
        storage->error.z = error->yaw_err;
        storage->u[0] = output->thrust;
        storage->u[1] = output->steer;
        storage->u[2] = output->elevator;
        storage->time = sclockGetLocalTicks();        
    }
    ppbuffFlip(&reg_state_buff);

}

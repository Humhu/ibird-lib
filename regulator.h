/**
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
 * I-Bird Attitude Regulation Module
 *
 * by Stanley S. Baek
 *
 * v.beta
 *
 */

#ifndef __REGULATOR_H
#define __REGULATOR_H

typedef enum {
    REG_OFF = 0,
    REG_TRACK,
    REG_REMOTE_CONTROL,
} RegulatorState;

typedef struct {
    unsigned char order;
    unsigned char type;
    float* xcoeffs;
    float* ycoeffs;
} RateFilterParamsStruct;

typedef RateFilterParamsStruct *RateFilterParams;

typedef struct {
    float ref;
    float offset;
    float kp;
    float ki;
    float kd;
    float beta;
    float gamma;
} PidParamsStruct;

typedef PidParamsStruct *PidParams;

/**
 * Set up regulator module
 *
 * @param ts - Execution period
 */
void rgltrSetup(float ts);

/**
 * Set regulator state
 *
 * @param flag - State to transition to
 */
void rgltrSetState(unsigned char flag);

/**
 * Set yaw filter parameters
 *
 * @param params - Yaw filter parameter struct
 */
void rgltrSetYawRateFilter(RateFilterParams params);
void rgltrSetPitchRateFilter(RateFilterParams params);
void rgltrSetRollRateFilter(RateFilterParams params);

/**
 * Set yaw PID parameters
 *
 * @param params - Yaw PID parameter struct
 */
void rgltrSetYawPid(PidParams params);
void rgltrSetPitchPid(PidParams params);
void rgltrSetRollPid(PidParams params);

/**
 * Set yaw reference value
 *
 * @param ref - Yaw reference in radians
 */
void rgltrSetYawRef(float ref);
void rgltrSetPitchRef(float ref);
void rgltrSetRollRef(float ref);

/**
 * Set remote control output values
 *
 * @param thrust - Thrust duty cycle from 0.0 to 100.0
 * @param steer - Steering duty cycle from -100.0 to 100.0
 */
void rgltrSetRemoteControlValues(float thrust, float steer);

/**
 * Execute control loop iteration. This method should be called regularly every
 * ts seconds as defined in rgltrSetup
 * @see rgltrSetup
 */
void rgltrRunController(void);

#endif  // __REGULATOR_H



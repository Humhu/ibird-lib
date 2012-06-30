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

#ifndef __MOTOR_CTRL_H
#define __MOTOR_CTRL_H

#define MC_NUM_CHANNELS     (4)
#define MC_CHANNEL_PWM1     (1)
#define MC_CHANNEL_PWM2     (2)
#define MC_CHANNEL_PWM3     (3)
#define MC_CHANNEL_PWM4     (4)

typedef enum {
    MC_STEER_DISC = 0,
    MC_STEER_CONT,
} McSteerMode;

#define MC_STEER_MODE_DISC  (0)   
#define MC_STEER_MODE_CONT  (1)   

#define MC_STEER_LEFT       (-1.0)
#define MC_STEER_RIGHT      (1.0)
#define MC_STEER_STRAIGHT   (0.0)

// by default, all RE ports are output
void mcSetup(void);

// the resolution of the duty cycle is 1/(2*PTPER)
void mcSetDutyCycle(unsigned char channel, float duty_cycle);

/**
 * Stop all motor controller axes
 */
void mcStop(void);

/**
 * Set the duty cycle on PWM1
 * @param value - Duty cycle in [0.0, 1.0]
 */
void mcThrust(float value);

/**
 * Set the duty cycle and direction on PWM2 and PWM3's H-Bridge
 * @param value - Duty cycle in [-1.0, 1.0]
 */
void mcSteer(float value);

/**
 * Set the H-Bridge steering mode
 * @param mode - Steering mode flag
 */
void mcSetSteerMode(McSteerMode mode);

#endif  // __MOTOR_CTRL_H

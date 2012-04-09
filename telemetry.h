/*
 * Copyright (c) 2011, Regents of the University of California
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
 * Telemetry packet formats
 *
 * by Humphrey Hu
 *
 * v. beta
 *
 * Revisions:
 *  Humphrey Hu      2011-08-10    Initial implementation
 *                      
 * 
 */
 #ifndef _telemetry_h_
 #define _telemetry_h_

#include "bams.h"

// Standard Telemetry Packet (Type A)
// 30 octets total
#define TELEMETRY_A_SIZE	(30)

typedef struct {
	// Standard telemetry packet fields (30 octets)
	unsigned int time;
	unsigned int xlData[3]; // X Y and Z
	unsigned int gyroData[3]; // X Y and Z
	unsigned int backEMF[2]; // Channel 1 and 2
	float motorPWM[2]; // Channel 1 and 2
	unsigned int battVoltage[2]; // Loaded and unloaded
} TelemetryStructA;

typedef TelemetryStructA* TelemetryA;
 
// Lightweight Telemetry Format (Type B)
// 34 octets total
#define TELEMETRY_B_SIZE	(10)

typedef struct {
	unsigned long time; // System clock timestamp (4)	
	bams16_t pose[3]; // Euler angles in binary angles (6)	
} TelemetryStructB;

typedef TelemetryStructB* TelemetryB;

// Send a type A telemetry packet
void telemSendA(unsigned int addr);

// Send a type B telemetry packet
void telemSendB(unsigned int addr);

#endif


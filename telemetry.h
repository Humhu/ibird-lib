/*
 * Copyright (c) 2011-2012, Regents of the University of California
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
 */
 #ifndef __TELEMETRY_H
 #define __TELEMETRY_H

#include "bams.h"
#include "quat.h"
#include "regulator.h"

typedef struct {
    RegulatorStateStruct reg_state;
    unsigned char ED;
    unsigned char RSSI;
} TelemetryDatapoint;
 
// State Telemetry Packet (Type B)
#define TELEMETRY_B_SIZE	(66)
typedef struct {
    unsigned long time;     // (4) Local time    
    Quaternion ref;         // (16) Reference
    Quaternion pose;        // (16) Position
    Quaternion error;       // (16) Error
    float u[3];             // (12) Output
    unsigned char ED;
    unsigned char RSSI;
} TelemetryStructB;
typedef TelemetryStructB* TelemetryB;

#define TELEMETRY_ATT_SIZE  (16)
typedef struct {
    Quaternion att;
} TelemetryStructAttitude;
typedef TelemetryStructAttitude* TelemetryAttitude;

void telemSetup(void);
void telemSetSubsampleRate(unsigned int rate);
void telemStartLogging(void);
void telemStopLogging(void);

// Writes into the buffer
void telemLog(void);
// Process the buffer
void telemProcess(void);

// Send a type B telemetry packet
void telemSendB(unsigned int addr);

// Send an attitude report packet
void telemSendAttitude(unsigned int addr);

#endif


/**
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
 * Clock Synchronization Module
 *
 * by Humphrey Hu
 *
 * v.beta
 *
 * Revisions:
 *  Humphrey Hu		2012-02-28		Initial implementation 
 *                      
 * Notes:
 *  Still need to implement iteration counting and pos/neg offset comparison for
 *  in-sync determination
 */

// ==== REFERENCES ============================================================
#include "mac_packet.h"
#include "radio.h"
#include "net.h"
#include "cmd_const.h"
#include "sys_clock.h"
#include "clock_sync.h"
#include "led.h"
// ==== CONSTANTS =============================================================

#define DEFAULT_SYNC_TOL        (312) // 0.5 ms (625 ticks/ms)

#define SAMPLES_PER_ITERATION   (1)
#define MAX_ITERATIONS          (40)
#define MAX_PENDING_REQUESTS    (10)

#define DEFAULT_MASTER_ADDR     (0x1021)
#define DEFAULT_MASTER_PAN      (0x1001)

// ==== STATIC VARIABLES ======================================================

SyncStatusStruct status;

// =========== Function Stubs =================================================

static void clksyncSendRequest(SyncStatus sync);
static void clksyncProcessSamples(SyncStatus sync);

// =========== Public Functions ===============================================
void clksyncSetup(void) {

    status.tolerance = DEFAULT_SYNC_TOL;
    
    status.master_addr = DEFAULT_MASTER_ADDR; // Make more general
    status.master_pan = DEFAULT_MASTER_PAN;

    status.state = STATE_UNSYNCED;
    status.requests = 0;
    status.responses = 0;
    status.iterations = 0;
    status.accumulator = 0;

    if(netGetLocalAddress() == status.master_addr &&
        netGetLocalPanID() == status.master_pan) {
        status.state = STATE_MASTER;
    }

    
}

void clksyncSetTolerance(unsigned long tol) {

    status.tolerance = tol;

}

void clksyncSetMasterAddr(unsigned int addr, unsigned int pan) {

    status.master_addr = addr;
    status.master_pan = pan;
    
}

void clksyncSync(void) {

    if(clksyncIsDone()) { return; }
    
    clksyncSendRequest(&status);

}

unsigned char clksyncIsDone(void) {

    return status.state != STATE_UNSYNCED;

}
 
void clksyncHandleRequest(MacPacket packet) {

    Payload pld;
    MacPacket response;
    unsigned long* frame;
    unsigned long s0, m1, m2;
    
    pld = macGetPayload(packet);
    frame = (unsigned long*) payGetData(pld);
    
    s0 = frame[0]; // Read requester time of flight
    m1 = packet->timestamp + sclockGetOffsetTicks(); // Read local time of reception

    response = radioRequestPacket(12); // Sending 3 longs
    if(response == NULL) { return; }

    macSetDestAddr(response, macGetSrcAddr(packet));
    macSetDestPan(response, macGetSrcPan(packet));
    pld = macGetPayload(response); // Create response packet
    paySetType(pld, CMD_CLOCK_UPDATE_RESPONSE);
    paySetData(pld, 4, (unsigned char*) &s0);
    payAppendData(pld, 4, 4, (unsigned char*) & m1);
    
    // Empty TX queue to minimize time of flight error
    while(!radioTxQueueEmpty());
    
    m2 = sclockGetGlobalTicks(); // Get approximate time of flight
    payAppendData(pld, 8, 4, (unsigned char*) & m2);
    
    while(!radioEnqueueTxPacket(response));
    radioProcess(); // Fast send

}

void clksyncHandleResponse(MacPacket packet) {

    Payload pld;    
    unsigned long* frame;
    unsigned long s0, m1, m2, s3;
    long long residual_offset;

    pld = macGetPayload(packet);
    frame = (unsigned long *) payGetData(pld);
    
    s0 = frame[0];
    m1 = frame[1];
    m2 = frame[2];
    s3 = packet->timestamp + sclockGetOffsetTicks();
    
    residual_offset = (-s0 - s3 + m1 + m2)/2;
    status.accumulator += residual_offset;
        
    status.responses++;

    if(status.responses >= SAMPLES_PER_ITERATION) {
        clksyncProcessSamples(&status);
    }
}
 
// ==== PRIVATE FUNCTIONS ===================================================== 
static void clksyncSendRequest(SyncStatus sync) {

    MacPacket packet;
    Payload pld;
    unsigned long s0;
    
    packet = radioRequestPacket(4);
    if(packet == NULL) { return; }
    macSetDestAddr(packet, sync->master_addr);
    macSetDestPan(packet, sync->master_pan);
    pld = macGetPayload(packet);
    paySetType(pld, CMD_CLOCK_UPDATE_REQUEST);

    while(!radioTxQueueEmpty());
    
    s0 = sclockGetGlobalTicks();
    pld = macGetPayload(packet);
    paySetData(pld, 4, (unsigned char*) &s0);

    while(!radioEnqueueTxPacket(packet));
    radioProcess(); // Fast send

    sync->requests++;
    if(sync->requests - sync->responses > MAX_PENDING_REQUESTS) {
        sync->state = STATE_REQUEST_TIMEOUT;        
    }

}

void clksyncProcessSamples(SyncStatus sync) {

    unsigned long current_offset, error;
    long long average_offset;

    current_offset = sclockGetOffsetTicks();
    average_offset = sync->accumulator/sync->responses;
    error = (unsigned long) current_offset + average_offset;

    sclockSetOffsetTicks(error);

    sync->accumulator = 0;
    sync->requests = 0;
    sync->responses = 0;
    sync->iterations++;

    if((average_offset < sync->tolerance)){ //&&
            //(average_offset > -(sync->tolerance))) {
        sync->state = STATE_SYNCED;
    }

    if(sync->iterations > MAX_ITERATIONS) {
        sync->state = STATE_ITERS_EXCEEDED;        
    }

}
 
 
 
 
 
 
 
 
 

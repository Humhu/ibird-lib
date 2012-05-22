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
 *
 */
 
#ifndef __CLK_SYNC_H 
#define __CLK_SYNC_H
 
#include "mac_packet.h"

typedef enum {
    STATE_UNSYNCED = 0,
    STATE_MASTER,
    STATE_SYNCED,
    STATE_REQUEST_TIMEOUT,
    STATE_ITERS_EXCEEDED,
} SyncState;

typedef struct {
    SyncState state;
    unsigned int master_addr;
    unsigned int master_pan;
    unsigned long tolerance;
    unsigned int requests;
    unsigned int responses;
    unsigned int iterations;
    long long accumulator;
} SyncStatusStruct;

typedef SyncStatusStruct* SyncStatus;

void clksyncSetup(void);

void clksyncSetTolerance(unsigned long tol);
void clksyncSetMasterAddr(unsigned int addr, unsigned int pan);

// Sync if last measured error was greater than tolerance
void clksyncSync(void);
// See if synced or not
unsigned char clksyncIsDone(void);
 
void clksyncHandleRequest(MacPacket packet);
void clksyncHandleResponse(MacPacket packet);
 
 
 
 
 
 
 
 
 
 
#endif

 
 

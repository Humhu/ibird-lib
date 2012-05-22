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
 * Network Directory
 *
 * by Humphrey Hu
 *
 * v.beta
 *
 * Revisions:
 *  Humphrey Hu     2011-09-03      Initial implementation
 *  Humphrey Hu     2012-04-04      Updates (FIX)      
 * Notes:
 */

 #ifndef __DIRECTORY_H
 #define __DIRECTORY_H
 
 #include "telemetry.h"
 #include "larray.h"

 typedef enum {
    BASESTATION = 0,
    IBIRD,
    OCTOROACH,
} DirEndpointType;

typedef struct {
    unsigned char imu : 3;
    unsigned char cam : 3;
    unsigned int padding : 10;
} DirEndpointParam;

 typedef struct {    // (28)
 
    unsigned long long uuid;    // Endpoint identifier number (8)
    DirEndpointType type;       // Type of endpoint (2)
    DirEndpointParam params;    // Available resource parameters (2)    
 
    unsigned long frame_period; // Camera period (4)
    unsigned long frame_start;  // Last frame capture start time (4)
 
    unsigned long timestamp;    // Last time of contact in local time (4)
    unsigned int address;       // 16-bit radio address (2)
    unsigned int pan_id;        // Radio PAN ID (2)
    
 } DirEntryStruct;
 
 typedef DirEntryStruct* DirEntry;

 // Return 1 if match, 0 if not match
 typedef unsigned int (*DirEntryTest)(DirEntry entry, void *args);
 //typedef LinArrayItemTest DirEntryTest;

 void dirInit(unsigned int size);

 unsigned int dirQuery(DirEntryTest comp, void *args, DirEntry *entry);
 unsigned int dirQueryN(DirEntryTest comp, void *args, DirEntry *entries,
                        unsigned int N);

 DirEntry dirQueryAddress(unsigned int addr, unsigned int pan);
 DirEntry dirQueryID(unsigned long long id);
 unsigned int dirGetSize(void);
 unsigned int dirGetEntries(DirEntry *entries);

 DirEntry dirAddNew(void);

 #endif


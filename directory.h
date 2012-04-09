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
 #include "linear_array.h"

typedef enum {
    BASESTATION = 0,
    IBIRD,
    OCTOROACH,
} DirEndpointType;

typedef struct {
    unsigned char imu : 3;
    unsigned char cam : 3;
    unsigned int padding : 2;
} DirEndpointParam;

 typedef struct {    
    unsigned int uuid;          // Endpoint identifier number
    DirEndpointType type;       // Type of endpoint
    DirEndpointParam params;    // Available resource parameters
    unsigned long timestamp;    // Last time of contact (local time)
    unsigned int address;       // 16-bit radio address
    unsigned int pan_id;        // Radio PAN ID
 } DirEntryStruct;
 
 typedef DirEntryStruct* DirEntry;

 // Return 1 if match, 0 if not match
 typedef unsigned int (*DirEntryTest)(DirEntry entry, void *args);
 //typedef LinArrayItemTest DirEntryTest;

 void dirInit(unsigned int size);

 DirEntry dirQuery(DirEntryTest comp, void *args);
 DirEntry dirAddNew(void);

 #endif


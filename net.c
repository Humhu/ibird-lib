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
* Network addressing module
*
* by Humphrey Hu
*
* v.beta
*
* Revisions:
*   Humphrey Hu         2011-07-27      Initial implementation
*   Humphrey Hu         2011-09-03      Moved directory to separate module
*                      
* Notes:
*
* TODO:
*	Implement robust multi-hop networking
*/

// ==== REFERENCES ==========================================
#include "net.h"
#include "directory.h"
#include "radio.h"
#include "telemetry.h"

#include "cmd_const.h"
#include "utils.h"
#include "counter.h"
#include "sys_clock.h" 

#include <stdlib.h>

// ==== CONSTANTS =========================================== 
#define MAX_REQUEST_ATTEMPTS        (10)

#define DEFAULT_LOCAL_ADDR          (0x1021)
#define DEFAULT_LOCAL_PAN           (0x1001)
#define DEFAULT_LOCAL_CHANNEL       (0x15)

#define DEFAULT_BASE_ADDR           (0x1020)
#define DEFAULT_BASE_PAN            (0x1001)
#define DEFAULT_BASE_CHANNEL        (0x15)

// ==== STATIC VARIABLES ====================================
// Local assigned parameters
static unsigned int localAddress;
static unsigned int localPanID;
static unsigned char localChannel;
static unsigned long long local_UUID;

// Network assigned parameters
static unsigned int baseStationAddress;
static unsigned int baseStationPanID;
static unsigned int baseStationChannel;

static unsigned int request_attempts;
static unsigned char attempts_exceeded, address_received;

// ==== FUNCTION STUBS ======================================
// Send methods
void netSendRequest(void);
void netSendAccept(long);

// ==== FUNCTION BODIES =====================================
// Setup the network module
// Note that this method blocks until an address is assigned!
void netSetup(unsigned int dir_size) {

    localAddress = DEFAULT_LOCAL_ADDR;
    localPanID = DEFAULT_LOCAL_PAN;
    localChannel = DEFAULT_LOCAL_CHANNEL;

    baseStationAddress = DEFAULT_BASE_ADDR;
    baseStationPanID = DEFAULT_BASE_PAN;
    baseStationChannel = DEFAULT_BASE_CHANNEL;

    // Generate 64-bit UUID
    unsigned long a, b;
    a = (unsigned long) rand();
    b = ((unsigned long) rand()) << 16;
    local_UUID = a|b;
    
    attempts_exceeded = 0;
    address_received = 0;
    
    dirInit(dir_size);
    
}

// Retrieve local address
unsigned int netGetLocalAddress(void) {

    return localAddress;

}

// Retrieve local PAN ID
unsigned int netGetLocalPanID(void) {

    return localPanID;
    
}

unsigned int netGetBaseAddress(void) {

    return baseStationAddress;
    
}

unsigned int netGetBasePanID(void) {

    return baseStationPanID;
    
}

unsigned char netAddressReceived(void) {

    return address_received || attempts_exceeded;

}

// Request an address
void netRequestAddress(void) {
    
    if(netAddressReceived()) { return; }
    
    netSendRequest();
    
    request_attempts++;    
    if(request_attempts > MAX_REQUEST_ATTEMPTS) {    
        attempts_exceeded = 1;
    }
    
}

void netHandleOffer(MacPacket packet) {
    
    Payload pld;    
    unsigned long long uuidKey;
    unsigned long offerID, timestamp;
    unsigned char* data;    

    pld = macGetPayload(packet);
    uuidKey = *((unsigned long long*) payGetData(pld));

    // If offer is addressed to us, decode offer and send acceptance
    if(uuidKey == local_UUID) {
        data = payGetData(pld);
        offerID = *((unsigned long*) (data + 8));
        localAddress = *((unsigned int*) (data + 12));
        localPanID = *((unsigned int*) (data + 14));
        localChannel = (unsigned char)*((unsigned int*) (data + 16));

        baseStationAddress = *((unsigned int*) (data + 18));
        baseStationPanID = *((unsigned int*) (data + 20));
        baseStationChannel = (unsigned char)*((unsigned int*) (data + 22));

        timestamp = *((unsigned long *) data + 24);

        netSendAccept(offerID);
        address_received = 1;
        return;
    }        

}

void netHandleRequest(MacPacket packet) {

    return;

}

void netHandleAccept(MacPacket packet) {

    return;

}

// =========== Private Functions ==============================================

// Broadcast a request for an address
// Note that the radio needs to be set to the appropriate PAN
// TODO: Have network setup set radio to appropriate PAN!
void netSendRequest(void) {
    
    MacPacket request_packet;
    Payload pld;    

    // Create request packet
    request_packet = radioRequestPacket(8);
    if(request_packet == NULL) { return; }
    macSetDestAddr(request_packet, NETWORK_BROADCAST_ADDR);
    pld = macGetPayload(request_packet);
    paySetData(pld, 8, (unsigned char *) &local_UUID);
    paySetStatus(pld, 0);
    paySetType(pld, CMD_ADDRESS_REQUEST);

    while(!radioEnqueueTxPacket(request_packet));

}

// Broadcasts acceptance of an address
void netSendAccept(long offerID) {
    
    MacPacket accept_packet;
    Payload pld;
    long offer = offerID;
    
    // Broadcast acceptance to inform other coordinators
    accept_packet = radioRequestPacket(12);
    if(accept_packet == NULL) { return; }
    macSetDestAddr(accept_packet, NETWORK_BROADCAST_ADDR);
    
    pld = macGetPayload(accept_packet);
    paySetData(pld, 8, (unsigned char *) &local_UUID);
    payAppendData(pld, 8, 4, (unsigned char *) & offer);
    paySetStatus(pld, 0);
    paySetType(pld, CMD_ADDRESS_ACCEPT);

    while(!radioEnqueueTxPacket(accept_packet));

}

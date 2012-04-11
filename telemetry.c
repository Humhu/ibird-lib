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
 * Telemetry module
 *
 * by Humphrey Hu
 *
 * v. beta
 *
 * Revisions:
 *  Humphrey Hu      2011-10-26    Initial implementation
 *                      
 * 
 */

#include "sys_clock.h"
#include "telemetry.h"
#include "radio.h"
#include "net.h"
#include "mac_packet.h"
#include "payload.h"
#include "gyro.h"
#include "xl.h"
#include "regulator.h"
#include "attitude.h"
#include "behavior.h"
#include "counter.h"

#include "cmd_const.h"

// Function stubs
void telemPopulateA(TelemetryStructA*); // Not yet implemented
void telemPopulateB(TelemetryStructB*); 

// Function bodies
// TODO: Implement!
void telemSendA(unsigned int addr) {

}

// TODO: Move mac packet creation from Radio_DMA to Mac_Packet
void telemSendB(unsigned int addr) {

	MacPacket packet;
	Payload pld;
	TelemetryStructB telemetryB;
	
	// Populate the telemetry fields
	telemPopulateB(&telemetryB);
	
	// Create a radio packet
	packet = radioRequestPacket(TELEMETRY_B_SIZE);
	if(packet == NULL) { return; }
        macSetDestAddr(packet, addr);
        macSetDestPan(packet, netGetLocalPanID());

	// Write the telemetry struct into the packet payload
	pld = macGetPayload(packet);
	paySetType(pld, CMD_RESPONSE_TELEMETRY);
	paySetData(pld, TELEMETRY_B_SIZE, (unsigned char *) &telemetryB);
	 if(!radioEnqueueTxPacket(packet)) {
		 radioReturnPacket(packet);	// Delete packet if append fails
	 }
	
}

void telemPopulateA(TelemetryStructA *telemetry) {

}

void telemPopulateB(TelemetryStructB *telemetry) {	    

    telemetry->time = sclockGetGlobalTicks();
    telemetry->pose[0] = attGetYawBAMS();
    telemetry->pose[1] = attGetPitchBAMS();
    telemetry->pose[2] = attGetRollBAMS();
    
    return;
	
}

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
#include "regulator.h"
#include "attitude.h"
#include "cmd_const.h"
#include "dfmem.h"
#include "led.h"
#include "utils.h"
#include "pbuff.h"

#include <string.h>

#define DEFAULT_START_PAGE      (0x80)
#define TELEM_BUFF_SIZE         (5)
#define DEFAULT_SUBSAMPLE       (1)

typedef enum {
    TELEM_IDLE = 0,
    TELEM_LOGGING,
    TELEM_ERROR,
} TelemStatus;

// =========== Static Variables ================================================
static unsigned char is_ready = 0;
static TelemStatus status = TELEM_IDLE;
static unsigned int iter_num, subsample_period;

static PoolBuffStruct telem_buff;
static TelemetryDatapoint datapoints[TELEM_BUFF_SIZE];

static DfmemGeometryStruct mem_geo;
static unsigned int mem_page_pos, mem_byte_pos, mem_buff_index;

// =========== Function Stubs ==================================================
void telemPopulateB(TelemetryB); 
void telemPopulateAttitude(TelemetryAttitude);

// =========== Public Methods ==================================================
void telemSetup(void) {

    unsigned int i;
    TelemetryDatapoint *s[TELEM_BUFF_SIZE];

    dfmemGetGeometryParams(&mem_geo); // Read memory chip sizing
    mem_page_pos = DEFAULT_START_PAGE;    
    mem_byte_pos = 0;
    mem_buff_index = 0;    

    for(i = 0; i < TELEM_BUFF_SIZE; i++) {
        s[i] = &datapoints[i];
    }

    pbuffInit(&telem_buff, TELEM_BUFF_SIZE, (void**)s);
    if(telem_buff.valid == 0) { return; }

    iter_num = 0;
    subsample_period = DEFAULT_SUBSAMPLE;
    
    is_ready = 1;

}

void telemSetSubsampleRate(unsigned int rate) {
    subsample_period = rate;
}

void telemStartLogging(void) {

    if(!is_ready) { return; }

    mem_page_pos = DEFAULT_START_PAGE;
    mem_byte_pos = 0;
    mem_buff_index = 0;

    dfmemEraseChip();
//    unsigned int i;
//    i = mem_page_pos;
//    while(i < mem_geo.max_pages/4) {
//        dfmemEraseSector(i);
//        i += mem_geo.pages_per_sector;
//    }
    
    while(!dfmemIsReady());
    status = TELEM_LOGGING;
    LED_RED = 1;

}

void telemStopLogging(void) {
    
    // TODO: Check for error condition
    status = TELEM_IDLE;
    LED_RED = 0;
    
}

void telemLog(void) {

    TelemetryDatapoint *data;
    RadioStatus radio_stat;
    
    if(!is_ready) { return; }
    if(status != TELEM_LOGGING) { return; }
    
    iter_num++;    
    if(iter_num % subsample_period != 0) { return; }
    
    data = pbuffGetIdle(&telem_buff);
    if(data == NULL) { return; }
    
    rgltrGetState(&data->reg_state); // Fetch regulator data
    radioGetStatus(&radio_stat);
    data->ED = radio_stat.last_ed;
    data->RSSI = radio_stat.last_rssi;
    pbuffAddActive(&telem_buff, data); // Queue data

}


void telemProcess(void) {

    TelemetryDatapoint *data;

    if(!is_ready) { return; }
    if(mem_page_pos >= mem_geo.max_pages) { telemStopLogging(); }
    if(status != TELEM_LOGGING) { return; }
    
    data = pbuffGetOldestActive(&telem_buff);
    if(data == NULL) { return; }    
    
    dfmemWriteBuffer((unsigned char*)data, sizeof(TelemetryDatapoint), mem_byte_pos, mem_buff_index);
    mem_byte_pos += sizeof(TelemetryDatapoint);
    
    if(mem_byte_pos + sizeof(TelemetryDatapoint) > mem_geo.bytes_per_page) {
    
        dfmemWriteBuffer2MemoryNoErase(mem_page_pos, mem_buff_index);
        mem_buff_index ^= 0x01;
        mem_byte_pos = 0;
        mem_page_pos++;                
        
    }

    pbuffReturn(&telem_buff, data);

}

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

void telemSendAttitude(unsigned int addr) {

    MacPacket packet;
	Payload pld;
	TelemetryStructAttitude telemetryAtt;
	
	// Populate the telemetry fields
	telemPopulateAttitude(&telemetryAtt);
	
	// Create a radio packet
	packet = radioRequestPacket(TELEMETRY_ATT_SIZE);
	if(packet == NULL) { return; }
    macSetDestAddr(packet, addr);
    macSetDestPan(packet, netGetLocalPanID());

	// Write the telemetry struct into the packet payload
	pld = macGetPayload(packet);
	paySetType(pld, CMD_RESPONSE_ATTITUDE);
	paySetData(pld, TELEMETRY_ATT_SIZE, (unsigned char *) &telemetryAtt);
	if(!radioEnqueueTxPacket(packet)) {
        radioReturnPacket(packet);	// Delete packet if append fails
	}
	
}

void telemPopulateB(TelemetryB telemetry) {	    

    RegulatorStateStruct state;

    rgltrGetState(&state);
    memcpy(telemetry, &state, sizeof(RegulatorStateStruct));

}

void telemPopulateAttitude(TelemetryAttitude att) {

    Quaternion pose;
    
    attGetQuat(&pose);
    memcpy(att, &pose, sizeof(Quaternion));    

}
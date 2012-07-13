/**
 * Copyright (c) 2010-2012, Regents of the University of California
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
 * Robot command processing module
 *
 * by Stan Baek
 *
 * v.beta
 *
 * Revisions:
 *  Stan Baek		 2011-07-10	   Initial implementation
 *  Humphrey Hu		 2011-08-06    Added more commands and reply-to functionality
 *                      
 * Notes:
 *
 * To do:
 *	
 */

#include <stdio.h>
#include "cam.h"
#include "telemetry.h"
#include "cmd.h"
#include "cmd_const.h"
#include "radio.h"
#include "rate.h"
#include "lstrobe.h"
#include "net.h"
#include "dfmem.h"
#include "utils.h"
#include "ports.h"
#include "gyro.h"
#include "xl.h"
#include "clock_sync.h"
#include "sys_clock.h"
#include "led.h"
#include "motor_ctrl.h"
#include "payload.h"
#include "attitude.h"
#include "regulator.h"
#include "mac_packet.h"
#include "cv.h"
#include "directory.h"
#include "carray.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>

typedef union {
    float fval;
    unsigned long lval;
    short sval[2];
    unsigned char cval[4];
} uByte4;

typedef union {
    unsigned short sval;
    unsigned char cval[2];
} uByte2;

// use an array of function pointer to avoid a number of case statements
// MAX_CMD_FUNC_SIZE is defined in cmd_const.h
void (*cmd_func[MAX_CMD_FUNC_SIZE])(MacPacket);

// ==== Static Variables =======================================================
static CircArray input_queue;

// ==== Function Prototypes ====================================================
static void cmdAddressRequest(MacPacket packet);
static void cmdAddressOffer(MacPacket packet);
static void cmdAddressAccept(MacPacket packet);

static void cmdDirUpdateRequest(MacPacket packet);
static void cmdDirUpdateResponse(MacPacket packet);
static void cmdDirDumpRequest(MacPacket packet);
static void cmdDirDumpResponse(MacPacket packet);

static void cmdRequestClockUpdate(MacPacket packet);
static void cmdResponseClockUpdate(MacPacket packet);

static void cmdSetRegulatorMode(MacPacket packet);
static void cmdSetRegulatorRef(MacPacket packet);
static void cmdSetRegulatorPid(MacPacket packet);
static void cmdSetRegulatorRateFilter(MacPacket packet);
static void cmdSetRemoteControlValues(MacPacket packet);

static void cmdSetRateMode(MacPacket packet);
static void cmdSetRateSlew(MacPacket packet);

static void cmdRequestRawFrame(MacPacket packet);
static void cmdResponseRawFrame(MacPacket packet);
static void cmdSetBackgroundFrame(MacPacket packet);
static void cmdCamParamRequest(MacPacket packet);
static void cmdCamParamResponse(MacPacket packet);

static void cmdRequestTelemetry(MacPacket packet);
static void cmdResponseTelemetry(MacPacket packet);
static void cmdRecordTelemetry(MacPacket packet);

static void cmdSetLogging(MacPacket packet);
static void cmdGetMemContents(MacPacket packet);

static void cmdRunGyroCalib(MacPacket packet);
static void cmdGetGyroCalibParam(MacPacket packet);

static void cmdSetEstimateRunning(MacPacket packet);

static void cmdSetHP(MacPacket packet);

static void cmdZeroEstimate(MacPacket packet);
static void cmdRequestAttitude(MacPacket packet);
static void cmdResponseAttitude(MacPacket packet);

static void cmdEcho(MacPacket packet);
static void cmdNop(MacPacket packet);

// =============== Public Functions ============================================
unsigned int cmdSetup(unsigned int queue_size) {

    unsigned int i;

    input_queue = carrayCreate(queue_size);
    if(input_queue == NULL) {
        return 0;
    }

    // initialize the array of func pointers with Nop()
    for(i = 0; i < MAX_CMD_FUNC_SIZE; ++i) {
        cmd_func[i] = &cmdNop;
    }

    cmd_func[CMD_ADDRESS_REQUEST] = &cmdAddressRequest;
    cmd_func[CMD_ADDRESS_OFFER] = &cmdAddressOffer;
    cmd_func[CMD_ADDRESS_ACCEPT] = &cmdAddressAccept;

    cmd_func[CMD_DIR_UPDATE_REQUEST] = &cmdDirUpdateRequest;
    cmd_func[CMD_DIR_UPDATE_RESPONSE] = &cmdDirUpdateResponse;
    cmd_func[CMD_DIR_DUMP_REQUEST] = &cmdDirDumpRequest;
    cmd_func[CMD_DIR_DUMP_RESPONSE] = &cmdDirDumpResponse;

    cmd_func[CMD_CLOCK_UPDATE_REQUEST] = &cmdRequestClockUpdate;
    cmd_func[CMD_CLOCK_UPDATE_RESPONSE] = &cmdResponseClockUpdate;

    cmd_func[CMD_SET_ESTIMATE_RUNNING] = &cmdSetEstimateRunning;

    cmd_func[CMD_ECHO] = &cmdEcho;

    cmd_func[CMD_SET_REGULATOR_MODE] = &cmdSetRegulatorMode;
    cmd_func[CMD_SET_REGULATOR_REF] = &cmdSetRegulatorRef;
    cmd_func[CMD_SET_REGULATOR_PID] = &cmdSetRegulatorPid;
    cmd_func[CMD_SET_REGULATOR_RATE_FILTER] = &cmdSetRegulatorRateFilter;

    cmd_func[CMD_SET_RATE_MODE] = &cmdSetRateMode;
    cmd_func[CMD_SET_RATE_SLEW] = &cmdSetRateSlew;

    cmd_func[CMD_RAW_FRAME_REQUEST] = &cmdRequestRawFrame;
    cmd_func[CMD_RAW_FRAME_RESPONSE] = &cmdResponseRawFrame;
    cmd_func[CMD_SET_BACKGROUND_FRAME] = &cmdSetBackgroundFrame;
    cmd_func[CMD_CAM_PARAM_REQUEST] = &cmdCamParamRequest;
    cmd_func[CMD_CAM_PARAM_RESPONSE] = &cmdCamParamResponse;

    cmd_func[CMD_SET_RC_VALUES] = &cmdSetRemoteControlValues;

    cmd_func[CMD_RECORD_TELEMETRY] = &cmdRecordTelemetry;
    cmd_func[CMD_REQUEST_TELEMETRY] = &cmdRequestTelemetry;
    cmd_func[CMD_RESPONSE_TELEMETRY] = &cmdResponseTelemetry;

    cmd_func[CMD_RECORD_SENSOR_DUMP] = &cmdSetLogging;
    cmd_func[CMD_GET_MEM_CONTENTS] = &cmdGetMemContents;
    cmd_func[CMD_RUN_GYRO_CALIB] = &cmdRunGyroCalib;
    cmd_func[CMD_GET_GYRO_CALIB_PARAM] = &cmdGetGyroCalibParam;

    
    cmd_func[CMD_ZERO_ESTIMATE] = &cmdZeroEstimate;
    cmd_func[CMD_REQUEST_ATTITUDE] = &cmdRequestAttitude;
    cmd_func[CMD_RESPONSE_ATTITUDE] = &cmdResponseAttitude;
    
    return 1;
    
}

void cmdSetHP(MacPacket packet) {

    cvSetHP();

}

unsigned int cmdQueuePacket(MacPacket packet) {

    return carrayAddTail(input_queue, packet);

}

void cmdProcessBuffer(void) {

    MacPacket packet;
    Payload pld;
    unsigned char command;  

    // Check for unprocessed packet
    //packet = radioDequeueRxPacket();
    packet = carrayPopTail(input_queue);
    if(packet == NULL) { return; }

    pld = macGetPayload(packet);
    command = payGetType(pld);
    if(command < MAX_CMD_FUNC_SIZE) {
        cmd_func[command](packet);
    }
    radioReturnPacket(packet);
    
}

// =============== Private Functions ===========================================

// ====== Networking ===========================================================

static void cmdAddressRequest(MacPacket packet) {
    
    netHandleRequest(packet);
    
}

static void cmdAddressOffer(MacPacket packet) {
    
    netHandleOffer(packet);
    
}

static void cmdAddressAccept(MacPacket packet) {
    
    netHandleAccept(packet);
    
}

static void cmdDirUpdateRequest(MacPacket packet) {

    

}

static void cmdDirUpdateResponse(MacPacket packet) {

    typedef struct {
        unsigned long long UUID;        
        unsigned long timestamp;
        unsigned int address;
    }UpdateEntry;

    Payload pld;
    unsigned int i, num_entries;
    DirEntry entry;
    UpdateEntry *update;

    pld = macGetPayload(packet);
    update = (UpdateEntry*) payGetData(pld);
    num_entries = payGetDataLength(pld)/sizeof(UpdateEntry);

    for(i = 0; i < num_entries; i++) {        
        entry = dirQueryID(update[i].UUID); // Retrieve entry
        if(entry == NULL) {                 // If not seen, create
            entry = dirAddNew();
            if(entry == NULL) { continue; } // Check for creation failure
        // Skip updates older than current info
        } else if(update[i].timestamp < entry->timestamp){ continue; }
        entry->uuid = update[i].UUID;
        entry->address = update[i].address;
        entry->pan_id = netGetLocalPanID();
        entry->timestamp = update[i].timestamp;
    }

}

static void cmdDirDumpRequest(MacPacket packet) {

    Payload pld;
    MacPacket response;
    unsigned int *frame, req_addr, req_pan, i, size;

    pld = macGetPayload(packet);
    frame = (unsigned int*) payGetData(pld);

    req_addr = frame[0];
    req_pan = frame[1];

    // Send all if both addresses 0
    if(req_addr == 0 && req_pan == 0) {

        size = dirGetSize();
        DirEntry entries[size];
        dirGetEntries(entries); // Assume we get size # of entries

        i = 0;
        while(i < size) {
            
            response = radioRequestPacket(sizeof(DirEntryStruct));
            if(response == NULL) { continue; }
            macSetDestAddr(response, macGetSrcAddr(packet));
            pld = macGetPayload(response);
            paySetType(pld, CMD_DIR_DUMP_RESPONSE);
            paySetData(pld, sizeof(DirEntryStruct), (unsigned char*) entries[i]);
            while(!radioEnqueueTxPacket(response));
            i++;
        }
                
    } else {

        DirEntry entry;

        entry = dirQueryAddress(req_addr, req_pan);
        if(entry == NULL) { return; }
        
        while(1) {
            
            response = radioRequestPacket(sizeof(DirEntryStruct));
            if(response == NULL) { continue; }
            macSetDestAddr(response, macGetSrcAddr(packet));
            pld = macGetPayload(response);
            paySetType(pld, CMD_DIR_DUMP_RESPONSE);
            //paySetData(pld, sizeof(DirEntryStruct), (unsigned char*) &entry);
            memcpy(payGetData(pld), entry, sizeof(DirEntryStruct));
            while(!radioEnqueueTxPacket(response));
            break;
        }
    }
    
}

static void cmdDirDumpResponse(MacPacket packet) {

    return;

}

static void cmdRequestClockUpdate(MacPacket packet) {

    clksyncHandleRequest(packet);

}

static void cmdResponseClockUpdate(MacPacket packet) {

    clksyncHandleResponse(packet);

}

// ====== Regulator and Control ===============================================

static void cmdSetRegulatorMode(MacPacket packet) {
    
    Payload pld = macGetPayload(packet);
    //unsigned char status = payGetStatus(pld);
    unsigned char* frame = payGetData(pld);
    
    unsigned char flag = frame[0];

    rgltrSetMode(flag);
    
}

static void cmdSetRegulatorRef(MacPacket packet) {

    Payload pld = macGetPayload(packet);
    Quaternion *ref = (Quaternion*)payGetData(pld);
    
    rgltrSetQuatRef(ref);
    
}

static void cmdSetRegulatorPid(MacPacket packet) {
        
    Payload pld;
    unsigned char *frame;
    PidParamsStruct *params;
    
    pld = macGetPayload(packet);
    frame = payGetData(pld);
    params = (PidParamsStruct*) frame;
    
    rgltrSetYawPid(&params[0]);
    rgltrSetPitchPid(&params[1]);
    rgltrSetRollPid(&params[2]);

}

static void cmdSetRegulatorRateFilter(MacPacket packet) {
    
    Payload pld;
    unsigned int* frame, i, j;
    RateFilterParamsStruct params;

    pld = macGetPayload(packet);
    frame = payGetData(pld);    

    j = 0;
    params.order = frame[j++];
    params.type = frame[j++];
    params.xcoeffs = (float*) (frame + j); // Order + 1 floats per array
    params.ycoeffs = params.xcoeffs + sizeof(float)*(params.order + 1);    
    
    rgltrSetYawRateFilter(&params);
    rgltrSetPitchRateFilter(&params);
    rgltrSetRollRateFilter(&params);
    
}
  
static void cmdSetRemoteControlValues(MacPacket packet) {
    
    Payload pld = macGetPayload(packet);
    float* frame = (float *)payGetData(pld);
        
    // parameters are: thrust, steer, and elevator
    rgltrSetRemoteControlValues(frame[0], frame[1], frame[2]);

}

static void cmdSetRateMode(MacPacket packet) {

    Payload pld = macGetPayload(packet);
    unsigned char flag = *(payGetData(pld));

    if(flag == 0) {
        rateDisable();
    } else if(flag == 1) {
        rateEnable();
    }

}

static void cmdSetRateSlew(MacPacket packet) {

    Payload pld = macGetPayload(packet);
    Rate slew = (Rate)payGetData(pld);
    rateSetGlobalSlew(slew);

}

// ====== Telemetry and Sensors ===============================================
static void cmdSetLogging(MacPacket packet) {

    Payload pld;    
    unsigned char *frame, flag;

    pld = macGetPayload(packet);
    frame = payGetData(pld);
    flag = frame[0];

    if(flag) {
        telemStartLogging();
    } else {
        telemStopLogging();
    }

}

static void cmdGetMemContents(MacPacket packet) {

    Payload pld;
    MacPacket data_packet;
    unsigned char *frame;
    DfmemGeometryStruct geo;

    pld = macGetPayload(packet);
    frame = payGetData(pld);
    dfmemGetGeometryParams(&geo);

    unsigned int start_page = frame[0] + (frame[1] << 8);
    unsigned int end_page = frame[2] + (frame[3] << 8);
    unsigned int tx_data_size = frame[4] + (frame[5] << 8);
    unsigned int page, j;
    unsigned char count = 0;
    
    // Send back memory contents
    for (page = start_page; page < end_page; ++page) {
        j = 0;
        while (j + tx_data_size <= geo.bytes_per_page) {
            data_packet = NULL;
            while(data_packet == NULL) {
                data_packet = radioRequestPacket(tx_data_size);
            }

            macSetDestAddr(data_packet, 0x1020);
            macSetDestPan(data_packet, 0x1001);
            pld = macGetPayload(data_packet);

            dfmemRead(page, j, tx_data_size, payGetData(pld));

            paySetStatus(pld, count++);
            paySetType(pld, CMD_RESPONSE_TELEMETRY);
            while(!radioEnqueueTxPacket(data_packet));
            j += tx_data_size;
            delay_ms(20);
        }

    }

    // Signal end of transfer    
    LED_GREEN = 0; LED_RED = 0; LED_ORANGE = 0;
    
}

static void cmdRunGyroCalib(MacPacket packet) {
    
    Payload pld = macGetPayload(packet);
    unsigned int* frame = (unsigned int*) payGetData(pld);
    
    unsigned int count = frame[0];

    radioSetWatchdogState(0);
    gyroRunCalib(count);    
    radioSetWatchdogState(1);

}

static void cmdGetGyroCalibParam(MacPacket packet) {
        
    //Payload pld = macGetPayload(packet);
    //unsigned char status = payGetStatus(pld);
    //unsigned char* frame = payGetData(pld);
    unsigned int srcAddr = macGetSrcAddr(packet);
    
    Payload pld;
    MacPacket response;
    
    response = radioRequestPacket(12);
    if(response == NULL) { return; }
    macSetDestAddr(response, srcAddr);
    pld = response->payload;
    paySetData(pld, 12, gyroGetCalibParam());
    paySetStatus(pld, 0);
    paySetType(pld, CMD_GET_GYRO_CALIB_PARAM);
    while(!radioEnqueueTxPacket(response));
}

static void cmdRecordTelemetry(MacPacket packet) {

    

}

static void cmdRequestTelemetry(MacPacket packet) {

    telemSendB(macGetSrcAddr(packet));

}

static void cmdResponseTelemetry(MacPacket packet) {

    return; // Do nothing
    
}

static void cmdSetEstimateRunning(MacPacket packet) {
        
    Payload pld = macGetPayload(packet);
    //unsigned char status = payGetStatus(pld);
    unsigned char* frame = payGetData(pld);
    
    
    if (frame[0] == 0) {
        attSetRunning(0);
    } else {
        attSetRunning(1);
    }
}

// ====== Camera and Vision ===================================================
// TODO: Use a struct to simplify the packetization
static void cmdRequestRawFrame(MacPacket packet) {
    
    unsigned int srcAddr, srcPan, height, width, i, temp;
    unsigned int sent, to_send, block_size = 75;
    MacPacket response;
    Payload pld;
    CamFrame frame;
    CamRow *row;
    CvResultStruct info;

    srcAddr = macGetSrcAddr(packet);
    srcPan = macGetSrcPan(packet);    

    frame = NULL;
    while(frame == NULL) {
        frame = camGetFrame();
    }           

    cvProcessFrame(frame, &info);    

    height = DS_IMAGE_ROWS;
    width = DS_IMAGE_COLS;

    for(i = 0; i < height; i++) {        
        row = &(frame->pixels[i]);
        to_send = width;
        while(to_send > 0) {            
            response = radioRequestPacket(block_size + 6);
            if(response == NULL) { continue; }
            pld = macGetPayload(response);
            paySetType(pld, CMD_RAW_FRAME_RESPONSE);
            paySetStatus(pld, 0);
            macSetDestAddr(response, srcAddr);
            macSetDestPan(response, srcPan);
            temp = frame->frame_num;
            paySetData(pld, 2, (unsigned char *)&temp);
            temp = i;
            payAppendData(pld, 2, 2, (unsigned char*)&temp);
            temp = width - to_send;
            payAppendData(pld, 4, 2, (unsigned char*)&temp);
            temp = (block_size < to_send) ? block_size : to_send;
            payAppendData(pld, 6, temp, *row + (width - to_send));

            while(!radioEnqueueTxPacket(response));

            to_send = to_send - temp;

        }

    }
    sent = 0;
    while(!sent) {        
        response = radioRequestPacket(10);
        if(response == NULL) { continue; }
        pld = macGetPayload(response);
        paySetType(pld, CMD_CENTROID_REPORT);
        paySetStatus(pld, 1);
        macSetDestAddr(response, srcAddr);
        macSetDestPan(response, srcPan);
        temp = info.centroid[0];
        paySetData(pld, 2, (unsigned char*)&temp);
        temp = info.centroid[1];
        payAppendData(pld, 2, 2, (unsigned char*)&temp);
        temp = info.max[0];
        payAppendData(pld, 4, 2, (unsigned char*)&temp);
        temp = info.max[1];
        payAppendData(pld, 6, 2, (unsigned char*)&temp);
        temp = info.max_lum;
        payAppendData(pld, 8, 1, (unsigned char*)&temp);
        temp = info.avg_lum;
        payAppendData(pld, 9, 1, (unsigned char*)&temp);
        while(!radioEnqueueTxPacket(response));
        sent = 1;
    }
    camReturnFrame(frame);

}

static void cmdResponseRawFrame(MacPacket packet) {
    return; // Do nothing
}

static void cmdSetBackgroundFrame(MacPacket packet) {

    CamFrame frame;

    frame = NULL;
    while(frame == NULL) {
        frame = camGetFrame();
    }

    camReturnFrame(cvSetBackgroundFrame(frame));

}

static void cmdCamParamRequest(MacPacket packet) {

    Payload pld;
    CamParamStruct params;
    MacPacket response;
    
    pld = macGetPayload(packet);
    camGetParams(&params);
    
    response = radioRequestPacket(sizeof(CamParamStruct));
    if(response == NULL) { return; }
    
    macSetDestAddr(response, macGetSrcAddr(packet));
    pld = macGetPayload(response);
    paySetType(pld, CMD_CAM_PARAM_RESPONSE);
    paySetStatus(pld, 0);
    paySetData(pld, sizeof(CamParamStruct), (unsigned char*)&params);

    while(!radioEnqueueTxPacket(response));


}

static void cmdCamParamResponse(MacPacket packet) {

    Payload pld;
    unsigned char *frame;
    CamParamStruct *params;
    LStrobeParamStruct lstrobe_params;
    DirEntry entry;
    unsigned int addr, pan;
    
    pld = macGetPayload(packet);
    frame = payGetData(pld);
    params = (CamParamStruct*) frame;
        
    addr = macGetSrcAddr(packet);
    pan = macGetSrcPan(packet);
    entry = dirQueryAddress(addr, pan);
    
    if(entry == NULL) { return; }
    entry->frame_period = params->frame_period;
    entry->frame_start = params->frame_start;

    lstrobe_params.period = 5*(params->frame_period/4);
    lstrobe_params.period_offset = (params->frame_start/4) % (params->frame_period/4);
    lstrobe_params.on_time = 625/4; // 1 ms
    lstrobe_params.off_time = lstrobe_params.period - lstrobe_params.on_time;
    lstrobeSetParam(&lstrobe_params);
    lstrobeStart();
    
}

static void cmdZeroEstimate(MacPacket packet) {

    attReset();
    //xlReadXYZ();
    //attZero();

}

static void cmdRequestAttitude(MacPacket packet) {

    telemSendAttitude(macGetSrcAddr(packet));

}

static void cmdResponseAttitude(MacPacket packet) {

    // Write me!
    return;

}

/*-----------------------------------------------------------------------------
 *          AUX functions
-----------------------------------------------------------------------------*/
static void cmdEcho(MacPacket packet) {
        
    Payload pld = macGetPayload(packet);
    unsigned char status = payGetStatus(pld);
    unsigned char* frame = payGetData(pld);
    unsigned int length = payGetDataLength(pld);
    unsigned int srcAddr = macGetSrcAddr(packet);
    
    MacPacket response;
    
    response = radioRequestPacket(length);
    if(response == NULL) { return; }
    macSetDestAddr(response, srcAddr);
    
    pld = response->payload;
    paySetData(pld, length, frame);
    paySetStatus(pld, status);
    paySetType(pld, CMD_ECHO);
    
    while(!radioEnqueueTxPacket(response));
}

static void cmdNop(MacPacket packet) {
            
    Nop();
}




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
 * iBird Main Loop
 *
 * by Stan Baek
 * 
 * Description:
 *    Main processing loop for iBird ornithopter platform
 *
 * v. beta
 *
 * Revisions:
 *  Stan Baek          2010-07-08        Initial implementation
 *  Humphrey Hu        2011-06-12        Changed to interleaved controller
 *  Humphrey Hu        2011-10-26        Changed to asynchronous camera capture                    
 */

// TODO: Slotted LED strobing
// TODO: Obstacle avoidance
// TODO: Directories and telemetry broadcasts

// ==== REFERENCES =============================================

#include "cmd.h"
#include "init_default.h"

// Utils
#include "counter.h"
#include "directory.h"
#include "utils.h"
#include "telemetry.h"
#include "sys_clock.h"
#include "battery.h"
#include "carray.h"
#include "ppool.h"

// Software Modules
#include "cv.h"
#include "regulator.h"
#include "attitude.h"
#include "net.h"
#include "clock_sync.h"

// Device Drivers
#include "led.h"
#include "timer.h"
#include "ovcam.h"
#include "cam.h"
#include "xl.h"
#include "gyro.h"
//#include "mpu6050.h"
#include "dfmem.h"
#include "radio.h"

#include "sync_servo.h"
#include "spi_controller.h"
#include "lstrobe.h"
#include "larray.h"
#include "bams.h"
#include <math.h>
#include "cmd_const.h"
#include <stdlib.h>
#include <string.h>

// ==== CONSTANTS =============================================
#define FCY                         (40000000)  // 40 MIPS   
#define REG_FCY                     (200)       // 200 Hz
#define RADIO_FCY                   (250)       // 250 Hz
#define RADIO_TX_QUEUE_SIZE         (40)        // 40 Outgoing
#define RADIO_RX_QUEUE_SIZE         (40)        // 40 Incoming

#define DIRECTORY_SIZE              (10)
#define NUM_CAM_FRAMES              (1)         // Camera driver frames

// ==== FUNCTION STUBS =========================================
static void processRadioBuffer(void);

static void setupAll(void);
static void setRandomSeed(void);
static void attemptNetworkConfig(void);
static void attemptClockSync(void);

static void setupTimer5(unsigned int per);
static void setupTimer6(unsigned int per);
void _T5Interrupt(void);
void _T6Interrupt(void);

// ==== STATIC VARIABLES =======================================

static CamFrameStruct cam_frames[NUM_CAM_FRAMES];

// ==== FUNCTION BODIES ========================================
int main(void) {
 
    unsigned long prev_millis, now, last_packet;
    unsigned char led_state;
    unsigned int phase;
    // CamFrame frame;
    // FrameInfoStruct info;

    prev_millis = 0;
    led_state = 0;    
    last_packet = 0;

    setupAll();    

    // Non-time-critical background tasks
    while(1) {
    
        processRadioBuffer();
        cmdProcessBuffer();

        now = sclockGetGlobalMillis();
        phase = now % 2000;

        // Blink LED at 1 Hz
        if(phase > 1000 && led_state == 0) {
            LED_GREEN = 1;            
            led_state = 1;
        } else if(phase < 1000 && led_state == 1) {
            LED_GREEN = 0;            
            led_state = 0;
        }

        // frame = camGetFrame();
        // if(frame != NULL) {
            // cvProcessFrame(frame, &info);
        // }
        // camReturnFrame(frame);

    }

} // End main

void processRadioBuffer(void) {

    MacPacket packet;
    
    packet = radioDequeueRxPacket();
    if(packet == NULL) { return; }

    // If enqueue fails, clean up packet
    if(cmdQueuePacket(packet) == 0) {
        radioReturnPacket(packet);
    }

}

// Set up hardware and software
void setupAll(void) {

    unsigned int i;    

    SetupClock(); // Setup clock and ports
    SwitchClocks();
    SetupPorts();

    sclockSetup(); // System clock
    batSetup(); // Battery monitor
    spicSetup(); // SPI-DMA controller
    ppoolInit(); // Initialize packet pool
    dirInit(DIRECTORY_SIZE); // Initialize directory

    LED_GREEN = 1; // First stage initialization clear

    // Set up peripherals
    // Note: OV7660 I2C operates at 100 kHz on the same bus
    // as the accelerometer. Make sure to set up camera module first!
    dfmemSetup(); // Flash memory device
    camSetup(cam_frames, NUM_CAM_FRAMES); // Camera device
    servoSetup(); // Soft servo control
    
    xlSetup();
    xlSetRange(16); // +- 16 g range
    xlSetOutputRate(0, 0x0c); // 800 Hz
    gyroSetup();
    gyroSetDeadZone(35);

    // Seeds random number generation using IMU sensors
    setRandomSeed(); 

    cmdSetup(RADIO_RX_QUEUE_SIZE); // Command packet processing module

    radioInit(RADIO_TX_QUEUE_SIZE, RADIO_RX_QUEUE_SIZE);
    setupTimer6(RADIO_FCY); // Set up radio and buffer loop timer
    
    netSetup(); // Set up networking module
    attemptNetworkConfig();
    radioSetSrcAddr(netGetLocalAddress());
    radioSetSrcPanID(netGetLocalPanID());    

    LED_ORANGE = 1; // Second stage initialization clear

    // Set up high level software modules
    attSetup(1.0/REG_FCY); // Pose estimation
    rgltrSetup(1.0/REG_FCY); // Pose control
    setupTimer5(REG_FCY); // Set up control loop timer
    attSetRunning(1);
    
    cvSetup(); // Set up computer vision module

//    clksyncSetup();
//    clksyncSetMasterAddr(DEFAULT_SYNC_ADDR, DEFAULT_SYNC_PAN);
//    attemptClockSync();

    LED_RED = 1; // Third stage initialization clear

    // Indicate set up completion
    for(i = 0; i < 4; i++) {
        delay_ms(25);
        LED_RED = ~LED_RED;
        delay_ms(25);
        LED_GREEN = ~LED_GREEN;
        delay_ms(25);
        LED_ORANGE = ~LED_ORANGE;
    }

    LED_RED = 0;
    LED_GREEN = 0;
    LED_ORANGE = 0;

    camStart();    // Start camera capture
    EnableIntT5; // Start control loop

    radioSetWatchdogState(1);
    radioSetWatchdogTime(400);

}

/**
 * Seeds the system pseudo-random number generator using the accelerometer and
 * gyro
 */
static void setRandomSeed(void) {

    unsigned int seed;
    int xlData[3], gyroData[3];

    xlGetXYZ((unsigned char*) xlData);
    gyroGetXYZ((unsigned char*) gyroData);
    seed = xlData[0] ^ xlData[1] ^ xlData[2] ^ gyroData[0] ^ gyroData[1]
            ^ gyroData[2];
    srand(seed);
    
}

static void attemptNetworkConfig(void) {

    radioSetWatchdogState(1);
    radioSetWatchdogTime(2000);
    
    while(!netAddressReceived()) {
        netRequestAddress();        
        delay_ms(300);
        processRadioBuffer();
        cmdProcessBuffer();

    }
    
}

static void attemptClockSync(void) {

    radioSetWatchdogState(1);
    radioSetWatchdogTime(300);
    
    while(!clksyncIsDone()) {
        clksyncSync();        
        delay_ms(50);
        processRadioBuffer();
        cmdProcessBuffer();

    }

}

/**
 * Interrupt handler for Timer 5
 * Polls sensors, estimates attitude, and runs controller at
 * regular interval. This timer is the lowest priority with
 * the highest execution time.
 */
void __attribute__((interrupt, no_auto_psv)) _T5Interrupt(void) {

    //xlReadXYZ();
    gyroReadXYZ();
    attEstimatePose();    
    rgltrRunController();
    
    _T5IF = 0;

}

void __attribute__((interrupt, no_auto_psv)) _T6Interrupt(void) {

    radioProcess();
    
    _T6IF = 0;

}

/**
 * Control loop timer setup
 */
static void setupTimer5(unsigned int fs) {

    unsigned int con_reg, period;
    
    con_reg =   T5_ON &         // Timer on
                T5_IDLE_STOP &  // Stop timer when idle
                T5_GATE_OFF &   // Gated mode off
                T5_PS_1_8 &     // Prescale 1:8
                T5_SOURCE_INT;  // Internal clock source

    // period value = Fcy/(prescale*Ftimer)
    period = FCY/(8*fs); 

    OpenTimer5(con_reg, period);
    ConfigIntTimer5(T5_INT_PRIOR_3 & T5_INT_OFF);

}

static void setupTimer6(unsigned int fs) {

    unsigned int con_reg, period;

    con_reg =   T6_ON &         // Timer on
                T6_IDLE_STOP &  // Stop timer when idle
                T6_GATE_OFF &   // Gated mode off
                T6_PS_1_8 &     // Prescale 1:8
                T6_SOURCE_INT &  // Internal clock source
                T6_32BIT_MODE_OFF; // 16 bit mode
    
    // period value = Fcy/(prescale*Ftimer)
    period = FCY/(8*fs);

    OpenTimer6(con_reg, period);
    ConfigIntTimer6(T6_INT_PRIOR_3 & T6_INT_ON);

}

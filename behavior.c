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
* I-Bird State Machine
*
* by Humphrey Hu
*
* v.beta
*
* Revisions:
*  Humphrey Hu     2011-09-04      Initial implementation
*  Humphrey Hu     2012-03-21      Update to new code base
* 
* Notes:
*
* TODO:
*/

// ==== REFERENCES ==========================================
#include "camera.h"
#include "cv.h"
#include "attitude.h"
#include "regulator.h"
#include "behavior.h"
#include "sys_clock.h"

// ==== CONSTANTS =========================================== 
#define TRACK_MIN_PIXELS             (4) // Min number of pixels visible to consider a centroid valid
#define REACQUIRE_MOTION_PERIOD       (50)    
#define CIRCLE_MOTION_PERIOD          (150)
#define SEARCH_MOTION_PERIOD         (200)

#define TRACK_PITCH_OFFSET           (80.0)
#define TRACK_YAW_OFFSET             (0.0)

#define REACQUIRE_PITCH_OFFSET        (80.0)
#define REACQUIRE_YAW_OFFSET          (0.0)
#define REACQUIRE_YAW_OFFSET_F        (0.524)
#define REACQUIRE_TURN_THRESHOLD      (0.524)    // 30 degrees in radians
#define REACQUIRE_MAX_TIME            (20)

#define CIRCLE_PITCH_OFFSET           (78.0)
#define CIRCLE_YAW_OFFSET             (0.0)

#define SEARCH_PITCH_OFFSET          (78.0)
#define SEARCH_YAW_OFFSET            (45.0)
#define SEARCH_PITCH_INCREMENT       (0.0)
#define SEARCH_YAW_INCREMENT         (0.0)
#define SEARCH_YAW_MINIMUM           (0.0)
#define SEARCH_YAW_MAXIMUM           (0.0)
#define SEARCH_PERIOD                (25)

#define DEFAULT_YAW_OFFSET              (0.0)
#define DEFAULT_PITCH_OFFSET            (80.0)

typedef enum {
    BEHAV_CIRCLE,
    BEHAV_TRACK,
    BEHAV_REACQUIRE,
    BEHAV_SEARCH,    
} BehaviorState;

// ==== STATIC VARIABLES ====================================
static unsigned char is_running = 0;
static unsigned char is_ready = 0;

static BehaviorState behav_state = BEHAV_CIRCLE;

// Reacquiring behav_state variables
static unsigned int reacquiringStartTime;

// Searching behav_state variables
static unsigned int searchingStartTime;

// ==== FUNCTION STUBS ======================================
static void runTrack(void);
static void runReacquire(void);
static void runSearch(void);
static void runCircl(void);

// Valid transition handlers
static void transitionTrackReacquire(void);
static void transitionReacquireTrack(void);
static void transitionReacquireSearch(void);
static void transitionSearchTrack(void);
static void transitionCircle(void);
static void transitionSearch(void);

static void resetOffsets(void);
// ==== FUNCTION BODIES =====================================

void behavSetup(void) {

    is_ready = 0;
    is_running = 0;
    behav_state = BEHAV_CIRCLE;
    
    behavReset();
    
}

void behavReset(void) {

    transitionCircle();    
    is_running = 0;
    

}

void behavSetState(unsigned char flag) {

    if(flag == BEHAVIOR_SEARCH) {
        transitionSearch();
    } else if(flag == BEHAVIOR_CIRCL) {
        transitionCircl();
    }
    
}

void behavSetRunning(unsigned char flag) {
    
    is_running = flag;
    
}

unsigned char behavIsRunning(void) {

    return is_running;
    
}

// Note: We don't expect to see stuttering since the camera capture
// is synchronous
void behavRunBehavior(void) {

    if(!is_ready) { return; }
    if(!is_running) { return; }

    switch(behav_state) {
        
    case BEHAV_TRACK:
        runTrack();
        break;
        
    case BEHAV_REACQUIRE:
        runReacquire();
        break;
        
    case BEHAV_SEARCH:
        runSearch();
        break;
        
    case BEHAV_CIRCLE:
        runCircle();
        break;
        
    }
    
}

void runTrack(void) {

    CamFrame frame;
    FrameInfo info;    
    int cX, cY;

    frame = camGetFrame();
    if(frame == NULL) { return; }
    
    info = cvProcessFrame(frame);
    camReturnFrame(frame);
        
    // If enough visible pixels
    if(info->mass > TRACK_MIN_PIXELS) {                       
                       
        cX = info->centroid[0] - info->offset[0];
        cY = info->centroid[1] - info->offset[1];
        
        rgltrSetYawRef(cX + attGetYaw());
        rgltrSetPitchRef(cY + attGetPitch());
        
        return;
        
    } 
    
    // Else transition to reacquiring behav_state    
    transitionTrackReacquire();    
    
}

void runReacquire(void) {

    FrameInfo frame;
    unsigned long time;
    
    frame = cvGetFrameInfo();
    
    // If target found, transition to tracking behav_state
    if(frame->mass > TRACK_MIN_PIXELS) {
        
        transitionReacquireTrack();
        runTrack();
        return;
        
    } 
    
    // Else attempt to reacquire
    // Check time elapsed since reacquisition start
    time = sclockTimeMillis();
    
    // If max time exceeded, transition to search behav_state
    if(time - reacquire_start_time > REACQUIRE_MAX_TIME) {
        
        transitionReacquireSearch();        
        return;
        
    } 
    
}


void runSearch(void) {

    FrameInfo frame;
    unsigned int time;
    
    frame = cvGetFrameInfo();
    
    // If target found, transition to tracking behav_state
    if(frame->mass > TRACK_MIN_PIXELS) {
        
        transitionSearchTrack();        
        return;
        
    } 
    
    // Check time passed
    time = cntrRead(clock);
    rgltrSetRemoteControlValues(SEARCH_PITCH_OFFSET, SEARCH_YAW_OFFSET);

}

void runCircle(void) {

    rgltrSetRemoteControlValues(currentPitchOffset, currentYawOffset);
    
}

void transitionTrackReacquire(void) {

    resetOffsets();                    // Reset to defaults
    //rgltrSetRemoteControl(0);        // Disable RC
    
    // Mark start time of reacquisition process
    reacquiringStartTime = cntrRead(clock);
    
    if(lastRelativeHeading > REACQUIRE_TURN_THRESHOLD) {
        lastRelativeHeading += REACQUIRE_YAW_OFFSET_F;
    } else if(lastRelativeHeading < -REACQUIRE_TURN_THRESHOLD) {
        lastRelativeHeading += -REACQUIRE_YAW_OFFSET_F;
    }
    //rgltrSetYawRef(lastRelativeHeading + attGetYaw());
    behav_state = BEHAV_REACQUIRE;
    
}

void transitionReacquireTrack(void) {
    
    resetOffsets();                    // Reset to defaults
    //rgltrSetRemoteControl(0);        // Disable RC
    behav_state = BEHAV_TRACK;

}

void transitionReacquireSearch(void) {
    
    resetOffsets();                    // Reset to defaults
    //rgltrSetRemoteControl(1);        // Enable RC mode
    rgltrSetRemoteControlValues(SEARCH_PITCH_OFFSET, SEARCH_YAW_OFFSET);
    searchingStartTime = cntrRead(clock);    // Read start time
    behav_state = BEHAV_SEARCH;
    
}

void transitionSearchTrack(void) {

    resetOffsets();                    // Reset to defaults
    //rgltrSetRemoteControl(0);        // Disable RC
    behav_state = BEHAV_TRACK;

}

// TODO: Merge w/ transitionReacquisitionSearching?
void transitionSearch(void) {

    resetOffsets();                    // Reset to defaults
    //rgltrSetRemoteControl(1);        // Enable RC mode
    rgltrSetRemoteControlValues(SEARCHING_PITCH_OFFSET, SEARCHING_YAW_OFFSET);
    behav_state = BEHAV_SEARCH;
    
}

void transitionCircle(void) {
    
    resetOffsets();                    // Reset to defaults
    //rgltrSetRemoteControl(0);        // Disable RC mode
    //rgltrSetYawRef(attGetYaw());    // Set to fly straight
    behav_state = BEHAV_CIRCLE;
    
}

void resetOffsets(void) {

    rgltrSetRemoteControlValues(DEFAULT_PITCH_OFFSET, DEFAULT_YAW_OFFSET);

}

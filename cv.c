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
* Online computer vision module
*
* by Humphrey Hu
*
* v. beta
*
* Revisions:
*  Humphrey Hu      2011-07-01      Initial implementation
*  Humphrey Hu      2012-02-16      Complete rewrite to use camera driver
*/

#include "cv.h"
#include "cam.h"
#include "counter.h"
#include "utils.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#define CV_INFO_BUFFER_SIZE		    (5)
#define CV_BINARY_CUTOFF_DEFAULT	(140)
#define CV_BINARY_MIN_DEFAULT		(40)

// State info
static unsigned char is_ready, is_running;
static unsigned int frame_size[2];

// Centroid finding variables
static long centroid_x_acc, centroid_y_acc;

static CamFrame background_frame;

// Private functions
static void cvBackgroundSubtractFrame(CamFrame frame, FrameInfo info);
static void cvCentroidFrame(CamFrame frame, FrameInfo info);
static void cvMaxPixelFrame(CamFrame, FrameInfo info);

// Setup the CV module
void cvSetup(void) {
    
    is_ready = 0;
    is_running = 0;

    camGetFrameSize(frame_size); // Look up driver output size
       
    background_frame = NULL;
    centroid_x_acc = 0;
    centroid_y_acc = 0;

    is_ready = 1;

}

void cvProcessFrame(CamFrame frame, FrameInfo info) {

    if(!is_ready) { return; } // Module readiness quick fail       

    cvBackgroundSubtractFrame(frame, info);
    cvCentroidFrame(frame, info);
    cvMaxPixelFrame(frame, info);

}

CamFrame cvSetBackgroundFrame(CamFrame frame) {

    CamFrame old;

    old = background_frame;
    background_frame = frame;

    return old;
    
}

void cvBackgroundSubtractFrame(CamFrame frame, FrameInfo info) {

    unsigned int i, j, width, height;
    unsigned char *raw_row, *bg_row;    

    if(background_frame == NULL) { return; }

    width = frame->num_cols;
    height = frame->num_rows;

    for(i = 0; i < height; i++) {
        raw_row = frame->rows[i]->pixels;
        bg_row = background_frame->rows[i]->pixels;
        for(j = 0; j < width; j++) {
            raw_row[j] = (raw_row[j] < bg_row[j]) ? 0 : (raw_row[j] - bg_row[j]);
        }
    }

}

void cvMaxPixelFrame(CamFrame frame, FrameInfo info) {

    CamRow *row_array;    
    unsigned int width, height, i, j, max_val, max_loc[2];
    unsigned char *pixels, val;

    width = frame->num_rows;
    height = frame->num_cols;
    row_array = frame->rows;
    max_val = 0;    
    max_loc[0] = 0;
    max_loc[1] = 1;

    for(i = 0; i < height; i++) {
        pixels = row_array[i]->pixels;
        for(j = 0; j < width; j++) {
            val = pixels[j];
            if(val > max_val) {
                max_val = val;
                max_loc[0] = j;
                max_loc[1] = i;
            }
        }
    }

    info->max[0] = max_loc[0];
    info->max[1] = max_loc[1];
    info->max_lum = max_val;

}

void cvCentroidFrame(CamFrame frame, FrameInfo info) {

    CamRow *row_array;
    unsigned long x_acc, y_acc, l_acc, temp;
    unsigned long x_sub_acc, y_sub_acc;
    unsigned int width, height, i, j, val;
    unsigned char *pixels;    

    x_acc = 0; // Initialize
    y_acc = 0;
    l_acc = 0;

    width = frame->num_cols; // Read frame info
    height = frame->num_rows;
    row_array = frame->rows;
    
    for(i = 0; i < height; i++) {        
        pixels = row_array[i]->pixels; // Get row pixels
        x_sub_acc = 0; // Reset subaccumulators
        y_sub_acc = 0;        
        for(j = 0; j < width; j++) {
            val = pixels[j];
            x_sub_acc += val*j;
            y_sub_acc += val; // Sum luminosity for multiply later
        }
        l_acc += y_sub_acc; // Accumulate luminosity
        y_sub_acc *= i; // Commutative multiplication
        x_acc += x_sub_acc; // Accumulate
        y_acc += y_sub_acc;
    }   
    
    info->valid = 0;
    info->mass = l_acc;
    info->avg_lum = l_acc/(width*height);

    if(l_acc == 0) {
        info->centroid[0] = 0;
        info->centroid[1] = 0;
    } else {
        temp = x_acc/l_acc;
        info->centroid[0] = (unsigned int) temp;
        temp = y_acc/l_acc;
        info->centroid[1] = (unsigned int) temp;
    }
    info->offset[0] = width/2;
    info->offset[1] = height/2;
    info->valid = 1;   

}

// =========== Private Functions ===============================================

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

#include "attitude.h"
#include "cv.h"
#include "cam.h"
#include "counter.h"
#include "utils.h"
#include "bams.h" // For fast trig

#include <math.h>
#include <stdlib.h>
#include <string.h>

#define CV_INFO_BUFFER_SIZE		    (5)
#define CV_BINARY_CUTOFF_DEFAULT	(140)
#define CV_BINARY_MIN_DEFAULT		(40)

// State info
static unsigned char is_ready, is_running;

// Centroid finding variables
static long centroid_x_acc, centroid_y_acc;

static CamFrame *background_frame;

// Private functions
static void cvBackgroundSubtractFrame(CamFrame *frame, FrameInfo info);
static void cvCentroidFrame(CamFrame *frame, FrameInfo info);
static void cvMaxPixelFrame(CamFrame *frame, FrameInfo info);
static void cvRotateFrame(CamFrame *frame, bams16_t theta);
static void cvShiftHoriz(CamFrame *frame, int num);
static void cvShiftVert(CamFrame *frame, int num);
static void colShift(CamFrame *frame, unsigned int col, 
                    unsigned int row_dst, unsigned int row_src, unsigned int num);
static void colSet(CamFrame *frame, unsigned int col, unsigned int row_dst,
                    unsigned char val, unsigned int num);
// static void vertmove(unsigned char *dst, unsigned char *src, 
                    // unsigned int num, unsigned int width);
// static void vertset(unsigned char *dst, unsigned char val, 
                    // unsigned int num, unsigned int width);
                    
// Setup the CV module
void cvSetup(void) {        
    
    is_ready = 0;
    is_running = 0;    
       
    background_frame = NULL;
    centroid_x_acc = 0;
    centroid_y_acc = 0;

    is_ready = 1;

}

void cvProcessFrame(CamFrame *frame, FrameInfo info) {

    if(!is_ready) { return; } // Module readiness quick fail       

    cvBackgroundSubtractFrame(frame, info);
    cvCentroidFrame(frame, info);
    cvMaxPixelFrame(frame, info);
    cvRotateFrame(frame, -attGetYawBAMS());

}

CamFrame* cvSetBackgroundFrame(CamFrame *frame) {

    CamFrame *old;

    old = background_frame;
    background_frame = frame;

    return old;
    
}

// =========== Private Functions ===============================================
/**
 * Subtract the set background frame from the input frame.
 *
 * @param frame - Input frame
 * @param info - Info struct to populate
 */
void cvBackgroundSubtractFrame(CamFrame *frame, FrameInfo info) {

    unsigned int i, j, width, height;
    unsigned char *raw_row, *bg_row;    

    if(background_frame == NULL) { return; }

    width = DS_IMAGE_COLS; // frame->num_cols;
    height = DS_IMAGE_ROWS; // frame->num_rows;

    for(i = 0; i < height; i++) {
        raw_row = frame->pixels[i]; //frame->rows[i]->pixels;
        bg_row = background_frame->pixels[i]; // background_frame->rows[i]->pixels;
        for(j = 0; j < width; j++) {
            raw_row[j] = (raw_row[j] < bg_row[j]) ? 0 : (raw_row[j] - bg_row[j]);
        }
    }

}

/**
 * Find the maximum luminosity pixel in the input frame.
 *
 * @param frame - Input frame
 * @param info - Info struct to populate
 */
void cvMaxPixelFrame(CamFrame *frame, FrameInfo info) {
    
    unsigned int width, height, i, j, max_val, max_loc[2];
    unsigned char val;    

    width = DS_IMAGE_COLS;
    height = DS_IMAGE_ROWS;    
    max_val = 0;    
    max_loc[0] = 0;
    max_loc[1] = 1;

    for(i = 0; i < height; i++) {        
        for(j = 0; j < width; j++) {
            val = frame->pixels[i][j];
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

/**
 * Find the frame luminosity centroid of the input frame.
 *
 * @param frame - Input frame
 * @param info - Info struct to populate
 */
void cvCentroidFrame(CamFrame *frame, FrameInfo info) {
    
    unsigned long x_acc, y_acc, l_acc, temp;
    unsigned long x_sub_acc, y_sub_acc;
    unsigned int width, height, i, j, val;    

    x_acc = 0; // Initialize
    y_acc = 0;
    l_acc = 0;

    width = DS_IMAGE_COLS; // frame->num_cols; // Read frame info
    height = DS_IMAGE_ROWS; // frame->num_rows;    
    
    for(i = 0; i < height; i++) {            
        x_sub_acc = 0; // Reset subaccumulators
        y_sub_acc = 0;        
        for(j = 0; j < width; j++) {
            val = frame->pixels[i][j];
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

static void cvRotateFrame(CamFrame *frame, bams16_t theta) {

    float alpha, beta;
    int horiz_shift, vert_shift;
    
    alpha = bams16Tan(theta/2);
    beta = bams16Sin(theta);

    // Calculate total horizontal shift of upper row
    horiz_shift = (int) (alpha*(DS_IMAGE_ROWS/2));
    // Calculate total horizontal shift of right column
    vert_shift = (int) (beta*(DS_IMAGE_COLS/2));
    
    cvShiftHoriz(frame, horiz_shift);    
    cvShiftVert(frame, vert_shift);
    cvShiftHoriz(frame, horiz_shift);

}

// Frame is pointer to CamFrame object
// num is number of pixels top row (row 0) is shifted to the right
static void cvShiftHoriz(CamFrame *frame, int num) {

    int shift, half_height, i, width, height;    
    unsigned char *row;
    
    height = (int) DS_IMAGE_ROWS; // frame->num_rows;
    width = (int) DS_IMAGE_COLS; // frame->num_cols;
    half_height = (int) height/2;
    
    for(i = 0; i < height; i++) {
            
        row = frame->pixels[i];
        shift = ((i - half_height)*num)/half_height;
        
        if(shift == 0) {
            // Do nothing
        } else if(shift > 0) {
            
            memmove(row + shift, row, width - shift);
            memset(row, 0, shift);
        
        } else { // shift < 0
        
            shift = -shift;
            memmove(row, row + shift, width - shift);
            memset(row + width - shift, 0, shift);
            
        }

        int j;
        j = 0;

    }

}

// Num is number of pixels leftmost column (col 0) is shifted down
static void cvShiftVert(CamFrame *frame, int num) {

    int shift, half_width, i, width, height;        
    
    height = (int) DS_IMAGE_ROWS; // frame->num_rows;
    width = (int) DS_IMAGE_COLS; // frame->num_cols;
    half_width = (int) width/2;
    
    for(i = 0; i < width; i++) {
                    
        shift = ((i - half_width)*num)/half_width;
        
        if(shift == 0) {
            // Do nothing
        } else if(shift > 0) {
            
            colShift(frame, i, 0, shift, height - shift);
            colSet(frame, i, height - shift, 0, shift); 
            
        } else { // shift < 0
        
            shift = -shift;
            colShift(frame, i, shift, 0, height - shift); 
            colSet(frame, i, 0, 0, shift);
            
        }

        int j;
        j = 0;

    }

}

static void colShift(CamFrame *frame, unsigned int col,
                    unsigned int row_dst, unsigned int row_src, unsigned int num) {

    int i, shift, step;
    unsigned int cnt;    
        
    shift = row_dst - row_src;
    
    if(shift == 0) {
        return;
    } else if(shift > 0) { // Start from tail
        step = -1;
        i = num - 1;
    } else { // shift < 0, start from head
        step = 1;
        i = 0;
    }
    
    cnt = num;
    while(cnt--) {
        frame->pixels[row_dst + i][col] = 
            frame->pixels[row_src + i][col];
        i = i + step;
    }
    
}

static void colSet(CamFrame *frame, unsigned int col, unsigned int row_dst,
                unsigned char val, unsigned int num) {

    unsigned int cnt, i;
    
    cnt = num;
    i = 0;
    
    while(cnt--) {
        frame->pixels[row_dst + i][col] = val;
        i++;
    }
                
}

//Like memmove but for bytes spaced width apart instead of sequential
// static void vertmove(unsigned char *dst, unsigned char *src, 
                    // unsigned int num, unsigned int width) {

    // int i, shift, step;
    // unsigned int cnt;
    
    // shift = dst - src; // Find direction of movement to check for overlap	
    
    // if(shift == 0 || num == 0) {
        // return; 
    // } else if(shift > 0) { // Start from tail
        // step = -width;
        // i = width*(num - 1);
    // } else { // shift < 0, start from head
        // step = width;
        // i = 0;
    // }
    
    // cnt = num;
    // while(cnt--) {
        // dst[i] = src[i];
        // i = i + step;
    // }
    
// }

// static void vertset(unsigned char *dst, unsigned char val, 
                    // unsigned int num, unsigned int width) {

    // unsigned int cnt, i;
    
    // cnt = num;
    // i = 0;
    
    // while(cnt--) {
    
        // dst[i] = val;
        // i = i + width;
        
    // }
                    
// }


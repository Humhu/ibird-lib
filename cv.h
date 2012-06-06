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
*  Humphrey Hu      2011-07-01    Initial implementation
*  Humphrey Hu      2012-02-16      Complete rewrite to use camera driver
*/

#ifndef __CV_H
#define __CV_H

#include "cam.h"

// Frame calculation result storage class
typedef struct {
    // State info
    unsigned int frameNum;      // Corresponding frame number
    unsigned char valid;        // Valid bit
    unsigned long timestamp;    // Time of processing
    // Processing results
    unsigned int offset[2];     // Location of center in camera frame
    unsigned int centroid[2];   // Centroid location
    unsigned int max[2];        // Max pixel location
    // Mass properties
    unsigned long mass;         // Sum of luminosity over frame
    unsigned char max_lum;      // Brightest pixel luminosity
    unsigned char avg_lum;      // Average luminosity
} FrameInfoStruct;

typedef FrameInfoStruct* FrameInfo;

/**
 * Set up the CV module
 */
void cvSetup(void);

/**
 * Set a frame to be subtracted from all processed frames
 *
 * @param frame - CamFrame to subtract from processed frames
 * @return Previously used CamFrame
 */
CamFrame cvSetBackgroundFrame(CamFrame frame);

/**
 * Process a frame and record properties into an info struct.
 *
 * @param frame - CamFrame to process
 * @param info - Pointer to FrameInfoStruct to populate with frame's properties
 */
void cvProcessFrame(CamFrame frame, FrameInfo info);

#endif

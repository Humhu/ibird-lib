/*
* Copyright (c) 2012, Regents of the University of California
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
* Ping-Pong Buffer
*
* by Humphrey Hu
*
* v.0.1
*
* Revisions:
*  Humphrey Hu     2012-07-21      Initial release
*/

#include "ppbuff.h"
#include "utils.h"
#include <stdlib.h>

#define FLIP(x)            ((x) ^ 0x01)

// =========== Public Methods =================================================
void ppbuffInit(PingPongBuffer *ppbuff) {

    if(ppbuff == NULL) { return; }
    ppbuff->active_index = 0;
    ppbuff->items[0] = NULL;
    ppbuff->items[1] = NULL;
    ppbuff->is_valid = 1;
    
}

void ppbuffFlip(PingPongBuffer *ppbuff) {

    if(ppbuff == NULL) { return; }
    
    CRITICAL_SECTION_START
    ppbuff->active_index = FLIP(ppbuff->active_index);
    CRITICAL_SECTION_END

}

PingPongItem ppbuffWriteActive(PingPongBuffer *ppbuff, PingPongItem item) {

    PingPongItem old;

    if(ppbuff == NULL) { return NULL; }
    
    CRITICAL_SECTION_START
    old = ppbuff->items[ppbuff->active_index];
    ppbuff->items[ppbuff->active_index] = item;
    CRITICAL_SECTION_END    
    
    return old;

}

PingPongItem ppbuffWriteInactive(PingPongBuffer *ppbuff, PingPongItem item) {

    PingPongItem old;

    if(ppbuff == NULL) { return NULL; }
    
    CRITICAL_SECTION_START
    old = ppbuff->items[FLIP(ppbuff->active_index)];
    ppbuff->items[FLIP(ppbuff->active_index)] = item;
    CRITICAL_SECTION_END    
    
    return old;
    
}

PingPongItem ppbuffReadActive(PingPongBuffer *ppbuff) {

    PingPongItem old;

    if(ppbuff == NULL) { return NULL; }

    CRITICAL_SECTION_START
    old = ppbuff->items[ppbuff->active_index];    
    CRITICAL_SECTION_END    
    
    return old;       
    
}

PingPongItem ppbuffReadInactive(PingPongBuffer *ppbuff) {

    PingPongItem old;

    if(ppbuff == NULL) { return NULL; }

    CRITICAL_SECTION_START
    old = ppbuff->items[FLIP(ppbuff->active_index)];    
    CRITICAL_SECTION_END    
    
    return old;       
    
}


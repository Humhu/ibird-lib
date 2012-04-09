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
 * Network Directory
 *
 * by Humphrey Hu
 *
 * v.beta
 *
 * Revisions:
 *  Humphrey Hu         2011-09-03    Initial implementation
 *                      
 * Notes:
 *
 * To do:
 *     Implement entry number counting
 */
// ==== REFERENCES ==============================================
#include "directory.h"
#include "linear_array.h"
#include <stdlib.h>

// ==== Static Variables =======================================================
LinArray dir;

DirEntry base;

// ==== FUNCTION STUBS ==========================================
DirEntry dirCreateEntry(void);
void dirDeleteEntry(DirEntry entry);
unsigned int entryCompare(void* p1, void* p2);

// ==== FUNCTION BODIES =========================================
void dirInit(unsigned int size) {

    dir = larrayCreate(size);    

 }



DirEntry dirQuery(DirEntryTest comp, void *args) {

    unsigned int i, num;
    DirEntry entry;

    num = larrayFindFirst(dir, (LinArrayItemTest)comp, args,
                &i, (LinArrayItem*)&entry);

    if(num == 0) { return NULL; }
    return entry;

}

DirEntry dirAddNew(void) {

    DirEntry entry;
    unsigned int i;

    // Allocate new entry
    entry = dirCreateEntry();
    if(entry == NULL) { return NULL; }

    // Find space
    if(larrayFindEmpty(dir, &i) == 0) { 
        dirDeleteEntry(entry);
        return NULL;
    }

    larrayReplace(dir, i, entry);
    return entry;

}

// ==== Private Functions ======================================================

unsigned int entryCompare(void* p1, void* p2) {

    DirEntry entry1, entry2;

    entry1 = (DirEntry) p1;
    entry2 = (DirEntry) p2;

    if(entry1 == NULL || entry2 == NULL) {
        return 1;
    }

    return entry1->address == entry2->address;

}

DirEntry dirCreateEntry(void) {

    DirEntry entry;

    entry = (DirEntry) malloc(sizeof(DirEntryStruct));
    if(entry == NULL) { return NULL; }

    entry->uuid = 0;
    entry->address = 0;
    entry->pan_id = 0;

    return entry;

}

void dirDeleteEntry(DirEntry entry) {

    if(entry == NULL) { return; }
    free(entry);
    
}

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
 */

#include "directory.h"
#include "larray.h"
#include <stdlib.h>

// ==== Static Variables =======================================================
static unsigned char is_ready = 0;
static LinArray dir;

// ==== Function Stubs =========================================================
DirEntry dirCreateEntry(void);
void dirDeleteEntry(DirEntry entry);
unsigned int searchAddress(DirEntry entry, void* args);
unsigned int searchID(DirEntry entry, void* args);
unsigned int searchValid(DirEntry entry, void* args);

// ==== Function Bodies ========================================================
void dirInit(unsigned int size) {

    dir = larrayCreate(size);    
    is_ready = 1;
    
 }

unsigned int dirQuery(DirEntryTest comp, void *args, DirEntry *entry) {

    unsigned int i;    

    return larrayFindFirst(dir, (LinArrayItemTest)comp, args,
                &i, (LinArrayItem*) entry);

}

unsigned int dirQueryN(DirEntryTest comp, void *args, DirEntry *entries,
                        unsigned int N) {

    unsigned int i[N];

    return larrayFindN(dir, (LinArrayItemTest)comp, args, i,
                            (LinArrayItem*) entries, N);

}

DirEntry dirQueryAddress(unsigned int addr, unsigned int pan) {

    unsigned int args[2];
    DirEntry entry;

    args[0] = addr;
    args[1] = pan;

    // If search successful, return item
    if(dirQuery(&searchAddress, args, &entry)) {
        return entry;
    }
    return NULL;

}

DirEntry dirQueryID(unsigned long long id) {

    DirEntry entry;

    // If search successful, return item
    if(dirQuery(&searchID, &id, &entry)) {
        return entry;
    }
    return NULL;

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

unsigned int dirGetSize(void) {

    return larrayGetSize(dir);

}

unsigned int dirGetEntries(DirEntry *entries) {
    
    return dirQueryN(&searchValid, NULL, entries, larrayGetMaxSize(dir));

}

// ==== Private Functions ======================================================

unsigned int searchAddress(DirEntry entry, void *args) {

    if(entry == NULL) { return 0; }
    return entry->address == ((unsigned int*)args)[0] &&
            entry->pan_id == ((unsigned int*)args)[1];

}

unsigned int searchID(DirEntry entry, void *args) {

    if(entry == NULL) { return 0; }
    return entry->uuid == *((unsigned long long *)args);

}

unsigned int searchValid(DirEntry entry, void *args) {

    return entry != NULL;

}

DirEntry dirCreateEntry(void) {

    DirEntry entry;

    entry = (DirEntry) calloc(sizeof(DirEntryStruct), 1);
    if(entry == NULL) { return NULL; }

    return entry;

}

void dirDeleteEntry(DirEntry entry) {

    if(entry == NULL) { return; }
    free(entry);
    
}

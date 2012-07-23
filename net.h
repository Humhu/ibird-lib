/*
 * Copyright (c) 2011, Regents of the University of California
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
 * Network addressing module
 *
 * by Humphrey Hu
 *
 * v.beta
 *
 * Revisions:
 *  Humphrey Hu		2011-06-28		Initial implementation
 *  Humphrey Hu		2011-09-03		Moved directory to separate module
 *
 * Notes:
 *
 */

#ifndef __NETWORK_H
#define __NETWORK_H

#include "mac_packet.h"

#define NETWORK_BASESTATION_CHANNEL			(0x15)
#define NETWORK_BASESTATION_PAN_ID			(0x1001)
#define NETWORK_BASESTATION_ADDR			(0x1020)

#define NETWORK_BOOTLOAD_SERVER_ADDR		(0x1020)
#define NETWORK_BOOTLOAD_SERVER_PAN			(0x1000)
#define NETWORK_BOOTLOAD_CLIENT_ADDR		(0x1101)
#define NETWORK_BOOTLOAD_CLIENT_PAN			(0x1000)
#define NETWORK_BOOTLOAD_CHANNEL			(0x15)
 
#define NETWORK_DISCOVERY_SERVER_ADDR		(0x1020)
#define NETWORK_DISCOVERY_SERVER_PAN		(0x1001)
#define NETWORK_DISCOVERY_CLIENT_ADDR		(0x1101) 
#define NETWORK_DISCOVERY_CLIENT_PAN		(0x1001)
#define NETWORK_DISCOVERY_CHANNEL			(0x15)

#define NETWORK_BROADCAST_ADDR  			(0xFFFF)

void netSetup(unsigned int dir_size);
unsigned int netGetLocalAddress(void);
unsigned int netGetLocalPanID(void);
unsigned int netGetBaseAddress(void);
unsigned int netGetBasePanID(void);
void netRequestAddress(void);
unsigned char netAddressReceived(void);

void netHandleOffer(MacPacket packet);
void netHandleRequest(MacPacket packet);
void netHandleAccept(MacPacket packet);

#endif // __NETWORK_H

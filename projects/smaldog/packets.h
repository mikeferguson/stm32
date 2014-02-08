/*
 * Copyright (c) 2013, Michael E. Ferguson
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef PACKETS_H_
#define PACKETS_H_

/* This is the maximum size of a packet -- note that it is slightly smaller than
   the maximum dynamixel packet size (143) */
#define P_BUFFER_SIZE       128

/* Device IDs */
#define DEVICE_AX           100
#define DEVICE_XBEE         252
#define DEVICE_CORE         253
// NOTE: Ethernet connection uses port # as device, port # must be > 255

/* State */
#define STATE_READY         0
#define STATE_BUSY          1

/*
 * Structure to hold packet data
 */
typedef struct
{
  uint16_t id;                      /* packet id, if needed */
  uint16_t destination;             /* device this packet is destined for, if known */
  uint16_t source;                  /* device this packet originated from, if known */
  uint8_t payload[P_BUFFER_SIZE];   /* packet data -- note: this is a dynamixel packet */
  uint8_t payload_length;           /* length packet data */
} packet_t;

/*
 * Initialize the packet router.
 */
void router_init();

/*
 * Post a packet to the router for dispatch.
 */
void router_dispatch(packet_t& p);

/*
 * Called in the main loop.
 */
void router_process();

#endif // end PACKETS_H_

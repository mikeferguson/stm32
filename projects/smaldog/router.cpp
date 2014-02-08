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

#include "eth_bridge.hpp"

#define P_BUFFER_COUNT      16
#define DEVICE_COUNT        3

#define INDEX_ETH   0
#define INDEX_AX    1
#define INDEX_XBEE  2
#define INDEX_CORE  3

typedef struct
{
  packet_t buffer[P_BUFFER_COUNT];
  uint8_t  head;
  uint8_t  tail;
} device_buffer_t;

device_buffer_t devices[DEVICE_COUNT];

uint32_t packet_counts[4];

void router_init()
{
  for(int i = 0; i < DEVICE_COUNT; i++)
  {
    devices[i].head = devices[i].tail = 0;
    packet_counts[i] = 0;
  }
  packet_counts[DEVICE_COUNT] = 0; // core has no buffer
}

void router_dispatch(packet_t& p)
{
  
  if(p.destination == 0)
  {
    /* destination is not already known */
    uint8_t id = p.payload[2];
    if(id == DEVICE_CORE)
      p.destination = DEVICE_CORE;
    else if(id == DEVICE_XBEE)
      p.destination = DEVICE_XBEE;
    else
      p.destination = DEVICE_AX; // TODO: how to decide if on RX bus?
  }
  
  /* sort out device index */
  uint8_t d_index = INDEX_ETH;
  if(p.destination == DEVICE_CORE)
  {
    /* the core uses no buffer, as it never has to wait for a device */
    core_dispatch(p);
    packet_counts[INDEX_CORE]++;
    return;
  }
  else if(p.destination == DEVICE_XBEE)
    d_index = INDEX_XBEE;
  else if(p.destination == DEVICE_AX)
    d_index = INDEX_AX;

  /* post to buffer */
  device_buffer_t* d = &devices[d_index];
  if((d->head+1)%P_BUFFER_COUNT == d->tail)
  {
    // TODO: device overflow?
  }
  else
  {
    d->buffer[d->head].destination = p.destination;
    d->buffer[d->head].source = p.source;
    d->buffer[d->head].id = p.id;
    d->buffer[d->head].payload_length = p.payload_length;
    for(int i = 0; i < p.payload_length; i++)
      d->buffer[d->head].payload[i] = p.payload[i];
    d->head = (d->head+1)%P_BUFFER_COUNT;
  }
}

void router_process()
{
  /* ethernet */
  if(dev_eth_getState() == STATE_READY)
  {
    if(devices[INDEX_ETH].head != devices[INDEX_ETH].tail)
    {
      dev_eth_dispatch(devices[INDEX_ETH].buffer[devices[INDEX_ETH].tail]);
      devices[INDEX_ETH].tail = (devices[INDEX_ETH].tail+1)%P_BUFFER_COUNT;
      packet_counts[INDEX_ETH]++;
    }
  }

  /* ax bus */
  if(dev_ax_getState() == STATE_READY)
  {
    if(devices[INDEX_AX].head != devices[INDEX_AX].tail)
    {
      dev_ax_dispatch(devices[INDEX_AX].buffer[devices[INDEX_AX].tail]);
      devices[INDEX_AX].tail = (devices[INDEX_AX].tail+1)%P_BUFFER_COUNT;
      packet_counts[INDEX_AX]++;
    }
  }

  // TODO: xbee packets

  /* NOTE: core is processed when dispatched */
}


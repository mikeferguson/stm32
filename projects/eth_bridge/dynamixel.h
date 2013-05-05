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

#ifndef DYNAMIXEL_H_
#define DYNAMIXEL_H_

#define DYNAMIXEL_BUFFER_SIZE 128
#define DYNAMIXEL_BUFFER_COUNT 16

#define DESTINATION_DEVICE_MASK     0xff00
#define DESTINATION_ETH             0x0100
#define DESTINATION_XBEE            0x0200

/*
 * Simple struct to hold packet data
 */
typedef struct
{
  uint8_t data[DYNAMIXEL_BUFFER_SIZE];  /* packet data */
  uint8_t data_length;                  /* length of packet data */
  uint8_t data_head;                    /* how far into the packet we have consumed */
  uint16_t destination;                 /* where return data should eventually go*/
  uint16_t port;                        /* port number, if over udp */
} dynamixel_packet_t;

/*
 * Possible states for bus arbitration
 */
enum
{
  BUS_IDLE,
  BUS_WRITING,
  BUS_READING_FF,
  BUS_READING_2ND_FF,
  BUS_READING_ID,
  BUS_READING_LENGTH,
  BUS_READING_PARAMS,
  BUS_PACKET_READY
};

/*
 * Simple structure to encompass the workings of a dynamixel bus
 */
typedef struct
{
  /* state */
  uint8_t state;
  uint32_t timeout;

  /* diagnostics */
  uint32_t p_sent;      /* # of packets successfully sent over dynamixel bus */
  uint32_t p_recv;      /* # of packets successfully recieved from dynamixel bus */
  uint32_t p_dropped;   /* # of packets that never entered queue because it was full */

  /* TODO: needed functions if making this generic? */

  /* buffers */
  dynamixel_packet_t rx;
  dynamixel_packet_t tx[DYNAMIXEL_BUFFER_COUNT];
  uint8_t tx_head;
  uint8_t tx_tail;
} dynamixel_bus_t;

void init_dynamixel();

/*
 * Copies available packet, if any, into the packet passed.
 * Returns number of data bytes copied.
 */
uint8_t recv_packet(/*dynamixel_bus_t& bus,*/ dynamixel_packet_t& packet);
/*
 * Copies packet into queue, if space exists.
 * Returns number of data bytes copied.
 */
uint8_t send_packet(/*dynamixel_bus_t& bus,*/ dynamixel_packet_t& packet);

#endif

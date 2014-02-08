/*
 * Copyright (c) 2012-2013, Michael E. Ferguson
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
#include "lwip/memp.h"
#include "lwip/tcp.h"
#include "lwip/udp.h"
#include <stdint.h>
#include <string.h>
#include <delay.hpp>

static err_t tcp_accept_callback(void *arg, struct tcp_pcb *pcb, err_t err);
static err_t tcp_recv_callback(void *arg, struct tcp_pcb *pcb, struct pbuf *p, err_t err);
static void tcp_conn_err_callback(void *arg, err_t err);

static void udp_recv_callback(void *arg, struct udp_pcb *udp, struct pbuf *p, struct ip_addr *addr, u16_t port);

struct udp_pcb *eth_udp = NULL;
struct ip_addr return_ipaddr;

int16_t commands[14];
// Initialize stuff needed for network application :
//   *  a udp connection listening on port 6707
int netapp_init()
{ 
  // UDP connection
  eth_udp = udp_new();
  if (eth_udp == NULL)
  {
    return 0;
  }

  if (udp_bind(eth_udp, IP_ADDR_ANY, 6707) != ERR_OK)
  {
    return 0;
  }

  udp_recv(eth_udp, udp_recv_callback, NULL);

  for (int i = 0; i < 14; ++i)
    commands[i] = -1;

  return 1;
}

static void udp_send_packet(uint8_t * packet, uint8_t len, u16_t port)
{
  struct pbuf * p_send = pbuf_alloc(PBUF_TRANSPORT, ETH_MAGIC_LENGTH + len, PBUF_RAM);
  unsigned char * x = (unsigned char *) p_send->payload;

  /* ethernet header */
  *x++ = 'S'; *x++ = 'M'; *x++ = 'A'; *x++ = 'L';

  /* payload */
  for(int i = 0; i < len; ++i)
    *x++ = packet[i];

  /* send it */
  udp_sendto(eth_udp, p_send, &return_ipaddr, port);
  pbuf_free(p_send);
}

static void udp_recv_callback(void *arg, struct udp_pcb *udp, struct pbuf *p, struct ip_addr *ipaddr, u16_t port)
{
  /* Toss any packets with deformed header. */
  uint8_t* data = (uint8_t*) p->payload;
  if( (p->len < ETH_MAGIC_LENGTH) ||
      (p->len != p->tot_len) ||
      (data[0] != 'S') || (data[1] != 'M') || (data[2] != 'A') || (data[3] != 'L') )
  {
    pbuf_free(p);
    return;
  }

  /* Update return ip */
  return_ipaddr = *ipaddr;
  register_table.last_packet = register_table.system_time;

  /* Split out individual dynamixel packets */
  uint16_t offset = ETH_MAGIC_LENGTH;
  while(offset < p->len)
  {
    /* Check header is 0xff 0xff */
    if ((data[offset] != 0xff) || (data[offset+1] != 0xff))
      break;

    /* Check ID is valid */
    if (data[offset+2] == 0xff)
      break;

    /* Check length */
    int len = data[offset+3];
    if (len + 4 + offset > p->len)
      break;

    if (data[offset+2] == 253)  /* ID = this board */
    {
      if (data[offset+4] == AX_FULL_SYNC)
      {
        /* Create sync_write packet */
        uint8_t packet[256];
        packet[0] = 0xff;
        packet[1] = 0xff;
        packet[2] = 254;
        packet[4] = AX_SYNC_WRITE;
        packet[5] = 0x1E; // goal position
        packet[6] = 2; // write 2 bytes
        int length = 3;
        for (int i = 0; i < 12; ++i)
        {
          int16_t v = data[offset+5+(i*2)] + (data[offset+6+(i*2)]<<8);
          if (v > 0)
          {
            commands[i] = v;
            packet[4+length++] = i+1;
            packet[4+length++] = data[offset+5+(i*2)];
            packet[4+length++] = data[offset+6+(i*2)];
            //packet[4+length++] = 0x80;
            //packet[4+length++] = 0x00;  // moving speed
          }
        }
        packet[3] = ++length;
        packet[4+length-1] = 0; // checksum?
        for (int i = 2; i < length + 3; ++i)
          packet[4+length-1] += packet[i];
        packet[4+length-1] = 255 - packet[4+length-1];
        if (length > 4)
        {
          dynamixel_write(packet, length + 4);
          delay_us(15);  // TODO: figure out why we need this and if we can get rid of it
        }

        /* Update board state */
        uint8_t ret_packet[256];
        for (int i = 0; i < 12; ++i)
        {
          /* Read servo i+1 */
          packet[2] = i + 1;
          packet[3] = 4;
          packet[4] = AX_READ_DATA;
          packet[5] = 0x24;  // PRESENT_POSITION_L;
          packet[6] = 2;  // 2 bytes;
          packet[7] = 255 - i - 1 - 4 - AX_READ_DATA - 0x24 - 2;
          uint8_t len = dynamixel_read(packet, 8, ret_packet);
          if ((len == 8) && (ret_packet[2] == i + 1))
          {
            reinterpret_cast<uint8_t*>(&register_table)[48 + (i*2)] = ret_packet[5];
            reinterpret_cast<uint8_t*>(&register_table)[49 + (i*2)] = ret_packet[6];
          }
          else
          {
            reinterpret_cast<int16_t*>(&register_table)[24 + i] = -1;
          }
          delay_us(15);  // TODO: figure out why we need this and if we can get rid of it
        }

        // TODO: read foot sensors
        // TODO: read IMU

        /* Send return packet */
        udp_send_packet(reinterpret_cast<uint8_t*>(&register_table), 80, port);
      }
      else if (data[offset+4] == AX_SYNC_READ)
      {
        // TODO: do sync read
      }
      else if (data[offset+4] == AX_READ_DATA)
      {
        /* Return data from register table */
        uint8_t packet[256];
        packet[0] = 0xff;
        packet[1] = 0xff;
        packet[2] = 253;
        packet[3] = data[offset+6];
        packet[4] = 0; // TODO: real error code?
        for (int i = 0; i < data[offset+6]; ++i)
          packet[5+i] = reinterpret_cast<uint8_t*>(&register_table)[data[offset+5]+i];
        packet[6+data[offset+6]] = 0; // TODO: add real checksum
        udp_send_packet(packet, data[offset+6] + 6, port);
      }
    }
    else  /* Pass through */
    {
      if (data[offset+4] == AX_READ_DATA)
      {
        uint8_t packet[256];
        uint8_t packet_len = dynamixel_read(&data[offset], len + 4, packet);
        if (packet_len > 0)
          udp_send_packet(packet, packet_len, port);
      }
      else
      {
        /* Just write */
        dynamixel_write(&data[offset], len + 4);
      }
    }

    offset += 4 + len;
  }

  pbuf_free(p);
}

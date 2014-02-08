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
#include "packets.h"
#include "lwip/memp.h"
#include "lwip/tcp.h"
#include "lwip/udp.h"
#include <stdint.h>
#include <string.h>

static err_t tcp_accept_callback(void *arg, struct tcp_pcb *pcb, err_t err);
static err_t tcp_recv_callback(void *arg, struct tcp_pcb *pcb, struct pbuf *p, err_t err);
static void tcp_conn_err_callback(void *arg, err_t err);

static void udp_recv_callback(void *arg, struct udp_pcb *udp, struct pbuf *p, struct ip_addr *addr, u16_t port);

struct udp_pcb *eth_udp = NULL;
struct ip_addr return_ipaddr;

// Initialize stuff needed for network application :
//   *  a udp connection listening on port 5048
int netapp_init()
{ 
  // UDP connection
  eth_udp = udp_new();
  if (eth_udp == NULL)
  {
  return 0;
  }

  if (udp_bind(eth_udp, IP_ADDR_ANY, 5048) != ERR_OK)
  {
  return 0;
  }

  udp_recv(eth_udp, udp_recv_callback, NULL);

  return 1;
}

static void udp_recv_callback(void *arg, struct udp_pcb *udp, struct pbuf *p, struct ip_addr *ipaddr, u16_t port)
{
  /* Toss any packets with deformed header. */
  uint8_t* data = (uint8_t*) p->payload;
  if( (p->len < ETH_MAGIC_LENGTH) ||
      (p->len != p->tot_len) ||
      (data[0] != 0xff) || (data[1] != 'B') || (data[2] != 'O') || (data[3] != 'T') )
  {
    pbuf_free(p);
    return;
  }

  /* Update return ip */
  return_ipaddr = *ipaddr;
  last_packet = sys_time;

  /* Split out individual dynamixel packets */
  uint16_t offset = ETH_MAGIC_LENGTH;
  while(offset < p->len)
  {
    packet_t pkt;

    /* handle destination/port */
    pkt.destination = 0;
    pkt.source = port;
    if( (data[offset] != 0xff) || (data[offset+2] == 0xff) )
      pkt.id = data[offset++];

    /* copy data */
    pkt.payload_length = data[offset+3]+4;
    for(int i = 0; i < pkt.payload_length; i++)
      pkt.payload[i] = data[offset+i];

    router_dispatch(pkt);
    offset += pkt.payload_length;
  }

  pbuf_free(p);
}

uint8_t dev_eth_getState()
{
  /* really no reason block... */
  return STATE_READY;
}

void dev_eth_dispatch(packet_t& p)
{
  struct pbuf * p_send = pbuf_alloc(PBUF_TRANSPORT, ETH_MAGIC_LENGTH + 1 + p.payload_length, PBUF_RAM);
  unsigned char * x = (unsigned char *) p_send->payload;

  /* ethernet header */
  *x++ = 0xff; *x++ = 'B'; *x++ = 'O'; *x++ = 'T';

  /* packet id */
  *x++ = p.id;

  /* payload */
  for(int i = 0; i < p.payload_length; i++)
    *x++ = p.payload[i];

  /* send it */
  udp_sendto(eth_udp, p_send, &return_ipaddr, p.destination);
  pbuf_free(p_send);
}

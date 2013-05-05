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
#include "dynamixel.h"
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

int reads;
    dynamixel_packet_t pkt;
static void udp_recv_callback(void *arg, struct udp_pcb *udp, struct pbuf *p, struct ip_addr *ipaddr, u16_t port)
{
  /* Toss any packets with deformed header. */
  uint8_t* data = (uint8_t*) p->payload;
  if( (p->len < ETH_MAGIC_LENGTH) ||
      (p->len != p->tot_len) ||
      (data[0] != 0xff) || (data[1] != 'E') || (data[2] != 'T') || (data[3] != 'H') )
  {
    pbuf_free(p);
    return;
  }

  /* Update return ip */
  return_ipaddr = *ipaddr;

  /* Split out individual dynamixel packets */
  uint16_t offset = ETH_MAGIC_LENGTH;
  while(offset < p->len)
  {

    /* handle destination/port */
    pkt.destination = DESTINATION_ETH;
    pkt.port = port;
    if( (data[offset] != 0xff) || (data[offset+2] == 0xff) )
      pkt.destination += data[offset++];

    /* copy data */
    pkt.data_length = data[offset+3]+4;
    for(int i = 0; i < pkt.data_length; i++)
      pkt.data[i] = data[offset+i];

    /* dispatch */
    if(pkt.data[2] == 253)
    {
      // TODO: this will also be used by XBEE -- should probably be moved elsewhere eventually
      if(pkt.data[4] == AX_READ_DATA)
      {
        int addr = pkt.data[5];
        int len = pkt.data[6];

        struct pbuf * p_send = pbuf_alloc(PBUF_TRANSPORT, ETH_MAGIC_LENGTH + 7 + len, PBUF_RAM);
        unsigned char * x = (unsigned char *) p_send->payload;
        /* ethernet header */
        *x++ = 0xff; *x++ = 'E'; *x++ = 'T'; *x++ = 'H';
        /* packet id */
        *x++ = pkt.destination & 0xff;
        /* dynamixel header */
        *x++ = 0xff;*x++ = 0xff;*x++=253;*x++=2+len;*x++=0 /*error*/;
        /* params */
        int checksum = 253 + 2 + len;
        for(int i = 0; i<pkt.data[5]; i++)
        {
          if(addr == REG_MODEL_NUMBER_L){
            *x++ = 45;
            checksum += 45;
          }else if(addr == REG_MODEL_NUMBER_H){
            *x++ = 1;  // 301
            checksum += 1;
          }else if(addr == REG_VERSION){
            *x++ = 0;
          }else if(addr == REG_ID){
            *x++ = 253;
            checksum += 253;
          }else if(addr == REG_BAUD_RATE){
            *x++ = 34; // 57600????
            checksum += 34;
          //else if(addr == REG_RETURN_DELAY)
          //else if(addr == REG_RETURN_LEVEL)
          //  *x++ = return_level;
          //else if(addr == REG_ALARM_LED)
          //else if(addr == REG_LED)
          }else if(addr == REG_PRESENT_VOLTAGE){
            *x = (uint8_t) (sys_voltage*10.0f);
            checksum += *x; x++;
          }else if(addr == REG_CURRENT_L){
            *x = (uint8_t) (((int)(sys_current/0.0045f)+2048) & 0xff);
            checksum += *x; x++;
          }else if(addr == REG_CURRENT_H){
            *x = (uint8_t) (((int)(sys_current/0.0045f)+2048) >> 8);
            checksum += *x; x++;
          }else{
            *x++ = 0;
          }
          addr++;
        }
        /* checksum */
        checksum = 255 - (checksum & 0xff);
        *x++ = checksum;
        /* send it */
        udp_sendto(eth_udp, p_send, ipaddr, port);
        pbuf_free(p_send);
      }
      else if(pkt.data[3] == AX_WRITE_DATA)
      {

      }
    }
    else
    {
      // TODO: implement sync_read
      send_packet(pkt);
    }
    offset += pkt.data_length;
  }

  pbuf_free(p);
}



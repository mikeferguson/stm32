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
#include "usart.hpp"
#include "dynamixel.h"

dynamixel_bus_t bus;
UsartWithEnable<USART3_BASE, 64, usart3_en> usart3;

void init_dynamixel()
{
  /* initialize bus data structure */
  bus.state = BUS_IDLE;
  bus.p_sent = bus.p_recv = bus.p_dropped = 0;
  bus.tx_head = bus.tx_tail = 0;

  /* setup usarts */
  RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
  usart3_tx::mode(GPIO_ALTERNATE | GPIO_AF_USART3);
  usart3_rx::mode(GPIO_ALTERNATE | GPIO_AF_USART3);
  usart3_en::mode(GPIO_OUTPUT);
  usart3.init(1000000);
  NVIC_EnableIRQ(USART3_IRQn);
}

extern "C"
{
  void USART3_IRQHandler(void)
  {
    usart3.irq();
  }
}

uint8_t recv_packet(dynamixel_packet_t& packet)
{
  if(bus.state == BUS_IDLE) return 0; // TODO ??

  /* process any data in the IRQ buffer */
  int16_t b = usart3.read();
  while(b != -1)
  {
    if(bus.state == BUS_READING_FF)
    {
      if(b == 0xff)
      {
        bus.rx.data_head = 0;
        bus.rx.data[bus.rx.data_head++] = b;
        bus.state = BUS_READING_2ND_FF;
      }
    }
    else if(bus.state == BUS_READING_2ND_FF)
    {
      if(b == 0xff)
      {
        bus.rx.data[bus.rx.data_head++] = b;
        bus.state = BUS_READING_ID;
      }
      else
      {
        bus.state = BUS_READING_FF;
      }
    }
    else if(bus.state == BUS_READING_ID)
    {
      if(b != 0xff)
      {
        bus.rx.data[bus.rx.data_head++] = b;
        bus.state = BUS_READING_LENGTH;
      }
    }
    else if(bus.state == BUS_READING_LENGTH)
    {
      if(b < 140) // TODO: calculate actual threshold for this
      {
        bus.rx.data[bus.rx.data_head++] = b;
        bus.state = BUS_READING_PARAMS;
        bus.rx.data_length = bus.rx.data[3] + 4;
      }
      else
      {
        bus.state = BUS_READING_FF;
      }
    }
    else if(bus.state == BUS_READING_PARAMS)
    {
      bus.rx.data[bus.rx.data_head++] = b;
      if(bus.rx.data_head == bus.rx.data_length)
      {
        /* checksum check */
        int checksum = 0;
        for(int i=2; i < bus.rx.data_length; i++)
          checksum += bus.rx.data[i];
        if((checksum & 0xff) == 0xff)
        {
          bus.p_recv++;
          bus.state = BUS_PACKET_READY;
          break;
        }
        else
        {
          // TODO: should this jump straight to writing again?
          bus.state = BUS_READING_FF;
        }
      }
    }
    b = usart3.read();
  }

  /* now, return if we have a packet */
  if(bus.state == BUS_PACKET_READY)
  {
    /* setup packet to return */
    packet.data_length = bus.rx.data_length;
    for(int i=0; i < packet.data_length; i++)
        packet.data[i] = bus.rx.data[i];
    packet.destination = bus.rx.destination;
    packet.port = bus.rx.port;
    bus.state = BUS_WRITING;
  }
  else
  {
    packet.data_length = 0;
  }
  
  /* kick off a new packet if any */
  if((bus.state == BUS_WRITING) || (sys_time > bus.timeout))
  {
    if(bus.tx_head != bus.tx_tail)
    {
      /* send next packet */
      usart3.setTX();
      for(int i = 0; i < bus.tx[bus.tx_tail].data_length; i++)
        usart3.write(bus.tx[bus.tx_tail].data[i]);
      usart3.setRX();
      bus.rx.destination = bus.tx[bus.tx_tail].destination;
      bus.rx.port = bus.tx[bus.tx_tail].port;
      /* update structs */
      if(bus.tx[bus.tx_tail].data[2] == 254)
        bus.timeout = 1; // tiny hack for sync write
      else
        bus.timeout = sys_time + 10;
      bus.tx_tail = (bus.tx_tail + 1) % DYNAMIXEL_BUFFER_COUNT;
      bus.state = BUS_READING_FF;
      bus.p_sent++;
    }
    else
    {
      bus.state = BUS_IDLE;
    }
  }

  return packet.data_length;
}

uint8_t send_packet(dynamixel_packet_t& packet)
{
  if((bus.tx_head+1)%DYNAMIXEL_BUFFER_COUNT == bus.tx_tail)
  {
    /* overflowed packet depth */
    bus.p_dropped++;
  }
  else
  {
    if(bus.state == BUS_IDLE)
    {
      /* skip the queue */
      usart3.setTX();
      for(int i = 0; i < packet.data_length; i++)
        usart3.write(packet.data[i]);
      usart3.setRX();
      bus.state = BUS_READING_FF;
      bus.timeout = sys_time + 10;
      bus.rx.destination = packet.destination;
      bus.rx.port = packet.port;
      bus.p_sent++;
    }
    else
    {
      bus.tx[bus.tx_head].data_length = packet.data_length; 
      bus.tx[bus.tx_head].destination = packet.destination;
      bus.tx[bus.tx_head].port = packet.port;
      for(int i = 0; i < packet.data_length; i++)
        bus.tx[bus.tx_head].data[i] = packet.data[i];
      bus.tx_head = (bus.tx_head+1)%DYNAMIXEL_BUFFER_COUNT;
    }
  }
}

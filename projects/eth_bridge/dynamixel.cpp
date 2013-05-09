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
  BUS_READING_PARAMS
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
  uint32_t p_timedout;  /* # of packets that timed out before a recieve */

  /* return data */
  packet_t p;           /* return packet */
  uint8_t head;         /* head of return packet */
} dynamixel_bus_t;

dynamixel_bus_t bus;
UsartWithEnable<USART3_BASE, 64, usart3_en> usart3;

void dynamixel_init()
{
  /* initialize bus data structure */
  bus.state = BUS_IDLE;
  bus.p_sent = bus.p_recv = bus.p_timedout = 0;

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

uint8_t dev_ax_getState()
{
  if(bus.state == BUS_IDLE) return STATE_READY;

  /* process any data in the IRQ buffer */
  int16_t b = usart3.read();
  while(b != -1)
  {
    if(bus.state == BUS_READING_FF)
    {
      if(b == 0xff)
      {
        bus.head = 0;
        bus.p.payload[bus.head++] = b;
        bus.state = BUS_READING_2ND_FF;
      }
    }
    else if(bus.state == BUS_READING_2ND_FF)
    {
      if(b == 0xff)
      {
        bus.p.payload[bus.head++] = b;
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
        bus.p.payload[bus.head++] = b;
        bus.state = BUS_READING_LENGTH;
      }
    }
    else if(bus.state == BUS_READING_LENGTH)
    {
      if(b < 140) // TODO: calculate actual threshold for this
      {
        bus.p.payload[bus.head++] = b;
        bus.state = BUS_READING_PARAMS;
        bus.p.payload_length = bus.p.payload[3] + 4;
      }
      else
      {
        bus.state = BUS_READING_FF;
      }
    }
    else if(bus.state == BUS_READING_PARAMS)
    {
      bus.p.payload[bus.head++] = b;
      if(bus.head == bus.p.payload_length)
      {
        /* checksum check */
        int checksum = 0;
        for(int i=2; i < bus.p.payload_length; i++)
          checksum += bus.p.payload[i];
        if((checksum & 0xff) == 0xff)
        {
          bus.p_recv++;
          router_dispatch(bus.p);
          bus.state = BUS_IDLE;
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

  if(sys_time > bus.timeout)
  {
    bus.p_timedout++;
    bus.state = BUS_IDLE;
  }

  if(bus.state == BUS_IDLE)
    return STATE_READY;
  else
    return STATE_BUSY;
}

void dev_ax_dispatch(packet_t& p)
{
  /* copy required data for return trip */
  bus.p.id = p.id;
  bus.p.destination = p.source;
  bus.p.source = p.destination; // NOTE: already known to be DEVICE_AX

  /* set state */
  bus.state = BUS_READING_FF;
  bus.timeout = sys_time + 10;

  /* send payload */
  usart3.setTX();
  for(int i = 0; i < p.payload_length; i++)
    usart3.write(p.payload[i]);
  usart3.setRX();
  bus.p_sent++;
}

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
#include "delay.hpp"
#include "netconf.h"
#include "netapp.h"
#include "stm32f4x7_eth.h"
#include "dynamixel.h"

uint32_t sys_time;
float sys_voltage;
float sys_current;

int main(void)
{
  sys_time = 0;
  sys_voltage = 12.0f;
  sys_current = 1.0f;
  NVIC_SetPriorityGrouping(3);

  // enable GPIO
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;

  act::mode(GPIO_OUTPUT);
  stat::mode(GPIO_OUTPUT);
  error::mode(GPIO_OUTPUT);
  estop::mode(GPIO_INPUT);

  // setup ethernet
  phy_rst::mode(GPIO_OUTPUT_2MHz);
  phy_rst::high(); // release reset line

  eth_mii_crs::mode(GPIO_ALTERNATE | GPIO_AF_ETH);
  eth_rx_clk::mode(GPIO_ALTERNATE | GPIO_AF_ETH);
  eth_mdio::mode(GPIO_ALTERNATE | GPIO_AF_ETH);
  eth_mii_col::mode(GPIO_ALTERNATE | GPIO_AF_ETH);
  eth_mii_rx_dv::mode(GPIO_ALTERNATE | GPIO_AF_ETH);

  eth_mii_rxd2::mode(GPIO_ALTERNATE | GPIO_AF_ETH);
  eth_mii_rxd3::mode(GPIO_ALTERNATE | GPIO_AF_ETH);
  eth_mii_rx_er::mode(GPIO_ALTERNATE | GPIO_AF_ETH);
  eth_mii_tx_en::mode(GPIO_ALTERNATE | GPIO_AF_ETH);
  eth_mii_txd0::mode(GPIO_ALTERNATE | GPIO_AF_ETH);
  eth_mii_txd1::mode(GPIO_ALTERNATE | GPIO_AF_ETH);

  eth_mdc::mode(GPIO_ALTERNATE | GPIO_AF_ETH);
  eth_mii_txd2::mode(GPIO_ALTERNATE | GPIO_AF_ETH);
  eth_mii_tx_clk::mode(GPIO_ALTERNATE | GPIO_AF_ETH);
  eth_mii_rxd0::mode(GPIO_ALTERNATE | GPIO_AF_ETH);
  eth_mii_rxd1::mode(GPIO_ALTERNATE | GPIO_AF_ETH);

  eth_mii_txd3::mode(GPIO_ALTERNATE | GPIO_AF_ETH);

  init_dynamixel();

  // setup systick
  SysTick_Config(SystemCoreClock/1000);
  NVIC_EnableIRQ(SysTick_IRQn);
  //NVIC_SetPriority(SysTick_IRQn,2);

  Ethernet_Init();
  LwIP_Init();
  if (!netapp_init())
    while(1);

  __enable_irq();

  while(1)
  {
    /* check if any packet received */
    if (ETH_CheckFrameReceived())
    { 
      /* process received ethernet packet */
      LwIP_Pkt_Handle();
    }
    LwIP_Periodic_Handle(sys_time);

    /* process dynamixel */
    dynamixel_packet_t p;
    if(recv_packet(p) > 0)
    {
      if((p.destination & DESTINATION_DEVICE_MASK) == DESTINATION_ETH)
      {
        /* send data as ethernet packet */
        struct pbuf * p_send = pbuf_alloc(PBUF_TRANSPORT, p.data_length + ETH_MAGIC_LENGTH + 1, PBUF_RAM);
        unsigned char * x = (unsigned char *) p_send->payload;
        *x++ = 0xff; *x++ = 'E'; *x++ = 'T'; *x++ = 'H';
        *x++ = p.destination & 0xff;
        for(int i = 0; i < p.data_length; i++)
        {
          *x++ = p.data[i];
        }
        udp_sendto(eth_udp, p_send, &return_ipaddr, p.port);
        pbuf_free(p_send);
      }
      // TODO: xbee
    }
  }
}

extern "C"
{

void SysTick_Handler(void)
{
  sys_time++;

  /* check e-stop */
  if(estop::value() > 0){
    //stat::high();
    // TODO: stop stuffs?
  }else{
    //stat::low();
  }

  /* toggle activity led */
  if(sys_time % 1000 == 0)
  {
    uint8_t i;
    act::high();
  }
  else if (sys_time % 1000 == 500)
  {
    uint8_t i;
    act::low();
  }
}

}

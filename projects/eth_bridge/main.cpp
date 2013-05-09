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

uint8_t sys_estop;
uint32_t sys_time;
uint32_t last_packet;
float sys_voltage;
float sys_current;

int main(void)
{
  sys_time = 0;
  sys_voltage = 12.0f;
  sys_current = 1.0f;
  NVIC_SetPriorityGrouping(3);

  /* enable GPIO */
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;

  act::mode(GPIO_OUTPUT);
  stat::mode(GPIO_OUTPUT);
  error::mode(GPIO_OUTPUT);
  error::high();
  estop::mode(GPIO_INPUT);

  /* setup ethernet */
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

  dynamixel_init();
  router_init();

  /* setup systick */
  SysTick_Config(SystemCoreClock/1000);
  NVIC_EnableIRQ(SysTick_IRQn);
  //NVIC_SetPriority(SysTick_IRQn,2);

  Ethernet_Init();
  LwIP_Init();
  if (!netapp_init())
    while(1);

  __enable_irq();

  /* done with setup, turn off err led */
  error::low();

  while(1)
  {
    /* check if any packet received */
    if (ETH_CheckFrameReceived())
    { 
      /* process received ethernet packet */
      LwIP_Pkt_Handle();
    }
    LwIP_Periodic_Handle(sys_time);

    /* process packets */
    router_process();
  }
}

void core_dispatch(packet_t& p)
{
  packet_t r;
  r.id = p.id;
  r.destination = p.source;
  r.source = p.destination;
  r.payload[0] = 0xff;
  r.payload[1] = 0xff;
  r.payload[2] = 253;

  // TODO: add additional error checking to this
  if(p.payload[4] == AX_READ_DATA)
  {
    int addr = p.payload[5];
    int len = p.payload[6];
    r.payload[3] = len + 2;
    r.payload[4] = 0; // TODO: meaningful error
    for(int i = 0; i<len; i++)
    {
      if(addr == REG_MODEL_NUMBER_L)
        r.payload[5+i] = 45;
      else if(addr == REG_MODEL_NUMBER_H)
        r.payload[5+i] = 1;  // 301
      else if(addr == REG_VERSION)
        r.payload[5+i] = 0;
      else if(addr == REG_ID)
        r.payload[5+i] = 253;
      else if(addr == REG_BAUD_RATE)
        r.payload[5+i] = 34; // 57600????
      //else if(addr == REG_RETURN_DELAY)
      //else if(addr == REG_RETURN_LEVEL)
      //  *x++ = return_level;
      //else if(addr == REG_ALARM_LED)
      //else if(addr == REG_LED)
      else if(addr == REG_PRESENT_VOLTAGE)
        r.payload[5+i] = (uint8_t) (sys_voltage*10.0f);
      else if(addr == REG_CURRENT_L)
        r.payload[5+i] = (uint8_t) (((int)(sys_current/0.0045f)+2048) & 0xff);
      else if(addr == REG_CURRENT_H)
        r.payload[5+i] = (uint8_t) (((int)(sys_current/0.0045f)+2048) >> 8);
      else
        r.payload[5+i] = 0;
      addr++;
    }
    /* checksum */
    uint8_t checksum = 0;
    for(int i = 2; i < 5+len; i++)
        checksum += r.payload[i];
    checksum = 255 - (checksum & 0xff);
    r.payload[5+len] = checksum;
    r.payload_length = 6+len;
    /* dispatch */
    router_dispatch(r);
  }
  else if(p.payload[4] == AX_WRITE_DATA)
  {
    int addr = p.payload[5];
    /* write to table */
    for(int i = 0; i<r.payload[3]-3; i++)
    {
      if(addr == REG_BAUD_RATE){
        // TODO
      }else if(addr == REG_LED){
        if(p.payload[6+i] > 0)
          error::high();
        else
          error::low();
      }
      addr++;
    }
    r.payload_length = 6;
    r.payload[3] = 2;
    r.payload[4] = 0; // TODO: meaningful error?
    r.payload[5] = 0; /* checksum == 0 */
    router_dispatch(r);
  }
}

extern "C"
{

void SysTick_Handler(void)
{
  sys_time++;

  /* check e-stop */
  if(estop::value() > 0){
    // TODO: stop stuffs?
    sys_estop = ESTOP_PRESSED;
    stat::high();
  }else{
    sys_estop = ESTOP_RELEASED;
    stat::low();
  }

  /* toggle activity led */
  if(sys_time - last_packet < 500)
  {
    if(sys_time % 200 == 0)
      act::high();
    else if (sys_time % 100 == 0)
      act::low();
  }
  else
    act::low();
}

}

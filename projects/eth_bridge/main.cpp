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

#include "stm32f4xx.h"
#include "gpio.hpp"
#include "delay.hpp"

#include "netconf.h"
#include "netapp.h"

#include "stm32f4x7_eth.h"

#include "lwip/udp.h"

/* activity, status and error LEDs */
typedef Gpio<GPIOD_BASE,1> act;
typedef Gpio<GPIOD_BASE,3> stat;
typedef Gpio<GPIOD_BASE,4> error;

/* estop switch */
typedef Gpio<GPIOE_BASE,7> estop;

/* Ethernet Pins */
typedef Gpio<GPIOA_BASE,0> eth_mii_crs;
typedef Gpio<GPIOA_BASE,1> eth_rx_clk;
typedef Gpio<GPIOA_BASE,2> eth_mdio;
typedef Gpio<GPIOA_BASE,3> eth_mii_col;
typedef Gpio<GPIOA_BASE,7> eth_mii_rx_dv;

typedef Gpio<GPIOB_BASE,0> eth_mii_rxd2;
typedef Gpio<GPIOB_BASE,1> eth_mii_rxd3;
typedef Gpio<GPIOB_BASE,10> eth_mii_rx_er;
typedef Gpio<GPIOB_BASE,11> eth_mii_tx_en;
typedef Gpio<GPIOB_BASE,12> eth_mii_txd0;
typedef Gpio<GPIOB_BASE,13> eth_mii_txd1;

typedef Gpio<GPIOC_BASE,1> eth_mdc;
typedef Gpio<GPIOC_BASE,2> eth_mii_txd2;
typedef Gpio<GPIOC_BASE,3> eth_mii_tx_clk;
typedef Gpio<GPIOC_BASE,4> eth_mii_rxd0;
typedef Gpio<GPIOC_BASE,5> eth_mii_rxd1;

typedef Gpio<GPIOE_BASE,2> eth_mii_txd3;
typedef Gpio<GPIOE_BASE,3> phy_rst;

/* analog - voltage/current sense */
typedef Gpio<GPIOC_BASE,0> voltage_sense;           /* adc123_in10 */
typedef Gpio<GPIOA_BASE,4> current_sense;           /* adc12_in4 */

/* usart3 - ax/mx bus */
typedef Gpio<GPIOD_BASE,8> usart3_tx;
typedef Gpio<GPIOD_BASE,9> usart3_rx;
typedef Gpio<GPIOD_BASE,10> usart3_en;

/* usart1 - rs-485 bus */
typedef Gpio<GPIOA_BASE,9> usart1_tx;
typedef Gpio<GPIOA_BASE,10> usart1_rx;
typedef Gpio<GPIOA_BASE,11> usart1_en;

unsigned long sys_time;

int main(void)
{
  sys_time = 0;
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

  /* setup usarts
  RCC->APB2ENR |= RCC_APB2ENR_USART1EN; // APB2 also has USART6
  RCC->APB1ENR |= RCC_APB1ENR_USART2EN; // ABP1 also has USART3, UART4/5
  */

  RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
  usart3_tx::mode(GPIO_ALTERNATE | GPIO_AF_USART3);
  usart3_rx::mode(GPIO_ALTERNATE | GPIO_AF_USART3);
  usart3_en::mode(GPIO_OUTPUT);
  //NVIC_EnableIRQ(USART3_IRQn);

  // setup systick
  SysTick_Config(SystemCoreClock/1000);
  NVIC_EnableIRQ(SysTick_IRQn);
  //NVIC_SetPriority(SysTick_IRQn,2);

  Ethernet_Init();
  LwIP_Init();
  if (!netapp_init())
    while(1)
    {
      error::high();
      delay_ms(50);
      error::low();
      delay_ms(1000);
    }

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

    /* process devices */
  }

}

extern "C"
{

void SysTick_Handler(void)
{
  sys_time++;

  /* check e-stop */
  if(estop::value() > 0){
    stat::high();
    // TODO: stop stuffs?
  }else{
    stat::low();
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

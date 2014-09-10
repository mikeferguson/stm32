/*
 * Copyright (c) 2012-2014, Michael E. Ferguson
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

#include "etherbotix.hpp"
#include "stm32f4x7_eth.h"

int main(void)
{
  NVIC_SetPriorityGrouping(3);

  // Enable GPIO
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;

  act::mode(GPIO_OUTPUT);
  error::mode(GPIO_OUTPUT);

  // Setup ethernet
  eth_mdio::mode(GPIO_ALTERNATE | GPIO_AF_ETH);
  eth_mdc::mode(GPIO_ALTERNATE | GPIO_AF_ETH);
  phy_rst::mode(GPIO_OUTPUT_2MHz);
  phy_rst::high(); // release reset line

  eth_rmii_ref_clk::mode(GPIO_ALTERNATE | GPIO_AF_ETH);
  eth_rmii_crs_dv::mode(GPIO_ALTERNATE | GPIO_AF_ETH);

  eth_rmii_tx_en::mode(GPIO_ALTERNATE | GPIO_AF_ETH);
  eth_rmii_txd0::mode(GPIO_ALTERNATE | GPIO_AF_ETH);
  eth_rmii_txd1::mode(GPIO_ALTERNATE | GPIO_AF_ETH);
  eth_rmii_rxd0::mode(GPIO_ALTERNATE | GPIO_AF_ETH);
  eth_rmii_rxd1::mode(GPIO_ALTERNATE | GPIO_AF_ETH);

  // setup systick
  SysTick_Config(SystemCoreClock/1000);
  NVIC_EnableIRQ(SysTick_IRQn);

  __enable_irq();

  while(1)
  {
  }
}

extern "C"
{

void SysTick_Handler(void)
{
}

}

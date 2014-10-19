/*
 * Copyright (c) 2014, Michael E. Ferguson
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
#include "netconf.h"
#include "lwip/memp.h"
#include "lwip/udp.h"
#include "stm32f4x7_eth.h"

/*int udp_interface_init()
{
  eth_udp = udp_new();
  if (eth_udp == NULL)
    return 0;

  if (udp_bind(eth_udp, IP_ADDR_ANY, 6707) != ERR_OK)
    return 0;

  udp_recv(eth_udp, udp_callback, NULL);
  return 1;
}*/

uint32_t system_time;
int main(void)
{
  NVIC_SetPriorityGrouping(3);
  system_time = 0;

  // Enable GPIO
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;

  act::mode(GPIO_OUTPUT);
  error::mode(GPIO_OUTPUT);
  error::high();  // Hold error high during setup

  // Setup ethernet
  setup_gpio_ethernet();

  // Setup systick
  SysTick_Config(SystemCoreClock/1000);
  NVIC_EnableIRQ(SysTick_IRQn);

  LwIP_Init();
  //if (!udp_interface_init())
  //  while(1);

  __enable_irq();
  error::low();  // Done with setup

  while(1)
  {
    LwIP_Periodic_Handle(system_time);
  }
}

extern "C"
{

void SysTick_Handler(void)
{
  ++system_time;
}

}

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

register_table_t register_table;

int main(void)
{
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

  /* Initialize Table */
  register_table.model_number = 302;
  register_table.version = 0;
  register_table.id = 253;
  register_table.baud_rate = 34; // 57600????
  register_table.last_packet = 0;
  register_table.system_time = 0;
  register_table.led = 0;

  dynamixel_init();

  /* setup analog */
  RCC->APB2ENR |= RCC_APB2ENR_ADC1EN | RCC_APB2ENR_ADC2EN | RCC_APB2ENR_ADC3EN;
  adc1.init(VOLTAGE_SENSE_ANALOG_CHANNEL,
            CURRENT_SENSE_ANALOG_CHANNEL);
  voltage_sense::mode(GPIO_INPUT_ANALOG);
  current_sense::mode(GPIO_INPUT_ANALOG);

  /* setup systick */
  SysTick_Config(SystemCoreClock/1000);

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
    LwIP_Periodic_Handle(register_table.system_time);
  }
}

extern "C"
{

void SysTick_Handler(void)
{
  ++register_table.system_time;

  float voltage = (adc1.get_channel1()/4096.0f) * 3.3f * 16.0f;
  register_table.system_voltage = (uint8_t)(voltage * 10.0f);

  float current = (((2048.0f-adc1.get_channel2())/4096.0f) * 3.3f) / 0.055f;  // 55mv/A
  register_table.system_current = (int16_t)(current/0.1f);

  /* check e-stop */
  if(estop::value() > 0){
    // TODO: stop stuffs?
    register_table.estop_status = ESTOP_PRESSED;
    stat::high();
  }else{
    register_table.estop_status = ESTOP_RELEASED;
    stat::low();
  }

  /* toggle activity led */
  if(register_table.system_time - register_table.last_packet < 500)
  {
    if(register_table.system_time % 200 == 0)
      act::high();
    else if (register_table.system_time % 100 == 0)
      act::low();
  }
  else
    act::low();

  adc1.convert();
}

}

/*
 * Copyright (c) 2018-2023, Michael E. Ferguson
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

#ifndef MOTOR_DYNO_CONFIG_HPP
#define MOTOR_DYNO_CONFIG_HPP

#include "delay.hpp"
#include "gpio.hpp"

// Test Controller Pinouts

typedef Gpio<GPIOA_BASE, 1> eth_rmii_ref_clk;
typedef Gpio<GPIOA_BASE, 2> eth_mdio;
typedef Gpio<GPIOA_BASE, 7> eth_rmii_crs_dv;
typedef Gpio<GPIOA_BASE, 8> usart1_en;
typedef Gpio<GPIOA_BASE, 9> usart1_tx;
typedef Gpio<GPIOA_BASE, 10> usart1_rx;

typedef Gpio<GPIOB_BASE, 7> breaker_fault;
typedef Gpio<GPIOB_BASE, 11> eth_rmii_tx_en;
typedef Gpio<GPIOB_BASE, 12> eth_rmii_txd0;
typedef Gpio<GPIOB_BASE, 13> eth_rmii_txd1;

typedef Gpio<GPIOC_BASE, 0> voltage_sense;  /* Analog Channel 10 */
typedef Gpio<GPIOC_BASE, 1> eth_mdc;
typedef Gpio<GPIOC_BASE, 2> breaker_sense;  /* Analog Channel 12 */
typedef Gpio<GPIOC_BASE, 4> eth_rmii_rxd0;
typedef Gpio<GPIOC_BASE, 5> eth_rmii_rxd1;
typedef Gpio<GPIOC_BASE, 13> force_bootloader;
typedef Gpio<GPIOC_BASE, 14> err;
typedef Gpio<GPIOC_BASE, 15> act;

typedef Gpio<GPIOE_BASE, 0> breaker_pg;
typedef Gpio<GPIOE_BASE, 1> breaker_en;
typedef Gpio<GPIOE_BASE, 7> phy_rst;

// Dyno Breakout Pinouts

typedef Gpio<GPIOD_BASE, 5> usart2_tx;  // Buck/Absorber controllers
typedef Gpio<GPIOD_BASE, 6> usart2_rx;
typedef Gpio<GPIOB_BASE, 8> usart2_en;

typedef Gpio<GPIOA_BASE, 5> adc_sck;  // SPI1
typedef Gpio<GPIOA_BASE, 6> adc_miso;
typedef Gpio<GPIOB_BASE, 5> adc_mosi;
typedef Gpio<GPIOC_BASE, 3> adc_reset;
typedef Gpio<GPIOE_BASE, 5> adc_cs;

typedef Gpio<GPIOB_BASE, 10> dut_dac_sck;  // SPI2
typedef Gpio<GPIOB_BASE, 15> dut_dac_mosi;
typedef Gpio<GPIOB_BASE, 1> dut_dac_cs;

typedef Gpio<GPIOD_BASE, 8> usart3_tx;  // Device under test
typedef Gpio<GPIOD_BASE, 9> usart3_rx;
typedef Gpio<GPIOC_BASE, 9> usart3_en;

typedef Gpio<GPIOA_BASE, 3> dut_current_fault;

typedef Gpio<GPIOC_BASE, 10> torque_sensor_cw_cal;
typedef Gpio<GPIOC_BASE, 12> torque_sensor_ccw_cal;

typedef Gpio<GPIOC_BASE, 6> absorber_a;  // TIM3
typedef Gpio<GPIOC_BASE, 7> absorber_b;

typedef Gpio<GPIOE_BASE, 9> alt_enc_a;  // TIM1
typedef Gpio<GPIOE_BASE, 11> alt_enc_b;

typedef Gpio<GPIOC_BASE, 8> status_led_1;
typedef Gpio<GPIOB_BASE, 9> status_led_2;

// Define analog channels

#define VOLTAGE_SENSE_CH      10
#define BREAKER_SENSE_CH      12

// Define analog channel sample times

#define SMPR1_VOLTAGE_SENSE_OFFSET  ((VOLTAGE_SENSE_CH-10)*3)
#define SMPR1_BREAKER_SENSE_OFFSET  ((BREAKER_SENSE_CH-10)*3)

// Init function for ethernet GPIO
inline void setup_gpio_ethernet()
{
  // Setup reset pin, hold low then release. Without this, several
  // of the boards would end up in 10Mbit mode when the proc was reset.
  phy_rst::mode(GPIO_OUTPUT_2MHz);
  phy_rst::low();
  delay_us(5);  // TLK110 specifies minimum 1uS low
  phy_rst::high();

  delay_ms(300);  // Based on 9.7.1

  eth_rmii_ref_clk::mode(GPIO_ALTERNATE | GPIO_AF_ETH);
  eth_rmii_crs_dv::mode(GPIO_ALTERNATE | GPIO_AF_ETH);

  eth_rmii_tx_en::mode(GPIO_ALTERNATE | GPIO_AF_ETH);
  eth_rmii_txd0::mode(GPIO_ALTERNATE | GPIO_AF_ETH);
  eth_rmii_txd1::mode(GPIO_ALTERNATE | GPIO_AF_ETH);
  eth_rmii_rxd0::mode(GPIO_ALTERNATE | GPIO_AF_ETH);
  eth_rmii_rxd1::mode(GPIO_ALTERNATE | GPIO_AF_ETH);

  eth_mdio::mode(GPIO_ALTERNATE | GPIO_AF_ETH);
  eth_mdc::mode(GPIO_ALTERNATE | GPIO_AF_ETH);
}

#endif  // MOTOR_DYNO_CONFIG_HPP

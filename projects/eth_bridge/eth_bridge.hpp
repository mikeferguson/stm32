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

#ifndef ETH_BRIDGE_HPP_
#define ETH_BRIDGE_HPP_

#include "stm32f4xx.h"
#include "gpio.hpp"
#include "lwip/udp.h"

/* Header Stuff */
#define ETH_MAGIC_LENGTH    4

/* Packet Stuff */
#define AX_READ_DATA        2
#define AX_WRITE_DATA       3
#define AX_SYNC_WRITE       131
#define AX_SYNC_READ        132

#define REG_MODEL_NUMBER_L  0
#define REG_MODEL_NUMBER_H  1
#define REG_VERSION         2
#define REG_ID              3
#define REG_BAUD_RATE       4
#define REG_RETURN_DELAY    5
#define REG_RETURN_LEVEL    16
#define REG_ALARM_LED       17
#define REG_LED             25
#define REG_PRESENT_VOLTAGE 42
#define REG_CURRENT_L       68
#define REG_CURRENT_H       69

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

#define ESTOP_RELEASED  1
#define ESTOP_PRESSED   0

extern uint32_t sys_time;
extern uint32_t last_packet;
extern float sys_voltage;
extern float sys_current;
extern uint8_t sys_estop;
extern struct udp_pcb * eth_udp;
extern struct ip_addr return_ipaddr;

#endif // ETH_BRIDGE_HPP_

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
#include "analog_sampler.hpp"
#include "lwip/udp.h"

/* Header Stuff */
#define ETH_MAGIC_LENGTH    4

/* Packet Stuff */
#define AX_READ_DATA        2
#define AX_WRITE_DATA       3
#define AX_SYNC_WRITE       131
#define AX_SYNC_READ        132
#define AX_FULL_SYNC        133

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
#define VOLTAGE_SENSE_ANALOG_CHANNEL    10
typedef Gpio<GPIOA_BASE,4> current_sense;           /* adc12_in4 */
#define CURRENT_SENSE_ANALOG_CHANNEL    4

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

typedef struct
{
  uint16_t model_number;
  uint8_t  version;
  uint8_t  id;
  uint8_t  baud_rate;
  uint8_t  return_delay;
  uint16_t REG_6;
  uint32_t last_packet;
  uint32_t system_time;

  int16_t system_current;
  int16_t computer_current;
  int16_t lf_leg_current;
  int16_t rr_leg_current;
  int16_t rf_leg_current;
  int16_t lr_leg_current;
  uint8_t  system_voltage;
  uint8_t  led;
  uint16_t REG_30;

  uint16_t IMU0;  // TODO: determine IMU data layout
  uint16_t IMU1;
  uint16_t IMU2;
  uint16_t IMU3;
  uint16_t IMU4;
  uint16_t IMU5;
  uint8_t  lf_foot_sensor;
  uint8_t  rr_foot_sensor;
  uint8_t  rf_foot_sensor;
  uint8_t  lr_foot_sensor;

  int16_t  servo_1_pos;
  int16_t  servo_2_pos;
  int16_t  servo_3_pos;
  int16_t  servo_4_pos;
  int16_t  servo_5_pos;
  int16_t  servo_6_pos;
  int16_t  servo_7_pos;
  int16_t  servo_8_pos;

  int16_t  servo_9_pos;
  int16_t  servo_10_pos;
  int16_t  servo_11_pos;
  int16_t  servo_12_pos;
  int16_t  servo_13_pos;
  int16_t  servo_14_pos;
  uint8_t  estop_status;
  uint8_t  REG_77;
  uint16_t REG_78;
} register_table_t;

extern register_table_t register_table;

extern struct udp_pcb * eth_udp;
extern struct ip_addr return_ipaddr;

/* dynamixel */
void dynamixel_init();
void dynamixel_write(uint8_t * packet, uint8_t len);
uint8_t dynamixel_read(uint8_t * packet, uint8_t len, uint8_t * ret_packet);

#endif // ETH_BRIDGE_HPP_

/*
 * Copyright (c) 2013-2014, Michael E. Ferguson
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

#ifndef __ETHERBOTIX_HPP__
#define __ETHERBOTIX_HPP__

#include "stm32f4xx.h"
#include "gpio.hpp"

// Activity and error LEDs
typedef Gpio<GPIOC_BASE, 14> error;
typedef Gpio<GPIOC_BASE, 15> act;

// Ethernet Pins
typedef Gpio<GPIOA_BASE, 2> eth_mdio;
typedef Gpio<GPIOC_BASE, 1> eth_mdc;
typedef Gpio<GPIOE_BASE, 7> phy_rst;

typedef Gpio<GPIOA_BASE, 1> eth_rmii_ref_clk;
typedef Gpio<GPIOA_BASE, 7> eth_rmii_crs_dv;

typedef Gpio<GPIOB_BASE, 11> eth_rmii_tx_en;
typedef Gpio<GPIOB_BASE, 12> eth_rmii_txd0;
typedef Gpio<GPIOB_BASE, 13> eth_rmii_txd1;
typedef Gpio<GPIOC_BASE, 4> eth_rmii_rxd0;
typedef Gpio<GPIOC_BASE, 5> eth_rmii_rxd1;

// Analog
typedef Gpio<GPIOA_BASE, 0> servo_sense;    // servo current: ADC123_IN0
#define SERVO_CURRENT_ANALOG_CHANNEL    0
typedef Gpio<GPIOA_BASE, 3> m2_sense;       // motor 2 current: ADC123_IN3
#define M2_CURRENT_ANALOG_CHANNEL       3
typedef Gpio<GPIOA_BASE, 4> m1_sense;       // motor 1 current: ADC12_IN4
#define M1_CURRENT_ANALOG_CHANNEL       4
typedef Gpio<GPIOB_BASE, 0> a0_sense;       // a0: ADC12_IN8
#define A0_ANALOG_CHANNEL               8
typedef Gpio<GPIOB_BASE, 1> aux_sense;      // aux. current: ADC12_IN9
#define AUX_CURRENT_ANALOG_CHANNEL      9
typedef Gpio<GPIOC_BASE, 0> voltage_sense;  // voltage: ADC123_IN10
#define VOLTAGE_ANALOG_CHANNEL          10
typedef Gpio<GPIOC_BASE, 2> a1_sense;       // a1: ADC123_IN12 (also SPI2_MISO)
#define A1_ANALOG_CHANNEL               12
typedef Gpio<GPIOC_BASE, 3> a2_sense;       // a2: ADC123_IN13 (also SPI2_MOSI)
#define A2_ANALOG_CHANNEL               13

// Motor1
typedef Gpio<GPIOE_BASE, 9> m1_pwm;         // motor1 PWM = tim1_ch1
typedef Gpio<GPIOE_BASE, 8> m1_a;           // motor1 A = tim1_ch1N
typedef Gpio<GPIOE_BASE, 11> m1_b;          // motor1 B = tim1_ch2
typedef Gpio<GPIOE_BASE, 10> m1_en;         // motor1 EN = tim1_ch2N
//typedef Gpio<GPIOE_BASE, 13> m1_en;       // motor1 EN = tim1_ch3 (alternate routing)

// Motor2
typedef Gpio<GPIOC_BASE, 6> m2_pwm;         // motor2 PWM = tim8_ch1
typedef Gpio<GPIOA_BASE, 5> m2_a;           // motor2 A = tim8_ch1N
typedef Gpio<GPIOC_BASE, 7> m2_b;           // motor2 B = tim8_ch2
typedef Gpio<GPIOB_BASE, 14> m2_en;         // motor2 EN = tim8_ch2N/tim12_CH1
//typedef Gpio<GPIOC_BASE, 8> m2_en;        // motor2 EN = tim8_ch3 (alternate routing)

// M1 encoder - tim4
typedef Gpio<GPIOD_BASE, 12> m1_enc_a;      // tim4_ch1
typedef Gpio<GPIOD_BASE, 13> m1_enc_b;      // tim4_ch2

// M2 encoder - tim3
typedef Gpio<GPIOA_BASE, 6> m2_enc_a;       // tim3_ch1
typedef Gpio<GPIOB_BASE, 5> m2_enc_b;       // tim3_ch2

// Usart1 - rs-485 bus
typedef Gpio<GPIOA_BASE, 9> usart1_tx;
typedef Gpio<GPIOA_BASE, 10> usart1_rx;
typedef Gpio<GPIOD_BASE, 15> usart1_en;

// Usart2 - ax/mx bus
typedef Gpio<GPIOD_BASE, 5> usart2_tx;
typedef Gpio<GPIOD_BASE, 6> usart2_rx;
typedef Gpio<GPIOD_BASE, 7> usart2_en;

// IMU - i2c1
typedef Gpio<GPIOB_BASE,6> imu_scl;
typedef Gpio<GPIOB_BASE,7> imu_sda;

// Expandable IO
typedef Gpio<GPIOB_BASE,10> d3;  // also SPI2_SCK, USART3_TX, TIM2_CH3
typedef Gpio<GPIOC_BASE,11> d4;  // also           USART3_RX
typedef Gpio<GPIOB_BASE,15> d5;  // also TIM12_CH2
typedef Gpio<GPIOE_BASE,5> d6;   // also TIM9_CH1
typedef Gpio<GPIOE_BASE,6> d7;   // also TIM9_CH2

#endif // __ETHERBOTIX_HPP__

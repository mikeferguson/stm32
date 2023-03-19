/*
 * Copyright (c) 2013-2023, Michael E. Ferguson
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

#ifndef __TABLEBOT_HPP__
#define __TABLEBOT_HPP__

#include "stm32f4xx.h"
#include "delay.hpp"
#include "gpio.hpp"
#include "usart_dma.hpp"

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
typedef Gpio<GPIOB_BASE, 0> left_cliff;     // a0: ADC12_IN8
#define A0_ANALOG_CHANNEL               8
typedef Gpio<GPIOB_BASE, 1> aux_sense;      // aux. current: ADC12_IN9
#define AUX_CURRENT_ANALOG_CHANNEL      9
typedef Gpio<GPIOC_BASE, 0> voltage_sense;  // voltage: ADC123_IN10
#define VOLTAGE_ANALOG_CHANNEL          10
typedef Gpio<GPIOC_BASE, 2> center_cliff;   // a1: ADC123_IN12 (also SPI2_MISO)
#define A1_ANALOG_CHANNEL               12
typedef Gpio<GPIOC_BASE, 3> right_cliff;    // a2: ADC123_IN13 (also SPI2_MOSI)
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

// Usart1 - rs-485 bus (unused)
typedef Gpio<GPIOA_BASE, 9> usart1_tx;
typedef Gpio<GPIOA_BASE, 10> usart1_rx;
typedef Gpio<GPIOD_BASE, 15> usart1_en;
typedef PeriphReadDMA<uint8_t, DMA2_Stream2_BASE, DMA_FLAG_TCIF2, DMA_Channel_4, USART1_BASE+4, 256> usart1_read_dma;
typedef PeriphWriteDMA<uint8_t, DMA2_Stream7_BASE, DMA_FLAG_TCIF7, DMA_Channel_4, USART1_BASE+4, 256> usart1_write_dma;
typedef UsartDMAWithEnable<USART1_BASE, usart1_en, usart1_read_dma, usart1_write_dma> usart1_t;

// Usart2 - ax/mx bus
typedef Gpio<GPIOD_BASE, 5> usart2_tx;
typedef Gpio<GPIOD_BASE, 6> usart2_rx;
typedef Gpio<GPIOD_BASE, 7> usart2_en;
typedef PeriphReadDMA<uint8_t, DMA1_Stream5_BASE, DMA_FLAG_TCIF5, DMA_Channel_4, USART2_BASE+4, 256> usart2_read_dma;
typedef PeriphWriteDMA<uint8_t, DMA1_Stream6_BASE, DMA_FLAG_TCIF6, DMA_Channel_4, USART2_BASE+4, 256> usart2_write_dma;
typedef UsartDMAWithEnable<USART2_BASE, usart2_en, usart2_read_dma, usart2_write_dma> usart2_t;

// IMU - i2c1
typedef Gpio<GPIOB_BASE, 6> imu_scl;
typedef Gpio<GPIOB_BASE, 7> imu_sda;

// Force bootloader pin
typedef Gpio<GPIOE_BASE, 14> force_bootloader;

// Expandable IO
typedef Gpio<GPIOB_BASE, 10> d3;  // also SPI2_SCK, USART3_TX, TIM2_CH3
typedef Gpio<GPIOC_BASE, 11> laser_rx;
typedef Gpio<GPIOB_BASE, 15> laser_pwm;  // also TIM12_CH2
typedef Gpio<GPIOE_BASE, 5> d6;   // also TIM9_CH1
typedef Gpio<GPIOE_BASE, 6> start_button;

typedef PeriphReadDMA<uint8_t, DMA1_Stream1_BASE, DMA_FLAG_TCIF5, DMA_Channel_4, USART3_BASE+4, 1024> usart3_read_dma;
typedef PeriphWriteDMA<uint8_t, DMA1_Stream3_BASE, DMA_FLAG_TCIF6, DMA_Channel_4, USART3_BASE+4, 256> usart3_write_dma;
typedef UsartDMA<USART3_BASE, usart3_read_dma, usart3_write_dma> usart3_t;

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

// Storage of system data
typedef struct
{
  uint32_t time;
  float voltage;
  float current;
  float servo_current;

  // Cliff sensors
  uint16_t cliff_left;
  uint16_t cliff_center;
  uint16_t cliff_right;

  // IMU data
  int16_t accel_x;
  int16_t accel_y;
  int16_t accel_z;
  int16_t gyro_x;
  int16_t gyro_y;
  int16_t gyro_z;
  int16_t mag_x;
  int16_t mag_y;
  int16_t mag_z;

  int16_t motor1_vel;
  int16_t motor2_vel;
  int32_t motor1_pos;
  int32_t motor2_pos;
  int16_t motor1_current;
  int16_t motor2_current;

  uint8_t run_state;
  uint8_t behavior_state;
  uint16_t neck_angle;

  float pose_x;
  float pose_y;
  float pose_th;

  float block_pose_x;
  float block_pose_y;
  float block_pose_z;

  float goal_pose_x;
  float goal_pose_y;
  float goal_pose_z;

  float target_dist;
  float target_yaw;

  uint32_t last_motor_command;
} system_state_t;

// Encoder is E4P-100-079-D-H-T-B
// 400 PPR * 30:1 Gearbox = 12000 Ticks/Rev
// 12000 cpr / (pi * 73mm diameter)
#define TICK_PER_METER   52324.913f
// Track width tuned on 3/10/2023
#define TRACK_WIDTH      0.150f
// Update motors at 100hz
// Which is every 10th iteration of systick
#define MOTOR_FREQUENCY   100
#define MOTOR_PERIOD      10

// These speeds are the ticks/period to input to the PID
#define MIN_SPEED         25
#define SLOW_SPEED        75
// Standard speed is just under 10cm/sec for starters
#define STANDARD_SPEED    150
// Max speed is 200RPM * 73mm diameter wheel = 0.243 m/s
#define MAX_SPEED         400

// Analog level that indicates cliff detected
// TODO: should this be done with digital?
#define CLIFF_DETECTED    1500

#endif // __TABLEBOT_HPP__

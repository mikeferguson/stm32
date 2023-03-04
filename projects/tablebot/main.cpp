/*
 * Copyright (c) 2012-2023, Michael E. Ferguson
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

#include "tablebot.hpp"
#include "dynamixel.hpp"
#include "pid.hpp"

#include "netconf.h"
#include "lwip/memp.h"
#include "lwip/udp.h"
#include "stm32f4x7_eth.h"

#include "mini_imu9.hpp"
#include "md01.hpp"
#include "encoder.hpp"
#include "usart_dma.hpp"
#include "analog_sampler.hpp"
#include "ld06.hpp"

MiniImu9<I2C1_BASE,
         DMA1_Stream0_BASE, 0 /* stream */, 1 /* channel */,
         imu_scl, imu_sda> imu;
Md01<TIM1_BASE, m1_a, m1_b, m1_en> m1;
Md01<TIM8_BASE, m2_a, m2_b, m2_en> m2;
Encoder<TIM4_BASE> m1_enc;
Encoder<TIM3_BASE> m2_enc;
usart2_t usart2;
DynamixelParser<usart2_t> usart2_parser;
usart3_t usart3;  // laser

LD06<usart3_t> laser;
ld06_packet_t latest_laser_packet;

system_state_t system_state;

struct udp_pcb *eth_udp = NULL;  // The actual UDP port
struct ip_addr return_ipaddr;  // The IP to return stuff to
uint16_t return_port;  // Port to return stuff to

// From IAP app note
typedef  void (*pFunction)(void);

void udp_send_packet(uint8_t * packet, uint8_t len, uint16_t port)
{
  struct pbuf * p_send = pbuf_alloc(PBUF_TRANSPORT, len + 4, PBUF_RAM);
  unsigned char * x = (unsigned char *) p_send->payload;

  // ethernet header
  *x++ = 0xff; *x++ = 'B'; *x++ = 'O'; *x++ = 'T';

  // copy payload
  for(int i = 0; i < len; ++i)
  {
    *x++ = packet[i];
  }

  // send it
  udp_sendto(eth_udp, p_send, &return_ipaddr, port);
  pbuf_free(p_send);
}

void udp_callback(void *arg, struct udp_pcb *udp, struct pbuf *p,
                  struct ip_addr *addr, uint16_t port)
{
  uint8_t * data = (uint8_t*) p->payload;

  // Toss any bad packets
  if ( (p->len < 4) ||
       (p->len != p->tot_len) ||
       (data[0] != 0xff) || (data[1] != 'B') || (data[2] != 'O') || (data[3] != 'T'))
  {
    pbuf_free(p);
    return;
  }

  // Update return data
  return_ipaddr = *addr;
  return_port = port;

  // Send packet
  udp_send_packet((uint8_t*) &system_state, sizeof(system_state), return_port);

  // Free buffer
  pbuf_free(p);
}

int udp_interface_init()
{
  eth_udp = udp_new();
  if (eth_udp == NULL)
    return 0;

  if (udp_bind(eth_udp, IP_ADDR_ANY, 6707) != ERR_OK)
    return 0;

  udp_recv(eth_udp, udp_callback, NULL);
  return 1;
}

int main(void)
{
  // Setup register table data
  system_state.time = 0;

  // TODO: set gains
  m1_pid.set_max_step(10);
  m1_pid.set_gains(1.0, 0.0, 0.0, 0.0);
  m2_pid.set_max_step(10);
  m2_pid.set_gains(1.0, 0.0, 0.0, 0.0);

  NVIC_SetPriorityGrouping(3);

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

  // Setup serial
  RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN | RCC_AHB1ENR_DMA2EN;
  RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
  usart2_tx::mode(GPIO_ALTERNATE | GPIO_AF_USART2);
  usart2_rx::mode(GPIO_ALTERNATE | GPIO_AF_USART2);
  NVIC_SetPriority(USART2_IRQn, 1);
  NVIC_EnableIRQ(USART2_IRQn);
  usart2.init(1000000, 8);

  // Setup motors
  RCC->APB2ENR |= RCC_APB2ENR_TIM1EN | RCC_APB2ENR_TIM8EN;
  m1_pwm::mode(GPIO_ALTERNATE | GPIO_AF_TIM1);
  m2_pwm::mode(GPIO_ALTERNATE | GPIO_AF_TIM8);
  m1.init(17, 1024);  // actually 16.7kHz, 10-bit resolution
  m2.init(17, 1024);

  // Setup encoders
  RCC->APB1ENR |= RCC_APB1ENR_TIM3EN | RCC_APB1ENR_TIM4EN;
  m1_enc_a::mode(GPIO_ALTERNATE | GPIO_AF_TIM4);
  m1_enc_b::mode(GPIO_ALTERNATE | GPIO_AF_TIM4);
  m2_enc_a::mode(GPIO_ALTERNATE | GPIO_AF_TIM3);
  m2_enc_b::mode(GPIO_ALTERNATE | GPIO_AF_TIM3);
  m1_enc.init();
  m2_enc.init();

  // Setup IMU
  imu.init(100000);
  imu.use_magnetometer(true);

  // Setup analog
  RCC->APB2ENR |= RCC_APB2ENR_ADC1EN | RCC_APB2ENR_ADC2EN | RCC_APB2ENR_ADC3EN;
  adc1.init(VOLTAGE_ANALOG_CHANNEL,
            SERVO_CURRENT_ANALOG_CHANNEL,
            AUX_CURRENT_ANALOG_CHANNEL,
            A0_ANALOG_CHANNEL);
  adc2.init(M1_CURRENT_ANALOG_CHANNEL,
            M2_CURRENT_ANALOG_CHANNEL,
            A1_ANALOG_CHANNEL,
            A2_ANALOG_CHANNEL);  // TODO: set trigger on motor timer
  voltage_sense::mode(GPIO_INPUT_ANALOG);
  servo_sense::mode(GPIO_INPUT_ANALOG);
  adc1.setSampleTime(SERVO_CURRENT_ANALOG_CHANNEL, ADC_SampleTime_84Cycles);
  aux_sense::mode(GPIO_INPUT_ANALOG);
  adc1.setSampleTime(AUX_CURRENT_ANALOG_CHANNEL, ADC_SampleTime_84Cycles);
  m1_sense::mode(GPIO_INPUT_ANALOG);
  m2_sense::mode(GPIO_INPUT_ANALOG);

  // Initialize cliff sensor pins
  left_cliff::mode(GPIO_INPUT_ANALOG);
  center_cliff::mode(GPIO_INPUT_ANALOG);
  right_cliff::mode(GPIO_INPUT_ANALOG);

  // Laser interface
  RCC->APB1ENR |= RCC_APB1ENR_USART3EN | RCC_APB1ENR_TIM12EN;
  laser_rx::mode(GPIO_ALTERNATE | GPIO_AF_USART3);
  laser_pwm::mode(GPIO_ALTERNATE | GPIO_AF_TIM12);
  laser.init(&usart3);

  // Start button
  start_button::mode(GPIO_INPUT);
  start_button::low();

  // Unused IO
  d3::mode(GPIO_INPUT);
  d6::mode(GPIO_INPUT);

  // Setup systick
  SysTick_Config(SystemCoreClock/1000);

  LwIP_Init();
  if (!udp_interface_init())
    while(1);

  __enable_irq();
  error::low();  // Done with setup

  while(1)
  {
    LwIP_Periodic_Handle(system_state.time);

    if (imu.update(system_state.time))
    {
      system_state.accel_x = imu.accel_data.x;
      system_state.accel_y = imu.accel_data.y;
      system_state.accel_z = imu.accel_data.z;

      system_state.gyro_x = imu.gyro_data.x;
      system_state.gyro_y = imu.gyro_data.y;
      system_state.gyro_z = imu.gyro_data.z;

      system_state.mag_x = imu.mag_data.x;
      system_state.mag_y = imu.mag_data.y;
      system_state.mag_z = imu.mag_data.z;
    }

    // Attempt to read from laser data
    int8_t length = laser.update(&usart3, system_state.time);
    if (length > 0)
    {
      // Parse packet into global view
      float angle = laser.packet.start_angle * 0.01f;
      float end_angle = laser.packet.end_angle * 0.01f;
      float step = (end_angle - angle) / (laser.packet.length - 1);

      // Decide where to insert this data
      int index = angle * 0.8f;
      for (int i = 0; i < length; ++i)
      {
        system_state.laser_data[index + i] = laser.packet.data[i].range;
        system_state.laser_data[index + i] = angle;
        angle += step;
      }
    }
    else if (length < 0)
    {
      laser.reset(&usart3);
    }

    // TODO: control loop
    m1_pid.update_setpoint(0);
    m2_pid.update_setpoint(0);
  }
}

extern "C"
{

void SysTick_Handler(void)
{
  ++system_state.time;

  // Get system voltage:
  //   adc is 12 bit (4096 count) spread over 3.3V
  //   voltage divider is 15k/1k
  system_state.voltage = (adc1.get_channel1() / 4096.0f) * 3.3f * 16;

  // Get system/servo currents:
  //   ACS711: vcc/2 = 0A, 55mV/A
  system_state.current = ((adc1.get_channel3()-2048.0f)/4096.0f) * 3.3f / 0.055f;
  system_state.servo_current = ((adc1.get_channel2()-2048.0f)/4096.0f) * 3.3f / 0.055f;

  // Analog channels
  system_state.cliff_left = adc1.get_channel4();
  system_state.cliff_center = adc2.get_channel3();
  system_state.cliff_right = adc2.get_channel4();

  // Motor current sense channels
  //system_state.motor1_current = adc2.get_channel1();
  //system_state.motor2_current = adc2.get_channel2();

  // Update motors
  if (system_state.time % 10 == 0)
  {
    // Update table with position/velocity
    system_state.motor1_pos = m1_enc.read();
    system_state.motor2_pos = m2_enc.read();
    system_state.motor1_vel = m1_enc.read_speed();
    system_state.motor2_vel = m2_enc.read_speed();

    // Update PID and set motor commands
    m1.set(m1_pid.update_pid(system_state.motor1_vel));
    m2.set(m2_pid.update_pid(system_state.motor2_vel));
  }

  // Toggle LED
  if (system_state.time % 200 == 0)
    act::high();
  else if (system_state.time % 100 == 0)
    act::low();

  // Start next conversion of voltage/current
  adc1.convert();
  adc2.convert();
}

// Turn around the ax/mx bus
void USART2_IRQHandler(void)
{
  usart2.usartIrqHandler();
}

}

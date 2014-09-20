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

#include "netconf.h"
#include "lwip/memp.h"
#include "lwip/udp.h"
#include "stm32f4x7_eth.h"

#include "analog_sampler.hpp"

struct udp_pcb *eth_udp = NULL;  // The actual UDP port
struct ip_addr return_ipaddr;  // The IP to return stuff to
uint16_t return_port;  // Port to return stuff to

registers_t registers;  // Register data
uint32_t last_packet;  // Timestamp of last packet

void udp_send_packet(uint8_t * packet, uint8_t len, uint16_t port)
{
  struct pbuf * p_send = pbuf_alloc(PBUF_TRANSPORT, len + 4, PBUF_RAM);
  unsigned char * x = (unsigned char *) p_send->payload;

  // ethernet header
  *x++ = 0xff; *x++ = 'B'; *x++ = 'O'; *x++ = 'T';

  // copy payload
  for(int i = 0; i < len; ++i)
    *x++ = packet[i];

  // send it
  udp_sendto(eth_udp, p_send, &return_ipaddr, port);
  pbuf_free(p_send);
}

void udp_callback(void *arg, struct udp_pcb *udp, struct pbuf *p,
                  struct ip_addr *addr, uint16_t port)
{
  uint8_t * data = (uint8_t*) p->payload;

  // Toss any bad packets
  if ( (p->len < 10) ||
       (p->len != p->tot_len) ||
       (data[0] != 0xff) || (data[1] != 'B') || (data[2] != 'O') || (data[3] != 'T'))
  {
    ++registers.packets_bad;
    pbuf_free(p);
    return;
  }

  // TODO

  // Free buffer
  pbuf_free(p);
  ++registers.packets_recv;
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
  registers.model_number = 301;  // Arbotix was 300
  registers.version = 0;
  registers.id = 253;
  registers.baud_rate = 34;  // 56700
  registers.digital_dir = 0;  // all in
  registers.digital_out = 0;
  registers.system_time = last_packet = 0;
  registers.motor_period = 10;  // 10mS period = 100hz
  registers.motor1_kp = registers.motor2_kp = 0;
  registers.motor1_kd = registers.motor2_kd = 0;
  registers.motor1_ki = registers.motor2_ki = 0;
  registers.motor1_windup = registers.motor2_windup = 0;
  registers.usart3_baud = 34;  // 56700
  registers.packets_recv = registers.packets_bad = 0;

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

  // Setup analog
  RCC->APB2ENR |= RCC_APB2ENR_ADC1EN | RCC_APB2ENR_ADC2EN | RCC_APB2ENR_ADC3EN;
  adc1.init(VOLTAGE_ANALOG_CHANNEL,
            SERVO_CURRENT_ANALOG_CHANNEL,
            AUX_CURRENT_ANALOG_CHANNEL,
            A0_ANALOG_CHANNEL);
  adc2.init(M1_CURRENT_ANALOG_CHANNEL,
            M2_CURRENT_ANALOG_CHANNEL,
            A2_ANALOG_CHANNEL,
            A2_ANALOG_CHANNEL);  // TODO: set trigger on motor timer
  voltage_sense::mode(GPIO_INPUT_ANALOG);  // TODO: set sample times
  servo_sense::mode(GPIO_INPUT_ANALOG);
  aux_sense::mode(GPIO_INPUT_ANALOG);
  a0_sense::mode(GPIO_INPUT_ANALOG);
  m1_sense::mode(GPIO_INPUT_ANALOG);
  m2_sense::mode(GPIO_INPUT_ANALOG);
  a1_sense::mode(GPIO_INPUT_ANALOG);
  a2_sense::mode(GPIO_INPUT_ANALOG);

  // Setup systick
  SysTick_Config(SystemCoreClock/1000);
  NVIC_EnableIRQ(SysTick_IRQn);

  LwIP_Init();
  if (!udp_interface_init())
    while(1);

  __enable_irq();
  error::low();  // Done with setup

  while(1)
  {
    LwIP_Periodic_Handle(registers.system_time);
  }
}

extern "C"
{

void SysTick_Handler(void)
{
  ++registers.system_time;

  // Get system voltage in 0.1V increment:
  //   adc is 12 bit (4096 count) spread over 3.3V
  //   voltage divider is 15k/1k
  registers.system_voltage = (adc1.get_channel1()/4096.0f) * 3.3f * 16 * 10;

  // Get aux/servo currents:
  //   ACS711: vcc/2 = 0A, 55mV/A
  registers.servo_current = ((adc1.get_channel2()-2048)/4096.0f) * 3.3f * 0.055f;
  registers.aux_current = ((adc1.get_channel3()-2048)/4096.0f) * 3.3f * 0.055f;

  // Analog channels
  registers.a0 = adc1.get_channel4();
  registers.a1 = adc2.get_channel3();
  registers.a2 = adc2.get_channel4();

  // Toggle LED
  if (1) //registers.system_time - last_packet < 500)
  {
    if (registers.system_time % 200 == 0)
      act::high();
    else if (registers.system_time % 100 == 0)
      act::low();
  }

  // Start next conversion of voltage/current
  adc1.convert();
  adc2.convert();
}

}

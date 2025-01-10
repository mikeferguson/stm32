/*
 * Copyright (c) 2012-2025, Michael E. Ferguson
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


#include "stm32f4xx.h"
#include "stm32f4xx_adc.h"
#include "config.hpp"

#include "netconf.h"
#include "lwip/memp.h"
#include "lwip/udp.h"
#include "stm32f4x7_eth.h"

#include "usart_dma.hpp"
typedef PeriphReadDMA<uint8_t, DMA1_Stream5_BASE, DMA_FLAG_TCIF5, DMA_Channel_4, USART2_BASE+4, 256> usart2_read_dma;
typedef PeriphWriteDMA<uint8_t, DMA1_Stream6_BASE, DMA_FLAG_TCIF6, DMA_Channel_4, USART2_BASE+4, 256> usart2_write_dma;
typedef UsartDMAWithEnable<USART2_BASE, usart2_en, usart2_read_dma, usart2_write_dma> usart2_t;
usart2_t usart2;

typedef PeriphReadDMA<uint8_t, DMA1_Stream1_BASE, DMA_FLAG_TCIF5, DMA_Channel_4, USART3_BASE+4, 256> usart3_read_dma;
typedef PeriphWriteDMA<uint8_t, DMA1_Stream3_BASE, DMA_FLAG_TCIF6, DMA_Channel_4, USART3_BASE+4, 256> usart3_write_dma;
typedef UsartDMAWithEnable<USART3_BASE, usart3_en, usart3_read_dma, usart3_write_dma> usart3_t;
usart3_t usart3;

#include "ads8684.hpp"
typedef PeriphReadDMA<uint8_t, DMA2_Stream0_BASE, DMA_FLAG_TCIF0, DMA_Channel_3, SPI1_BASE+12, 12> spi1_read_dma;
typedef PeriphWriteDMA<uint8_t, DMA2_Stream3_BASE, DMA_FLAG_TCIF3, DMA_Channel_3, SPI1_BASE+12, 4> spi1_write_dma;
ADS8684<SPI1_BASE, adc_cs, adc_reset, spi1_read_dma, spi1_write_dma> adc;

#include "dac121.hpp"
DAC121<SPI2_BASE, dut_dac_cs> dac;

// Encoder is AMT102-V, set to 2048PPR = 8192CPR
// Belt reduction is 74:21
#include "encoder.hpp"
Encoder<TIM3_BASE> absorber_enc;
#define ABSORBER_CPR            8192.0f * 21.0f / 71.0f
#define TO_RADIANS              (2.0f * 3.141592653589793f)
#define VELOCITY_FILTER_COEFF   0.3f

#include "copy_float.hpp"

typedef struct
{
  uint32_t system_time;  // 1/25000 hz
  float system_voltage;  // Volts
  float buck_voltage;    // Volts
  float buck_current;    // Amps
  float torque;          // Nm
  float position;        // rad
  float velocity;        // rad/S
} dyno_data_t;

dyno_data_t dyno;
uint32_t last_packet;

// Set Vsense to +/-5.12V range
// TODO: this can probably be tighter
#define BUCK_VSENSE_RANGE ADS8684_RANGE_5v12
float convertBuckVoltage(uint16_t raw)
{
  // Scale to volts
  float voltage = static_cast<float>(raw - 32768) / 32768.0f * 5.12f;
  // Apply reverse transformation through AMC1100 (8x gain) and voltage divider (1k/430k)
  // Also apply correction for specific model since resistors are 5% (eww)
  voltage *= 53.875f * 1.0394f;
  return voltage;
}

// ACS711 has 0-5V range
#define BUCK_CURRENT_RANGE ADS8684_RANGE_P5v12
float convertBuckCurrent(uint16_t raw)
{
  // Scale to volts
  float voltage = static_cast<float>(raw) / 65536.0f * 5.12f;
  // TODO: calibrate offset
  float current = voltage - 1.65f;
  // ACS711 has 50mV/A output
  current /= 0.050f;
  // Apply (unknown correction)
  current *= 0.91f;
  return current;
}

// Torque sensor needs full +/-10.24V range
#define TORQUE_SENSOR_RANGE ADS8684_RANGE_10v24
float convertTorqueSensor(uint16_t raw)
{
  // Torque sensor is 56.5Nm/10V
  return (raw - 32767) * 0.001765625f;
}

struct udp_pcb *eth_udp = NULL;  // The actual UDP port

// From IAP app note
typedef  void (*pFunction)(void);

void udp_callback(void *arg, struct udp_pcb *udp, struct pbuf *p,
                  struct ip_addr *addr, uint16_t port)
{
  uint8_t * data = (uint8_t*) p->payload;

  // Toss any bad packets
  if ((p->len < 4) ||
      (p->len != p->tot_len) ||
      (data[0] != 'D') || (data[1] != 'Y') || (data[2] != 'N') || (data[3] != 'O'))
  {
    // TODO: increase bad packet counter?
    pbuf_free(p);
    return;
  }

  // Valid packet, turn on activity light
  last_packet = 0;

  uint8_t idx = 4;
  while (idx < p->len)
  {
    if (data[idx] == 'S')
    {
      // Next 4 bytes are desired absorber speed as float
      float desired_absorber_speed = copyFloat(data[idx + 1]);
      // TODO: send speed command to absorber controller
      idx += 5;
    }
    else if (data[idx] == 'V')
    {
      // Next 4 bytes are desired buck voltage as float
      float desired_buck_voltage = copyFloat(data[idx + 1]);
      // TODO: send voltage command to buck controller
      idx += 5;
    }
    else if (data[idx] == 'D')
    {
      // Command to device under test
      if (data[idx + 1] == '4')
      {
        // RS-485 packet to send
        uint8_t len = data[idx + 2];
        if (usart3.done())
        {
          usart3.write(&data[idx + 3], len);
        }
        idx += 2 + len;
      }
      else if (data[idx + 1] == 'A')
      {
        // Next 4 bytes are desired analog output as float
        float desired_dut_voltage = copyFloat(data[idx + 2]);
        dac.setOutput(desired_dut_voltage / 5.0f * 4095);
        idx += 6;
      }
      else if (data[idx + 1] == 'B')
      {
        // Next 2 bytes are desired baud rate
        uint16_t baud = (data[idx + 2] << 8) + data[idx + 3];
        usart3.init(baud, 8);
      }
    }
  }

  // Update data from ADC before sending return packet
  dyno.buck_current = convertBuckCurrent(adc.getChannel(0));
  dyno.buck_voltage = convertBuckVoltage(adc.getChannel(1));
  dyno.torque = convertTorqueSensor(adc.getChannel(2));

  // Send return packet
  struct pbuf * p_send = pbuf_alloc(PBUF_TRANSPORT, sizeof(dyno) + 4, PBUF_RAM);
  unsigned char * x = (unsigned char *) p_send->payload;

  // ethernet header
  *x++ = 'D'; *x++ = 'Y'; *x++ = 'N'; *x++ = 'O';

  // Copy packet data
  __disable_irq();
  uint8_t * dyno_data = (uint8_t *) &dyno;
  for (int i = 0; i < sizeof(dyno_data_t); ++i)
  {
    *x++ = *(dyno_data++);
  }
  __enable_irq();

  // send it
  udp_sendto(eth_udp, p_send, addr, port);

  // Free buffer
  pbuf_free(p_send);
  pbuf_free(p);
}

int udp_interface_init()
{
  eth_udp = udp_new();
  if (eth_udp == NULL)
    return 0;

  if (udp_bind(eth_udp, IP_ADDR_ANY, 5000) != ERR_OK)
    return 0;

  udp_recv(eth_udp, udp_callback, NULL);
  return 1;
}

int main(void)
{
  NVIC_SetPriorityGrouping(3);
  dyno.system_time = 0;
  last_packet = 5000;

  // Enable GPIO
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;

  act::mode(GPIO_OUTPUT);
  err::mode(GPIO_OUTPUT);
  err::high();  // Hold error high during setup

  // Setup ethernet
  setup_gpio_ethernet();

  // Setup buck/absorber serial
  RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN | RCC_AHB1ENR_DMA2EN;
  RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
  usart2_tx::mode(GPIO_ALTERNATE | GPIO_AF_USART2);
  usart2_rx::mode(GPIO_ALTERNATE | GPIO_AF_USART2);
  NVIC_SetPriority(USART2_IRQn, 1);
  NVIC_EnableIRQ(USART2_IRQn);
  usart2.init(1000000, 8);

  // Setup DUT serial
  RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
  usart3_tx::mode(GPIO_ALTERNATE | GPIO_AF_USART3);
  usart3_rx::mode(GPIO_ALTERNATE | GPIO_AF_USART3);
  NVIC_SetPriority(USART3_IRQn, 1);
  NVIC_EnableIRQ(USART3_IRQn);
  usart3.init(1000000, 8);

  // Absorber Speed
  RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
  absorber_a::mode(GPIO_ALTERNATE | GPIO_AF_TIM3);
  absorber_b::mode(GPIO_ALTERNATE | GPIO_AF_TIM3);
  absorber_enc.init();

  // Setup analog
  RCC->APB2ENR |= RCC_APB2ENR_ADC1EN | RCC_APB2ENR_ADC2EN | RCC_APB2ENR_ADC3EN;
  voltage_sense::mode(GPIO_INPUT_ANALOG);
  breaker_sense::mode(GPIO_INPUT_ANALOG);
  // PCLK2 = 84MHZ, ADC_CLK has max of 36MHz
  //  ADCPRE = 01b = divide PCLK by 4
  ADC->CCR |= ADC_CCR_ADCPRE_0;
  // ADC1 will convert voltage then breaker current sense. Triggered by systick.
  ADC1->SMPR1 |= (ADC_SampleTime_15Cycles << SMPR1_VOLTAGE_SENSE_OFFSET) |
                 (ADC_SampleTime_15Cycles << SMPR1_BREAKER_SENSE_OFFSET);
  ADC1->JSQR = (1 << 20)    // Sample 2 channels
             | VOLTAGE_SENSE_CH << 10
             | BREAKER_SENSE_CH << 15;
  ADC1->CR1 = ADC_CR1_SCAN;
  ADC1->CR2  = ADC_CR2_ADON;

  // Setup fast ADC
  adc_sck::mode(GPIO_ALTERNATE | GPIO_AF_SPI1);
  adc_mosi::mode(GPIO_ALTERNATE | GPIO_AF_SPI1);
  adc_miso::mode(GPIO_ALTERNATE | GPIO_AF_SPI1);
  adc.init();
  adc.setChannelRange(0, BUCK_CURRENT_RANGE);
  adc.setChannelRange(1, BUCK_VSENSE_RANGE);
  adc.setChannelRange(2, TORQUE_SENSOR_RANGE);
  adc.setChannelPower(3, false);  // power down channel 3
  DMA2_Stream3->CR |= DMA_SxCR_TCIE;
  NVIC_SetPriority(DMA2_Stream3_IRQn, 1);
  NVIC_EnableIRQ(DMA2_Stream3_IRQn);

  // Setup DAC output
  dut_dac_sck::mode(GPIO_ALTERNATE | GPIO_AF_SPI2);
  dut_dac_mosi::mode(GPIO_ALTERNATE | GPIO_AF_SPI2);
  dac.init();

  // Setup systick
  SysTick_Config(SystemCoreClock / 25000);

  LwIP_Init();
  if (!udp_interface_init())
    while(1);

  err::low();  // Done with setup

  __enable_irq();

  while(1)
  {
    LwIP_Periodic_Handle(dyno.system_time);
  }
}

extern "C"
{

// Systick runs at 25khz
void SysTick_Handler(void)
{
  // Update system time
  ++dyno.system_time;
  ++last_packet;

  // Get system voltage
  //   adc is 12 bit (4096 count) spread over 3.3V
  //   voltage divider is 20k/1k
  dyno.system_voltage = (ADC1->JDR1 / 4096.0f) * 3.3f * 21.0f;
  ADC1->CR2 |= ADC_CR2_JSWSTART;

  // Encoder update runs at 250hz
  if (dyno.system_time % 100 == 0)
  {
    int32_t pos = absorber_enc.read();
    int32_t vel = absorber_enc.read_speed();
    // Convert position of encoder (not usually displayed, but useful for debugging)
    dyno.position = static_cast<float>(pos) / ABSORBER_CPR * TO_RADIANS;
    // Convert velocity and then filter it in to estimate
    float v = (static_cast<float>(vel) * 25000.0f / 100 / ABSORBER_CPR) * TO_RADIANS;
    dyno.velocity = (1 - VELOCITY_FILTER_COEFF) * dyno.velocity + VELOCITY_FILTER_COEFF * v;
  }

  adc.update();

  if (last_packet < 5000)
  {
    // Activity LED on
    act::high();
  }
  else
  {
    act::low();
  }
}

// Turn around the Buck/Absorber RS-485 bus
void USART2_IRQHandler(void)
{
  usart2.usartIrqHandler();
}

// Turn around the DUT RS-485 bus
void USART3_IRQHandler(void)
{
  usart3.usartIrqHandler();
}

void DMA2_Stream3_IRQHandler(void)
{
  // Call handler
  adc.irqHandler();
}

}

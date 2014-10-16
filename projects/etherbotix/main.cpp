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
#include "dynamixel.hpp"
#include "pid.hpp"

#include "netconf.h"
#include "lwip/memp.h"
#include "lwip/udp.h"
#include "stm32f4x7_eth.h"

#include "mini_imu9_v2.hpp"
#include "md01.hpp"
#include "encoder.hpp"
#include "usart_dma.hpp"
#include "analog_sampler.hpp"

MiniImu9v2<I2C1_BASE,
           DMA1_Stream0_BASE, 0 /* stream */, 1 /* channel */,
           imu_scl, imu_sda> imu;
Md01<TIM1_BASE, m1_a, m1_b, m1_en> m1;
Md01<TIM8_BASE, m2_a, m2_b, m2_en> m2;
Encoder<TIM4_BASE> m1_enc;
Encoder<TIM3_BASE> m2_enc;
usart1_t usart1;
usart2_t usart2;
DynamixelParser<usart1_t> usart1_parser;
DynamixelParser<usart2_t> usart2_parser;

registers_t registers;  // Register data
uint32_t last_packet;  // Timestamp of last packet
uint32_t last_motor_cmd;  // Timestamp of last motor command

#include "user_io.hpp"

// Setup the baud rate of dynamixel ports
int set_dynamixel_baud(uint8_t value)
{
  uint32_t baud = 0;
  if (value == 1)
    baud = 1000000;
  else if (value == 3)
    baud = 500000;
  else if (value == 34)
    baud = 57600;
  else if (value == 250)
    baud = 2250000;
  else if (value == 251)
    baud = 2500000;
  else if (value == 252)
    baud = 3000000;
  else
    return -1;  // Unsupported baud rate

  usart1.init(baud, 8);
  usart2.init(baud, 8);
  registers.baud_rate = value;
  return 1;
}

struct udp_pcb *eth_udp = NULL;  // The actual UDP port
struct ip_addr return_ipaddr;  // The IP to return stuff to
uint16_t return_port;  // Port to return stuff to


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

  // Update return data
  last_packet = registers.system_time;
  return_ipaddr = *addr;
  return_port = port;

  // For each packet
  size_t i = 4;
  while (i < p->len)
  {
    if ((data[i] != 0xff) || (data[i+1] != 0xff))
    {
      // Packet has become corrupted?
      ++registers.packets_bad;
      pbuf_free(p);
      return;
    }

    uint8_t id = data[i+2];
    uint8_t len = data[i+3];
    uint8_t instruction = data[i+4];

    if (id == ETHERBOTIX_ID)
    {
      // Process packets for self
      if (instruction == DYN_READ_DATA)
      {
        uint8_t read_addr = data[i+5];
        uint8_t read_len = data[i+6];

        // Update anything that isn't periodically updated
        user_io_update();

        uint8_t packet[256];
        packet[0] = 0xff;
        packet[1] = 0xff;
        packet[2] = ETHERBOTIX_ID;
        packet[3] = read_len;
        packet[4] = 0;  // No error
        packet[5+read_len] = ETHERBOTIX_ID + read_len;  // Init checksum

        // Copy packet data
        uint8_t * reg_data = (uint8_t *) &registers;
        reg_data += read_addr;
        for (int j = 0; j < read_len; ++j)
        {
          packet[5+j] = *(reg_data++);
          packet[5+read_len] += packet[5+j];
        }

        packet[5+read_len] = 255 - packet[5+read_len];  // Compute checksum
        udp_send_packet(packet, read_len + 6, port);
      }
      else if (instruction == DYN_WRITE_DATA)
      {
        uint8_t write_addr = data[i+5];
        if (write_addr >= 128)
        {
          // This is a device, write all data to single place
          if (write_addr == DEVICE_USART3_DATA)
          {
            user_io_usart_write(&data[i+6], len-3);
          }
          else if (write_addr == DEVICE_SPI2_DATA)
          {
            // TODO
          }
        }
        else
        {
          int j = 0;
          bool update_gains = false;
          while (j < len -3)
          {
            if (write_addr + j == REG_BAUD_RATE)
            {
              // Update baud rate
              uint8_t baud = data[i+6+j];
              set_dynamixel_baud(baud);
            }
            else if (write_addr + j == REG_DELAY_TIME)
            {
              // TODO
            }
            else if (write_addr + j == REG_DIGITAL_IN)
            {
              user_io_update_mask(data[i+6+j]);
            }
            else if (write_addr + j == REG_DIGITAL_OUT)
            {
              user_io_set_output(data[i+6+j]);
            }
            else if (write_addr + j == REG_DIGITAL_DIR)
            {
              user_io_set_direction(data[i+6+j]);
            }
            else if (write_addr + j == REG_LED)
            {
              registers.led = data[i+6+j];
              if (data[i+6+j] > 0)
                error::high();
              else
                error::low();
            }
            else if (write_addr + j == REG_MOTOR_PERIOD)
            {
              if (data[i+6+j] > 0 && data[i+6+j] < 100)
                registers.motor_period = data[i+6+j];
            }
            else if (write_addr + j == REG_MOTOR_MAX_STEP)
            {
              int16_t v = data[i+6+j] + (data[i+7+j]<<8);
              m1_pid.set_max_step(v);
              m2_pid.set_max_step(v);
              ++j;  // uses 2 bytes
            }
            else if (write_addr + j == REG_MOTOR1_VEL)
            {
              // Write 16-bit setpoint
              int16_t v = data[i+6+j] + (data[i+7+j]<<8);
              m1_pid.update_setpoint(v);
              last_motor_cmd = registers.system_time;
              ++j;  // uses 2 bytes
            }
            else if (write_addr + j == REG_MOTOR2_VEL)
            {
              // Write 16-bit setpoint
              int16_t v = data[i+6+j] + (data[i+7+j]<<8);
              m2_pid.update_setpoint(v);
              last_motor_cmd = registers.system_time;
              ++j;  // uses 2 bytes
            }
            else if (write_addr + j >= REG_MOTOR1_KP &&
                     write_addr + j < REG_ACC_X)
            {
              // Updating gains
              uint8_t * reg_data = (uint8_t *) &registers;
              reg_data[write_addr + j] = data[i+6+j];
              update_gains = true;
            }
            else if (write_addr + j == REG_USART3_BAUD)
            {
              // Set baud, start usart
              registers.usart3_baud = data[i+6+j];
              user_io_usart_init();
            }
            else if (write_addr + j == REG_SPI2_BAUD)
            {
              // TODO Set baud rate of SPI2
            }
            else if (write_addr + j == REG_TIM12_MODE)
            {
              user_io_tim12_init(data[i+6+j]);
            }
            else
            {
              // INSTRUCTION ERROR on invalid write?
            }
            ++j;
          }
          if (update_gains)
          {
            // Stop motors
            m1_pid.reset();
            m2_pid.reset();

            // Actually update gains
            m1_pid.set_gains(registers.motor1_kp, registers.motor1_kd,
                             registers.motor1_ki, registers.motor1_windup);
            m2_pid.set_gains(registers.motor2_kp, registers.motor2_kd,
                             registers.motor2_ki, registers.motor2_windup);
          }
        }
      }
    }
    else  // Pass through to servos
    {
      // Make sure any previous packets are done sending
      while (!(usart1.done() && usart2.done()))
        ;

      // Serial bus deals in 16-bit data, need to copy packet
      uint16_t packet[256];
      for (int j = 0; j < len+4; ++j)
        packet[j] = data[i+j];

      // Reset parsers
      usart1_parser.reset(&usart1);
      usart2_parser.reset(&usart2);

      if (instruction == DYN_READ_DATA)
      {
        // Blast packet to each bus
        usart1.write(packet, len+4);
        usart2.write(packet, len+4);

        // Wait for send to complete
        while (!(usart1.done() && usart2.done()))
          ;

        // Wait for response on at least one bus
        while (true)
        {
          int8_t p1 = usart1_parser.parse(&usart1, registers.system_time);
          if (p1 > 0)
          {
            // Got a packet
            uint8_t packet[256];
            packet[0] = 0xff;
            packet[1] = 0xff;
            packet[2] = usart1_parser.packet.id;
            packet[3] = usart1_parser.packet.length;
            packet[4] = usart1_parser.packet.error;
            for (int j = 0; j < usart1_parser.packet.length-2; ++j)
              packet[5+j] = usart1_parser.packet.parameters[j];
            packet[3+usart1_parser.packet.length] = usart1_parser.packet.checksum;
            udp_send_packet(packet, usart1_parser.packet.length + 4, port);
            break;
          }

          int8_t p2 = usart2_parser.parse(&usart2, registers.system_time);
          if (p2 > 0)
          {
            // Got a packet
            uint8_t packet[256];
            packet[0] = 0xff;
            packet[1] = 0xff;
            packet[2] = usart2_parser.packet.id;
            packet[3] = usart2_parser.packet.length;
            packet[4] = usart2_parser.packet.error;
            for (int j = 0; j < usart2_parser.packet.length-2; ++j)
              packet[5+j] = usart2_parser.packet.parameters[j];
            packet[3+usart2_parser.packet.length] = usart2_parser.packet.checksum;
            udp_send_packet(packet, usart2_parser.packet.length + 4, port);
            break;
          }

          if (p1 < 0 && p2 < 0)
          {
            // Timeout or other error
            break;
          }
        }
      }
      else if (instruction == DYN_SYNC_READ)
      {
        // What to read
        uint8_t read_addr = data[i+5];  // First parameter is the read address
        uint8_t read_len = data[i+6];   // Second parameter is # of bytes to read

        // Single response packet to send back to PC from several servo packets
        uint8_t packet[256];
        packet[0] = 0xff;
        packet[1] = 0xff;
        packet[2] = 0xfe;  // broadcast
        packet[3] = 2 + (read_len*(len-4));
        packet[4] = 0;  // No error
        uint8_t packet_idx = 5;
        uint8_t packet_chk = 0xfe + packet[3];

        // Read data from each servo
        for (int j = 2; j < len-2; ++j)
        {
          uint16_t pkt[256];
          pkt[0] = 0xff;
          pkt[1] = 0xff;
          pkt[2] = data[i+5+j];  // ID of this servo
          pkt[3] = 4;  // len remaining
          pkt[4] = DYN_READ_DATA;
          pkt[5] = read_addr;
          pkt[6] = read_len;
          uint8_t chk = 0;
          for (int p = 2; p < 7;p++)
            chk += pkt[p];
          pkt[7] = 255 - chk;

          // Reset parsers
          usart1_parser.reset(&usart1);
          usart2_parser.reset(&usart2);

          // Send read to each bus
          usart1.write(pkt, 8);
          usart2.write(pkt, 8);

          // Wait for send to complete
          while (!(usart1.done() && usart2.done()))
            ;

          // Wait for response on at least one bus
          while (true)
          {
            int8_t p1 = usart1_parser.parse(&usart1, registers.system_time);
            if (p1 > 0)
            {
              // Got a packet
              for (uint8_t k = 0; k < read_len; ++k)
              {
                packet[packet_idx++] = usart1_parser.packet.parameters[k];
                packet_chk += usart1_parser.packet.parameters[k];
              }
              break;
            }

            int8_t p2 = usart2_parser.parse(&usart2, registers.system_time);
            if (p2 > 0)
            {
              // Got a packet
              for (uint8_t k = 0; k < read_len; ++k)
              {
                packet[packet_idx++] = usart2_parser.packet.parameters[k];
                packet_chk += usart2_parser.packet.parameters[k];
              }
              break;
            }

            if (p1 < 0 && p2 < 0)
            {
              // Timeout or other error
              for (uint8_t k = 0; k < read_len; ++k)
              {
                packet[packet_idx++] = 0xff;  // 0xffff is not a valid servo position
                packet_chk += 0xff;
              }
              break;
            }
          }  // end while wait for packet
        }  // end for each servo

        packet[packet_idx++] = 255-packet_chk;
        udp_send_packet(packet, packet_idx, port);
      }
      else if (instruction == DYN_WRITE_DATA ||
               instruction == DYN_SYNC_WRITE)
      {
        // Blast packet to each bus
        usart1.write(packet, len+4);
        usart2.write(packet, len+4);

        // Wait for send to complete
        while (!(usart1.done() && usart2.done()))
          ;

        // No need to wait for response
      }
    }

    i += len + 6;
  }

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
  // TODO save/load register table from flash

  // Setup register table data
  registers.model_number = 301;  // Arbotix was 300
  registers.version = 0;
  registers.id = 253;
  registers.baud_rate = 1;  // 1mbps
  registers.digital_dir = 0;  // all in
  registers.digital_out = 0;
  registers.system_time = last_packet = last_motor_cmd = 0;
  registers.motor_period = 10;  // 10mS period = 100hz
  registers.motor_max_step = 10;
  registers.motor1_kp = registers.motor2_kp = 1.0;
  registers.motor1_kd = registers.motor2_kd = 0;
  registers.motor1_ki = registers.motor2_ki = 0.1;
  registers.motor1_windup = registers.motor2_windup = 400;
  registers.usart3_baud = 34;  // 56700
  registers.packets_recv = registers.packets_bad = 0;

  m1_pid.set_max_step(registers.motor_max_step);
  m1_pid.set_gains(registers.motor1_kp, registers.motor1_kd,
                   registers.motor1_ki, registers.motor1_windup);
  m2_pid.set_max_step(registers.motor_max_step);
  m2_pid.set_gains(registers.motor2_kp, registers.motor2_kd,
                   registers.motor2_ki, registers.motor2_windup);

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
  RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
  usart1_tx::mode(GPIO_ALTERNATE | GPIO_AF_USART1);
  usart1_rx::mode(GPIO_ALTERNATE | GPIO_AF_USART1);
  NVIC_SetPriority(USART1_IRQn, 1);
  NVIC_EnableIRQ(USART1_IRQn);
  RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
  usart2_tx::mode(GPIO_ALTERNATE | GPIO_AF_USART2);
  usart2_rx::mode(GPIO_ALTERNATE | GPIO_AF_USART2);
  NVIC_SetPriority(USART2_IRQn, 1);
  NVIC_EnableIRQ(USART2_IRQn);
  set_dynamixel_baud(registers.baud_rate);

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

    if (imu.update(registers.system_time))
    {
      registers.accel_x = imu.accel_data.x;
      registers.accel_y = imu.accel_data.y;
      registers.accel_z = imu.accel_data.z;

      registers.gyro_x = imu.gyro_data.x;
      registers.gyro_y = imu.gyro_data.y;
      registers.gyro_z = imu.gyro_data.z;

      registers.mag_x = imu.mag_data.x;
      registers.mag_y = imu.mag_data.y;
      registers.mag_z = imu.mag_data.z;
    }
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

  // Motor current sense channels
  registers.motor1_current = adc2.get_channel1();
  registers.motor2_current = adc2.get_channel2();

  // Update motors
  if (registers.system_time - last_motor_cmd < 250)
  {
    if (registers.system_time % registers.motor_period == 0)
    {
      // Update table with position/velocity
      registers.motor1_pos = m1_enc.read();
      registers.motor2_pos = m2_enc.read();
      registers.motor1_vel = m1_enc.read_speed();
      registers.motor2_vel = m2_enc.read_speed();

      // Update PID and set motor commands
      m1.set(m1_pid.update_pid(registers.motor1_vel));
      m2.set(m2_pid.update_pid(registers.motor2_vel));
    }
  }
  else
  {
    // Motor commands have timed out
    m1.set(0);
    m2.set(0);
    m1_pid.reset();
    m2_pid.reset();
  }

  // Toggle LED
  if (registers.system_time - last_packet < 500)
  {
    if (registers.system_time % 200 == 0)
      act::high();
    else if (registers.system_time % 100 == 0)
      act::low();
  }
  else
  {
    act::low();
  }

  // Start next conversion of voltage/current
  adc1.convert();
  adc2.convert();
}

// Turn around the rs-485 bus
void USART1_IRQHandler(void)
{
  usart1.usartIrqHandler();
}

// Turn around the ax/mx bus
void USART2_IRQHandler(void)
{
  usart2.usartIrqHandler();
}

}

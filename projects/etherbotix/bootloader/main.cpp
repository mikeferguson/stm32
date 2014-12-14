/*
 * Copyright (c) 2014, Michael E. Ferguson
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

#include "main.h"
#include "etherbotix.hpp"
#include "netconf.h"
#include "lwip/memp.h"
#include "lwip/udp.h"
#include "stm32f4x7_eth.h"
#include "tftpserver.h"

// From IAP app note
pFunction Jump_To_Application;
uint32_t JumpAddress;

// Global status
uint32_t system_time;
uint8_t bootloader_status;

// 512 bytes of metadata in front of actual firmware
typedef struct
{
  uint32_t crc32;   // crc32 of firmware
  uint32_t length;  // length in 32-bit words
  uint8_t version_string[504];
} metadata_t;
#define METADATA_LEN    512

// Check the firmware
uint32_t crc32;
uint8_t check_firmware()
{
  metadata_t * meta = (metadata_t*) USER_FLASH_FIRST_PAGE_ADDRESS;
  uint32_t * firmware = (uint32_t*) (USER_FLASH_FIRST_PAGE_ADDRESS+METADATA_LEN);

  // Check length is reasonable
  if (meta->length > (USER_FLASH_END_ADDRESS - USER_FLASH_FIRST_PAGE_ADDRESS))
    return BOOTLOADER_BAD_LENGTH;

  // Enable & Reset CRC
  RCC->AHB1ENR |= RCC_AHB1ENR_CRCEN;
  CRC->CR = 1;

  // Compute CRC
  // Note: __RBIT funkiness is so that CRC will match standard calculation
  // See http://forum.chibios.org/phpbb/viewtopic.php?f=2&t=1475 for details
  for (uint32_t i = 0; i < meta->length; i++)
    CRC->DR = __RBIT(*(firmware+i));
  crc32 = __RBIT(CRC->DR) ^ 0xFFFFFFFF;

  // Disable CRC
  RCC->AHB1ENR &= ~RCC_AHB1ENR_CRCEN;

  // Check CRC
  if (crc32 != meta->crc32)
    return BOOTLOADER_BAD_CRC32;

  // Firmware is OK
  return 0;
}

int main(void)
{
  NVIC_SetPriorityGrouping(3);
  system_time = 0;
  bootloader_status = 0;

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

  // Setup systick
  SysTick_Config(SystemCoreClock/1000);

  // Is bootloader forced either by external pulldown or
  // by firmware setting to output and low
  force_bootloader::pullup();
  delay_ms(1);
  if (force_bootloader::value() == 0)
  {
    bootloader_status = BOOTLOADER_FORCED;
  }

  // Check firmware status
  if (bootloader_status == 0)
  {
    bootloader_status = check_firmware();
  }

  if (bootloader_status != 0)
  {
    LwIP_Init();
    IAP_tftpd_init();

    __enable_irq();
    error::low();  // Done with setup

    while (true)
    {
      LwIP_Periodic_Handle(system_time);

      // If we have new firmware, check firmware again
      if (bootloader_status & BOOTLOADER_TRY_AGAIN)
      {
        bootloader_status = check_firmware();
        if (bootloader_status == 0)
          break;
      }

      // If we were forced into bootloader, but 10s has passed, try to boot
      if (bootloader_status == BOOTLOADER_FORCED &&
          system_time > 10000)
      {
        bootloader_status = check_firmware();
        if (bootloader_status == 0)
          break;
      }
    }
  }

  // Be sure interrupts disabled
  __disable_irq();

  JumpAddress = *(__IO uint32_t*) (USER_FLASH_FIRST_PAGE_ADDRESS + METADATA_LEN + 4);
  Jump_To_Application = (pFunction) JumpAddress;
  __set_MSP(*(__IO uint32_t*) (USER_FLASH_FIRST_PAGE_ADDRESS + METADATA_LEN));
  Jump_To_Application();
}

extern "C"
{

void SysTick_Handler(void)
{
  ++system_time;
}

}

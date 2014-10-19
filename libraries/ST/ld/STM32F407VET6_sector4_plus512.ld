/*
 * STM32F407VET6 Linker Script
 * Now based on CMSIS 3.01 example
 */

__estack = 0x20020000;

MEMORY
{
  FLASH (rx) : ORIGIN = 0x08010200, LENGTH = 512K
  RAM (xrw)  : ORIGIN = 0x20000000, LENGTH = 128K
}

INCLUDE STM32.ld

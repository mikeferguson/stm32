/*
 * STM32103C8T6 Linker Script
 * Now based on CMSIS 3.01 example
 */

__estack = 0x20005000;

MEMORY
{
  FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 64K
  RAM (xrw)  : ORIGIN = 0x20000000, LENGTH = 20K
}

INCLUDE STM32.ld

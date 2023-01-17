# Motor Dyno

This is the firmware for the motor dyno.

## Features
 * Ethernet connectivity.
 * ADS8684 analog to digital converter (SPI1).
 * Buck/Absorber RS-485 bus (USART2).
 * Absorber speed encoder (TIM3).
 * Alternate encoder input (TIM1).
 * Device Under Test RS-485 bus (USART3).
 * Device Under Test Analog DAC (SPI2).

## Implementation Notes

 * Ethernet callback processes all local packets
 * DMA Usage
   * DMA1, Stream1, Ch4 = USART3_RX
   * DMA1, Stream3, Ch4 = USART3_TX
   * DMA1, Stream5, Ch4 = USART2_RX
   * DMA1, Stream6, Ch4 = USART2_RX
   * DMA2, Stream0, Ch3 = SPI1_RX
   * DMA2, Stream3, Ch3 = SPI1_TX

## Communications Protocol

All commands packets are prefixed with "DYNO".

 * Sxxxx - Set the absorber speed to a floating point command (xxxx).
 * Vxxxx - Set the desired voltage to a floating point command (xxxx).
 * DAxxxx - Send command to analog DUT. Voltage is floating point (xxxx).
 * DBxx - Set the baud rate of RS-485 DUT. "xx" is uint16_t baud rate.
 * D4x123... - Send a command to RS-485 DUT. Length of packet is "x", packet bytes follow.

The return packets are formatted as:

 * Prefixed with "DYNO"
 * The data structure "dyno_data_t". See main.cpp for definition.

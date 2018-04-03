# Etherbotix

This is the firmware for the Etherbotix board.

## Features
 * Accepts a Pololu MD03 as motor driver.
   On-board PID, selectable period and gains.
 * Accepts a Pololu MiniIMU9-v2, v3 or v5 as IMU.
 * Exposes several analog and digital IO, an extra SPI, and USART.
   Will eventually expose several timers, maybe servo outputs.

## LED Usage

The Etherbotix has 2 leds:
 * ACT - green - indicates system activity, blinks at 5hz when packets are
   being properly recieved
 * ERROR - red - similar to the LED on the back of a dynamixel servo, can be
   controlled via register table. ERROR led also flashes when the board is
   in the bootloader.

## User IO

| Pin     | Analog In   | Digital In | Digital Out | SPI    | USART   | TIM9  | TIM12  |
|---------|-------------|------------|-------------|--------|---------|-------|--------|
| A0      | Yes, 12-bit |            | Yes         |        |         |       |        |
| A1      | Yes, 12-bit |            | Yes         | MOSI   |         |       |        |
| A2      | Yes, 12-bit |            | Yes         | MISO   |         |       |        |
| D3      |             | Yes        | Yes         | SCK    | TX      |       |        |
| D4      |             | Yes        | Yes         |        | RX      |       |        |
| D5      |             | Yes        | Yes         |        |         |       | CH2    |
| D6      |             | Yes        | Yes         |        |         | CH1   |        |
| D7      |             | Yes        | Yes         |        |         | CH2   |        |

## Ethernet Protocol

Packets sent over ethernet need the following 4-character magic number
prepended to the packet: '0xffBOT'

After the magic number, any number of valid Dynamixel packets may be
appended. For instance:

  0xff B O T 0xff 0xff 1 5 3 30 0 2 214 0xff 0xff 1 5 3 30 0 2 214

Each Dynamixel packet is put into a queue and processed sequentially, return
packets are sent individually, with the same magic number prepended.

## Extended Device

As with the ArbotiX, there is an extended device at address 253. The standard
parts of the register table are as follows:

    MODEL_NUMBER_L      0       // Model # is 301
    MODEL_NUMBER_H      1
    VERSION             2       // Version is 0
    ID                  3       // Always 253, currently not writable
    BAUD_RATE           4       // Standard values from MX-64T datasheet
    RETURN_DELAY        5
    PRESENT_VOLTAGE     24      // Voltage in 100mV increments
    LED                 25      // Controls error LED

Note that PRESENT_VOLTAGE is usually at address 42 on MX-64T devices. In
addition to the "standard" register table entries above, the Etherbotix
supports a number of additional register addresses defined in etherbotix.hpp.

## Support for Sync Read

This firmware supports the sync_read instruction (0x84) as with the ArbotiX.

## Implementation Notes

 * Ethernet callback processes all local packets
 * DMA Usage
   * DMA1, Stream0, Ch1 = I2C1_RX (IMU)
   * DMA1, Stream1, Ch4 = USART3_RX
   * DMA1, Stream3, Ch4 = USART3_TX
   * DMA1, Stream5, Ch4 = USART2_RX
   * DMA1, Stream6, Ch4 = USART2_RX
   * DMA2, Stream2, Ch4 = USART1_RX
   * DMA2, Stream7, Ch4 = USART1_TX

## Status

This is a work in progress. Overview:
 * Motor current sense is unimplemented (maybe need to sync tim1/8, certainly
   need to trigger adc2 with the PWM)
 * SPI2 support is not implemented
 * Tim12 has only external trigger mode, Tim9 is not implemented

## Networking Notes

I'm connecting through an IOGEAR GWU627 Ethernet-to-Wifi adapter (sometimes).
My wifi is setup for 192.168.0.1/24. I then configure my ethernet adapter for
192.168.0.33/28 when I want to bypass the wifi and directly connect to the
board. The board is always 192.168.0.42/24 (static, no DHCP).

# Tablebot

Apparently I'm building a table bot, based on an Etherbotix.

NOTE: this uses the same bootloader as the Etherbotix board.

## Wiring

| Pin     | Function                          |
|---------|-----------------------------------|
| A0      | Cliff sensor (likely analog)      |
| A1      | Cliff sensor (likely analog)      |
| A2      | Cliff sensor (likely analog)      |
| D3      |                                   |
| D4      | LD06 Lidar RX (USART3_RX) - White |
| D5      | LD06 Lidar PWM (TIM12_CH2) - Blue |
| D6      |                                   |
| D7      | Start button (input)              |

## Implementation Notes

 * Ethernet callback processes all local packets
 * DMA Usage
   * DMA1, Stream0, Ch1 = I2C1_RX (IMU)
   * DMA1, Stream1, Ch4 = USART3_RX (laser)
   * DMA1, Stream5, Ch4 = USART2_RX (dynamixel)
   * DMA1, Stream6, Ch4 = USART2_RX (dynamixel)
 * Motor control:
   * Period is configurable from 1-100mS per timestep
   * Velocity commands/status are in ticks/period
 * Lidar (LD06)
   * Baud is 230400, 8N1
   * PWM is 30khz typical, 40% is about 10hz rotation
   * 4500 samples per second
   * Range is 2cm to 12m

## Mode Selection

 * At startup we do not have a mode selected - red LED is solid
 * Turn the robot onto its side to select phase
   * Left side up = Phase 1
   * Right side up = Phase 2
   * Front side up = Phase 3
 * Once the phase has been detected, red led blinks out the phase
 * Once turned right side up, 3 seconds will elapse and then the
   blue LED starts to blink, laser spins up

## Connecting to the board

    openocd -f interface/stlink-v2.cfg -f target/stm32f4x_stlink.cfg

## Wireless Configuration

The GL-AR300M16 is configured as 192.168.0.99
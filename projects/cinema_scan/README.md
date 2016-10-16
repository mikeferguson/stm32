# Cinema Scan

## Component Information

 * Etherbotix Board w/ Custom Pinout
   * Motor Driver Header
     * Pin 01 - M1_EN - PE10 - 
     * Pin 02 - M1_A - PE8
     * Pin 03 - M1_B - PE11 - Touchscreen X+
     * Pin 04 - M1_PWM - PE9 - Touchscreen Y-
     * Pin 05 - M1_SENSE - PA4 - Analog - Touchscreen Y+
     * Pin 06 - 12V
     * Pin 07 - GND
     * Pin 08 - 5V
     * Pin 09 - GND
     * Pin 10 - 12V
     * Pin 11 - M2_SENSE - PA3 - Analog - Touchscreen X-
     * Pin 12 - M2_PWM - PC6 - USART6_TX - d.bus serial connection
     * Pin 13 - M2_B - PC7 - Screen D/C
     * Pin 14 - M2_A - PA5 - SCK1 - Screen CLK
     * Pin 15 - M2_EN - PC8 - Screen CS
   * Motor1 Encoder Header (Filtering removed, R1 = R2 = 0Ohm, removed R10, R11, C4, C13)
     * Pin 1 - GND - Remove Ground
     * Pin 2 - 5V - N/C
     * Pin 3 - M1_ENC_B - Remove Select (internal pullup, switch connects to ground)
     * Pin 4 - M1_ENC_A - Remove Next (internal pullup, switch connects to ground)
   * Motor2 Encoder Header (Filtering removed, R3 = R4 = 0Ohm, removed R12, R13, C17, C18)
     * Pin 1 - GND - Screen Ground
     * Pin 2 - 5V - Screen Power
     * Pin 3 - M2_ENC_B - MOSI1 - Screen MOSI
     * Pin 4 - M2_ENC_A - MISO1 - Screen MISO
 * SICK TIM571
   * Powered from AUX 12V header (orange terminal blocks)
     * Blue = GND
     * Brown = +12V
   * Aux current sense = Lidar power usage
   * [Manual](https://sick-virginia.data.continum.net/media/dox/3/33/133/Technical_information_TiM55x_TiM56x_TiM57x_Ranging_Laser_Scanner_en_IM0053133.PDF)
   * [Telegram Listings](https://sick-virginia.data.continum.net/media/dox/7/27/927/Technical_information_Telegram_Listing_Ranging_sensors_LMS1xx_LMS5xx_TiM5xx_NAV310_LD_OEM15xx_LD_LRS36xx_en_IM0045927.PDF)
 * Adafruit 2050 Resistive LCD 320x480 [Manual](https://cdn-learn.adafruit.com/downloads/pdf/adafruit-3-5-color-320x480-tft-touchscreen-breakout.pdf)
   * Digikey P/N 1528-1470-ND
   * Pin 1 - Card Detect - N/C
   * Pin 2 - Card CS - N/C
   * Pin 3 - IM0 - N/C
   * Pin 4 - IM1 - N/C
   * Pin 5 - IM2 - N/C (We solder IM2 jumper so that IM2=3.3V, noting we are in SPI mode)
   * Pin 6 - Ground - N/C
   * Pin 7 - X-
   * Pin 8 - Y-
   * Pin 9 - X+
   * Pin 10 - Y+
   * Pin 11 - Lite - N/C
   * Pin 12 - RST - N/C
   * Pin 13 - D/C (SPI data/command select)
   * Pin 14 - CS
   * Pin 15 - MOSI
   * Pin 16 - MISO
   * Pin 17 - CLK
   * Pin 18 - 3.3V out - N/C
   * Pin 19 - 3-5V
   * Pin 20 - GND
 * Dynamixel Servo
   * Connects to servo headers
   * Servo current sense = servo power usage
 * DJI Ronin-M
   * Uses "d.bus", which is an "open" version of Futaba S-Bus
   * Protocol:
     * Inverted 8E2 (signal is big endian, but each byte is little endian)
     * 25 Bytes long, sent every 14ms (analog mode) or 7ms (highspeed mode)
     * One byte = 1 startbit + 8 data bits + 1 parity bit + 2 stopbits
     * Baud rate = 100kbit/sec
     * Highest bit is sent first
     * Each packet:
       * startbyte = 0xF0
       * data 1-22 = channel data, the data is actually split across bytes:
         * ch1 (11bit) = 8 bits from byte1 and 3 bits from byte2
         * ch2 (11bit) = 5 bits from byte2 and 6 bits from byte3
         * ...
         * ch16 (11bit) = ...
       * flags
         * bit7 = digital channel 17
         * bit6 = digital channel 16
         * bit5 = frame lost (equiv. to red LED on receiver)
         * bit4 = failsafe activated
         * bit3 = n/a
         * bit2 = n/a
         * bit1 = n/a
         * bit0 = n/a
       * endbyte = 0x00
   * Protocol References:
     * https://developer.mbed.org/users/Digixx/notebook/futaba-s-bus-controlled-by-mbed/
     * http://forum.arduino.cc/index.php/topic,99708.0.html
 * [D-tap power cable](https://www.amazon.com/gp/product/B00D0HPTRO)

## Network Setup
 * Etherbotix - 192.168.0.42
 * Debugging computer - 192.168.0.2
 * Laser - 192.168.0.1 (default)

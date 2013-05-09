#Eth-bridge Firmware
This is the firmware for the "stm32 eth/xbee/ax/rx bridge" board.

##LED Usage
The bridge has 3 leds:
 * ACT - green - indicates system activity, blinks at 5hz when packets are
   being properly recieved
 * STAT - yellow - indicates status of e-stop, illuminated when e-stop is pressed
 * ERROR - red - similar to the LED on the back of a dynamixel servo, can be
   controlled via register table.

##Ethernet Protocol
Packets sent over ethernet need the following 4-character magic number
prepended to the packet: '0xffBOT'

After the magic number, any number of valid Dynamixel packets may be
appended. For instance:

  0xff B O T 0xff 0xff 1 5 3 30 0 2 214 0xff 0xff 1 5 3 30 0 2 214

Additionally, a packet can be prepended with a 8-bit ID number, allowing
return packets to be easier lined up with sent packets:

  0xff B O T packet# 0xff 0xff 1 5 3 30 0 2 214 packet# 0xff 0xff 1 5 3 30 0 2 214

Each Dynamixel packet is put into a queue and processed sequentially, return packets
are sent individually, with the same magic number attached, and will also carry
the 8-bit ID if included in the original packet sent to the eth_bridge.

##Extended Device
As with the ArbotiX, there is an extended device at address 253. The register table 
is as follows:

    MODEL_NUMBER_L      0       // Model # is 301
    MODEL_NUMBER_H      1
    VERSION             2       // Version is 1
    ID                  3       // Always 253, currently not writable
    BAUD_RATE           4       // Standard values
    RETURN_DELAY        5
    RETURN_LEVEL        16
    ALARM_LED           17
    POWER_ENABLE        24      // 0 if estopped, 1 otherwise
    LED                 25
    PRESENT_VOLTAGE     42
    CURRENT_L           68      // Same as MX-64, 2048 (0x800) when idle
    CURRENT_H           69      // I = 4.5mA * (Current - 2048)

##Support for Sync Read
This firmware supports the sync_read instruction (0x84) as with the ArbotiX.

##XBEE Protocol
Packets sent over the XBEE connection are standard Dynamixel packets. The 
XBEE socket supports all the extended devices and sync read, the same as
the Ethernet protocol.

##Packet Router
Internally, the firmware acts as a router, sending packets to the proper device (ethernet, ax-bus, etc).
This is implemented in router.cpp. Each device has a dev_X_getState() function to determine if it is ready for
more packets, and a dev_X_dispatch(packet_t& p) function to send it a new packet. The router itself has a
generic dispatch function to which all packets should be sent. It will internally route to the proper device,
and will buffer them until that device is ready.

#Status
This firmware is a work-in-progress. Things left to do:
 * Drivers for Voltage/Current Sense
 * Drivers for IMU, including streamer (that bypasses router?)
 * Perhaps a GPS device?
 * Relax all servos on e-stop, and stop passing goal position commands
 * Actually implement XBEE port (UART2), RX port (UART1)
 * Currently, an 8-bit ID is always prepended to each packet, it is 0 if no ID
   was sent. This should be fixed.
 * DMA-read/write for serial port.
 * Implement ability to forward packets through the XBEE (or any device for that matter). For instance:

        ex: 0xff 0xff XBEE_ID outer_len=9 0xff 0xff servo_id inner_len=4 READ read_addr read_len inner_chk outer_chk

   could be recognized as an improper packet (0xff is not a valid instruction) and the XBEE could determine that the
   internal payload of the packet is indeed a valid packet and forward it on.

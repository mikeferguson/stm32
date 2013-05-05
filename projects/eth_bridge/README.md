#Eth-bridge Firmware
This is the firmware for the "stm32 eth/xbee/ax/rx bridge" board.

##Ethernet Protocol
Packets sent over ethernet need the following 4-character magic number
prepended to the packet: '0xffETH'

After the magic number, any number of valid Dynamixel packets may be
appended. For instance:

  0xff E T H 0xff 0xff 1 5 3 30 0 2 214 0xff 0xff 1 5 3 30 0 2 214

Additionally, a packet can be prepended with a 8-bit ID number, allowing
return packets to be easier lined up with sent packets:

  0xff E T H packet# 0xff 0xff 1 5 3 30 0 2 214 packet# 0xff 0xff 1 5 3 30 0 2 214

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
    CURRENT_L           20      // Measured current going to servos, in mA. 
    CURRENT_H           21
    LED                 25

##Support for Sync Read
This firmware supports the sync_read instruction (0x84) as with the ArbotiX.

##XBEE Protocol
Packets sent over the XBEE connection are standard Dynamixel packets. The 
XBEE socket supports all the extended devices and sync read, the same as
the Ethernet protocol.

#Status
This firmware is a work-in-progress. Things left to do:
 * Drivers for Voltage/Current Sense
 * Drivers for IMU
 * DMA-read/write for serial port
 * Relax all servos on e-stop, and stop passing goal position commands
 * Actually implement XBEE port (UART2), RX port (UART1)

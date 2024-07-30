# Eth-bridge Firmware
Temporary firmware for SMALdog v1.5 based on an eth-bridge board. This firmware
will eventually be cleaned up for the new SMALdog v2 controller board.

## LED Usage
The bridge has 3 leds:
 * ACT - blue - indicates system activity, blinks at 5hz when packets are
   being properly recieved
 * STAT - gree - indicates status of e-stop, illuminated when e-stop is pressed
 * ERROR - red - similar to the LED on the back of a dynamixel servo, can be
   controlled via register table. Is also illuminated when the robot is booting.

##Ethernet Protocol
Packets sent over ethernet need the following 4-character magic number
prepended to the packet: 'SMAL'

After the magic number, any number of valid Dynamixel packets may be
appended. For instance:

  S M A L 0xff 0xff 1 5 3 30 0 2 214 0xff 0xff 1 5 3 30 0 2 214

Each packet will be handled one at a time, and each will get it's own
ethernet response packet.

##Extended Device
As with the ArbotiX, there is an extended device at address 253. The register
table is defined in eth_bridge.hpp.

##Support for Sync Read
This firmware does not yet support the sync_read instruction (0x84).

## Networking Notes
I'm connecting through an IOGEAR GWU627 Ethernet-to-Wifi adapter (sometimes).
My wifi is setup for 192.168.0.1/24. I then configure my ethernet adapter
for 192.168.0.33/28 when I want to bypass the wifi and directly connect to the
board. The board is always 192.168.0.42/24 (static, no DHCP).

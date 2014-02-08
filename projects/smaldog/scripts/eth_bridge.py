#!/usr/bin/env python

# Copyright (c) 2008-2013, Michael E. Ferguson
# All right reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above copyright
#     notice, this list of conditions and the following disclaimer in the
#     documentation and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE 
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL 
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

## @file eth_bridge.py Low-level code to control an EthBridge.

from __future__ import print_function
import socket, time
from ax12 import *

## @brief This connects to the stm32 eth/xbee/ax/rx bridge over Ethernet.
class EthBridge:
    magic = '\xffBOT'

    ## @brief Constructs an EthBridge instance and creates the ethernet socket.
    ##
    ## @param ip The IP address of the EthBridge.
    ##
    ## @param port The port number to transmit/recieve from.
    ##
    ## @param timeout How long to wait for packets to come back.
    def __init__(self, ip="192.168.0.42", port = 5048, timeout = 1.0):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind( ("",0) )
        self.sock.setblocking(0)
        self.ip = ip
        self.port = port
        self.timeout = timeout
        self.p_id = 0

    ## @brief Send packet(s) over Ethernet.
    ##
    ## @param packets A list of integers that constitutes the packet(s).
    def send(self, packets):
        msg = self.magic + "".join([chr(m) for m in packets])
        self.sock.sendto(msg, 0, (self.ip, self.port))

    ## @brief Read packet(s) over Ethernet.   
    def recv(self):
        t = time.time()
        while(True):
            if time.time() > t + self.timeout:
                return None
            try:
                data = self.sock.recv(1024)
                # TODO: parse it
                if data[0:4] != self.magic:
                    print("Invalid Header")
                    continue
                return data[4:]
            except:
                pass

    def clear_recv(self):
        try:
            while(True):
                self.sock.recv(1024)
                time.sleep(0.001)
        except:
            pass

    ## @brief Read a dynamixel return packet.
    ##
    ## @param mode This should be 0 to start reading packet. 
    ##
    ## @return The parameters of the packet.
    def getPacket(self):
        data = self.recv()
        if data == None:
            return None
        else:
            # TODO: more error processing/checking here
            return [ord(b) for b in data[6:-1]]

    ## @brief Create a buffer for this packet
    ##
    ## @param index The servo ID.
    ##
    ## @param ins The instruction to send.
    ##
    ## @param params The parameters for the packet.
    ##
    ## @param p_id The packet ID to attach to packet (if any).
    ##
    ## @return A list of integers that can be sent over the port.
    def makePacket(self, index, ins, params, p_id = 0):
        if p_id == 0:
            p_id = self.p_id
        self.p_id += 1
        if self.p_id > 255:
            self.p_id = 0
        length = 2 + len(params)
        checksum = 255 - ((index + length + ins + sum(params))%256)
        return [p_id, 0xff, 0xff, index, length, ins] + params + [checksum, ]

    ## @brief Write values to registers.
    ##
    ## @param index The ID of the servo.
    ##
    ## @param start The starting register address to begin writing to.
    ##
    ## @param values The data to write, in a list.
    ##
    ## @return The error level.
    def write(self, index, start, values):
        self.send(self.makePacket(index, AX_WRITE_DATA, [start] + values))

    ## @brief Read values of registers.
    ##
    ## @param index The ID of the servo.
    ## 
    ## @param start The starting register address to begin the read at.
    ##
    ## @param length The number of bytes to read.
    ##
    ## @return A list of the bytes read, or -1 if failure.
    def read(self, index, start, length):
        self.clear_recv()
        self.send(self.makePacket(index, AX_READ_DATA, [start, length]))
        values = self.getPacket()
        if values == None:
            return -1        
        else:
            return values

    ## @brief Set the status of the LED on a servo.
    ##
    ## @param index The ID of the device to write.
    ##
    ## @param value 0 to turn the LED off, >0 to turn it on
    ##
    ## @return The error level.
    def setLed(self, index, value):
        return self.write(index, P_LED, [value])

    ## @brief Set the position of a servo.
    ##
    ## @param index The ID of the device to write.
    ##
    ## @param value The position to go to in, in servo ticks.
    ##
    ## @return The error level.
    def setPosition(self, index, value):
        return self.write(index, P_GOAL_POSITION_L, [value%256, value>>8])

    ## @brief Set the speed of a servo.
    ##
    ## @param index The ID of the device to write.
    ##
    ## @param value The speed to write.
    ##
    ## @return The error level.
    def setSpeed(self, index, value):
        return self.write(index, P_GOAL_SPEED_L, [value%256, value>>8])

    ## @brief Get the position of a servo.
    ##
    ## @param index The ID of the device to read.
    ##
    ## @return The servo position.
    def getPosition(self, index):
        values = self.read(index, P_PRESENT_POSITION_L, 2)
        try:
            return int(values[0]) + (int(values[1])<<8)
        except:
            return -1

    ## @brief Get the voltage of a device.
    ##
    ## @param index The ID of the device to read.
    ##
    ## @return The voltage, in Volts.
    def getVoltage(self, index):
        try:
            return int(self.read(index, P_PRESENT_VOLTAGE, 1)[0])/10.0
        except:
            return -1

    ## @brief Get the current of a device.
    ##
    ## @param index The ID of the device to read.
    ##
    ## @return The current, in Amps.
    def getCurrent(self, index):
        values = self.read(index, P_CURRENT_L, 2)
        try:
            return (int(values[0]) + (int(values[1])<<8)-2048)*0.0045
        except:
            return -1

if __name__=="__main__":
    eth = EthBridge()

    # blink AX-12 with ID1
    eth.setLed(1,1)
    time.sleep(0.5)
    eth.setLed(1,0)

    # set position, read it
    eth.setPosition(1,512)
    print(eth.getPosition(1))
    print(eth.getVoltage(1))

    # read from eth-bridge itself
    print(eth.getVoltage(253))
    print(eth.getCurrent(253))
    
    eth.recv()


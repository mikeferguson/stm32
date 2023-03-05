#!/usr/bin/env python3

# Copyright (c) 2014-2023, Michael E. Ferguson
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import binascii
import tftpy  # pip3 install tftpy (not in debs)
import time
import socket

# Load File
firmware = open("./build/tablebot.bin", "rb").read()
print("Size of upload", len(firmware))

# Pad firmware to 4-byte aligned
while len(firmware) % 4 != 0:
    firmware += chr(0)

# Compute length (in words)
length = int(len(firmware) / 4)

# Compute crc32
crc32 = binascii.crc32(firmware) & 0xffffffff
print("CRC32", hex(crc32))

# Insert metadata
metadata = [0 for i in range(512)]
metadata[0] = crc32 & 0xff
metadata[1] = (crc32 >> 8) & 0xff
metadata[2] = (crc32 >> 16) & 0xff
metadata[3] = (crc32 >> 24) & 0xff
metadata[4] = length & 0xff
metadata[5] = (length >> 8) & 0xff
metadata[6] = (length >> 16) & 0xff
metadata[7] = (length >> 24) & 0xff
firmware = bytes(metadata) + firmware

# Newer versions of tftpy will allow passing in a StringIO,
# but even Ubuntu Trusty does not have this version :(
open("/tmp/firmware.bin", "wb").write(firmware)

# Reboot the board
conn = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
conn.bind(("", 0))
conn.sendto(b"\xffBOTBOOT", 0, ("192.168.0.42", 6707))
time.sleep(3.0)

# Upload using TFTP, set large timeout
client = tftpy.TftpClient("192.168.0.42", 69)
client.upload("firmware", "/tmp/firmware.bin", timeout=30)

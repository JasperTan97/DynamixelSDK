#!/usr/bin/env python
# -*- coding: utf-8 -*-

################################################################################
# Copyright 2017 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
################################################################################

# Author: Ryu Woon Jung (Leon)

import time
import serial

LATENCY_TIMER = 16
DEFAULT_BAUDRATE = 1000000

class PortHandlerMac(object):
    def __init__(self, port_name):
        self.is_open = False
        self.baudrate = DEFAULT_BAUDRATE
        self.packet_start_time = 0.0
        self.packet_timeout = 0.0
        self.tx_time_per_byte = 0.0

        self.is_using = False
        self.setPortName(port_name)

    def openPort(self):
        return self.setBaudRate(self.baudrate)

    def closePort(self):
        self.ser.close()
        self.is_open = False

    def clearPort(self):
        self.ser.flush()

    def setPortName(self, port_name):
        self.port_name = port_name

    def getPortName(self):
        return self.port_name

    def setBaudRate(self, baudrate):
        baud = self.getCFlagBaud(baudrate)

        if baud <= 0:
            return False
        else:
            self.baudrate = baudrate
            return self.setupPort(baud)

    def getBaudRate(self):
        return self.baudrate

    def getBytesAvailable(self):
        return self.ser.in_waiting

    def readPort(self, length):
        read_bytes = []
        read_bytes.extend([ord(ch) for ch in self.ser.read(length)])
        return read_bytes

    def writePort(self, packet):
        return self.ser.write(packet)

    def setPacketTimeout(self, packet_length):
        self.packet_start_time = self.getCurrentTime()
        self.packet_timeout = (self.tx_time_per_byte * self.packet_length) + (LATENCY_TIMER * 2.0) + 2.0

    def setPacketTimeout(self, msec):
        self.packet_start_time = self.getCurrentTime()
        self.packet_timeout = msec

    def isPacketTimeout(self):
        if self.getTimeSinceStart() > self.packet_timeout:
            self.packet_timeout = 0
            return True

        return False

    def getCurrentTime(self):
        return round(time.time() * 1000000000) / 1000000.0

    def getTimeSinceStart(self):
        time = self.getCurrentTime() - self.packet_start_time
        if time < 0.0:
            packet_start_time = self.getCurrentTime()

        return time

    def setupPort(self, cflag_baud):
        if self.is_open:
            self.closePort()

        self.ser = serial.Serial(
            port = self.port_name,
            baudrate = self.baudrate,
            # parity = serial.PARITY_ODD,
            # stopbits = serial.STOPBITS_TWO,
            bytesize = serial.EIGHTBITS,
            timeout = 0
        )

        self.is_open = True

        self.ser.reset_input_buffer()

        self.tx_time_per_byte = (1000.0 / self.baudrate) * 10.0

        return True

    def getCFlagBaud(self, baudrate):
        if baudrate == 9600:
            return 9600
        elif baudrate == 19200:
            return 19200
        elif baudrate == 38400:
            return 38400
        elif baudrate == 57600:
            return 57600
        elif baudrate == 115200:
            return 115200
        elif baudrate == 230400:
            return 230400
        elif baudrate == 460800:
            return 460800
        elif baudrate == 500000:
            return 500000
        elif baudrate == 576000:
            return 576000
        elif baudrate == 921600:
            return 921600
        elif baudrate == 1000000:
            return 1000000
        elif baudrate == 1152000:
            return 1152000
        elif baudrate == 1500000:
            return 1500000
        elif baudrate == 2000000:
            return 2000000
        elif baudrate == 2500000:
            return 2500000
        elif baudrate == 3000000:
            return 3000000
        elif baudrate == 3500000:
            return 3500000
        elif baudrate == 4000000:
            return 4000000
        else:
            return -1

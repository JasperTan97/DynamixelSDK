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

class PortHandlerWindows(object):
    def __init__(self, port_name):
        self.socket_fd = 0.0
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

    def clearPort(self):
        self.ser.flush()

    def setPortName(self, port_name):
        self.port_name = port_name

    def getPortName(self):
        return self.port_name

    def setBaudRate(self, baudrate):
        baud = self.getCFlagBaud(baudrate)

        self.closePort()

        if baud <= 0:
            setupPort(38400)
            self.baudrate = baudrate
            return setCustomBaudrate(baudrate)
        else:
            self.baudrate = baudrate
            return setupPort(baud)
        
        # TODO:more simplify

    def getBaudRate(self):
        return self.baudrate

    def getBytesAvailable(self):
        pass

        # TODO: fill

    def readPort(self, packet, length):
        pass

        # TODO: fill
    
    def writePort(self, packet):
        ser.write(packet)


# ser.isOpen()

# print 'Enter your commands below.\r\nInsert "exit" to leave the application.'

# input=1
# while 1 :
#     # get keyboard input
#     input = raw_input(">> ")
#         # Python 3 users
#         # input = input(">> ")
#     if input == 'exit':
#         ser.close()
#         exit()
#     else:
#         # send the character to the device
#         # (note that I happend a \r\n carriage return and line feed to the characters - this is requested by my device)
#         ser.write(input)
#         # out = ''
#         # # let's wait one second before reading output (let's give device time to answer)
#         # time.sleep(1)
#         # while ser.inWaiting() > 0:
#         #     out += ser.read(1)

#         # if out != '':
#         #     print ">>" + out




    def setPacketTimeout(self, packet_length):
        self.packet_start_time = self.getCurrentTime()
        self.packet_timeout = (self.tx_time_per_byte * self.packet_length) + (LATENCY_TIMER * 2.0) + 2.0

    def setPacketTimeout(self, msec):
        self.packet_start_time = self.getCurrentTime()
        self.packet_timeout = msec

    def isPacketTimeout(self):
        if self.getTimeSinceStart() > packet_timeout:
            self.packet_timeout = 0
            return True
        
        return False

    def getCurrentTime(self):
        pass

        #TODO : fill

    def getTimeSinceStart(self):
        time = self.getCurrentTime() - self.packet_start_time
        if time < 0.0:
            packet_start_time = self.getCurrentTime()

        return time

    def setupPort(self, cflag_baud):
        
        # TODO: Error exception

        self.ser = serial.Serial(
            port = self.port_name, #'/dev/ttyUSB0',
            baudrate = self.baudrate, # 1000000,
            parity = serial.PARITY_ODD,
            stopbits = serial.STOPBITS_TWO,
            bytesize = serial.SEVENBITS
        )

        self.tx_time_per_byte = (1000.0 / self.baudrate) * 10.0

        return True

    def setCustomBaudrate(self, speed):
        pass

        #TODO: fill

    def getCFlagBaud(self, baudrate):
        if baudrate is 9600:
            return 9600
        elif baudrate is 19200:
            return 19200
        elif baudrate is 38400:
            return 38400
        elif baudrate is 57600:
            return 57600
        elif baudrate is 115200:
            return 115200
        elif baudrate is 230400:
            return 230400
        elif baudrate is 460800:
            return 460800
        elif baudrate is 500000:
            return 500000
        elif baudrate is 576000:
            return 576000
        elif baudrate is 921600:
            return 921600
        elif baudrate is 1000000:
            return 1000000
        elif baudrate is 1152000:
            return 1152000
        elif baudrate is 1500000:
            return 1500000
        elif baudrate is 2000000:
            return 2000000
        elif baudrate is 2500000:
            return 2500000
        elif baudrate is 3000000:
            return 3000000
        elif baudrate is 3500000:
            return 3500000
        elif baudrate is 4000000:
            return 4000000
        else:
            return -1
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

from .robotis_def import *

PARAM_NUM_DATA = 0
PARAM_NUM_ADDRESS = 1
PARAM_NUM_LENGTH = 2

class GroupBulkRead:
    def __init__(self, port, ph):
        self.port = port
        self.ph = ph

        self.last_result = False
        self.is_param_changed = False
        self.param = []
        self.data_dict = {}

        self.clearParam()

    def makeParam(self):
        if not self.data_dict:
            return

        self.param = []

        for id in self.data_dict:
            if self.ph.getProtocolVersion() == 1.0:
                self.param.append(self.data_dict[id][2])                # LEN
                self.param.append(id)                                   # ID
                self.param.append(self.data_dict[id][1])                # ADDR
            else:
                self.param.append(id)                                   # ID
                self.param.append(DXL_LOBYTE(self.data_dict[id][1]))    # ADDR_L
                self.param.append(DXL_HIBYTE(self.data_dict[id][1]))    # ADDR_H
                self.param.append(DXL_LOBYTE(self.data_dict[id][2]))    # LEN_L
                self.param.append(DXL_HIBYTE(self.data_dict[id][2]))    # LEN_H

    def addParam(self, id, start_address, data_length):
        if id in self.data_dict: # id already exist
            return False

        data = [] # [0] * data_length
        self.data_dict[id] = [data, start_address, data_length]

        self.is_param_changed = True
        return True

    def removeParam(self, id):
        if not id in self.data_dict: # NOT exist
            return

        del self.data_dict[id]

        self.is_param_changed = True

    def clearParam(self):
        self.data_dict.clear()
        return

    def txPacket(self):
        if len(self.data_dict.keys()) == 0:
            return COMM_NOT_AVAILABLE

        if self.is_param_changed == True or not self.param:
            self.makeParam()

        if self.ph.getProtocolVersion() == 1.0:
            return self.ph.bulkReadTx(self.port, self.param, len(self.data_dict.keys()) * 3)
        else:
            return self.ph.bulkReadTx(self.port, self.param, len(self.data_dict.keys()) * 5)

    def rxPacket(self):
        self.last_result = False

        result = COMM_RX_FAIL

        if len(self.data_dict.keys()) == 0:
            return COMM_NOT_AVAILABLE

        for id in self.data_dict:
            self.data_dict[id][PARAM_NUM_DATA], result, _ = self.ph.readRx(self.port, id, self.data_dict[id][PARAM_NUM_LENGTH])
            if result != COMM_SUCCESS:
                return result

        if result == COMM_SUCCESS:
            self.last_result = True

        return result

    def txRxPacket(self):
        result = COMM_TX_FAIL

        result = self.txPacket()
        if result != COMM_SUCCESS:
            return result

        return self.rxPacket()

    def isAvailable(self, id, address, data_length):
        if self.last_result == False or not id in self.data_dict:
            return False

        start_addr = self.data_dict[id][PARAM_NUM_ADDRESS]

        if (address < start_addr) or (start_addr + self.data_dict[id][PARAM_NUM_LENGTH] - data_length < address):
            return False

        return True

    def getData(self, id, address, data_length):
        if self.isAvailable(id, address, data_length) == False:
            return 0

        start_addr = self.data_dict[id][PARAM_NUM_ADDRESS]

        if data_length == 1:
            return self.data_dict[id][PARAM_NUM_DATA][address - start_addr]
        elif data_length == 2:
            return DXL_MAKEWORD(self.data_dict[id][PARAM_NUM_DATA][address - start_addr], self.data_dict[id][PARAM_NUM_DATA][address - start_addr + 1])
        elif data_length == 4:
            return DXL_MAKEDWORD(DXL_MAKEWORD(self.data_dict[id][PARAM_NUM_DATA][address - start_addr + 0], self.data_dict[id][PARAM_NUM_DATA][address - start_addr + 1]), DXL_MAKEWORD(self.data_dict[id][PARAM_NUM_DATA][address - start_addr + 2], self.data_dict[id][PARAM_NUM_DATA][address - start_addr + 3]))
        else:
            return 0

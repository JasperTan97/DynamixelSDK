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

class GroupSyncWrite:
    def __init__(self, port, ph, start_address, data_length):
        self.port = port
        self.ph = ph
        self.start_address = start_address
        self.data_length = data_length

        self.is_param_changed = False
        self.param = []
        self.data_dict = {}

        self.clearParam()

    def makeParam(self):
        if not self.data_dict:
            return

        self.param = []

        for id in self.data_dict:
            if not self.data_dict[id]:
                return

            self.param.append(id)
            self.param.extend(self.data_dict[id])

    def addParam(self, id, data):
        if (id in self.data_dict): # id already exist
            return False

        if len(data) > self.data_length: # input data is longer than set
            return False

        self.data_dict[id] = data

        self.is_param_changed = True
        return True

    def removeParam(self, id):
        if not id in self.data_dict: # NOT exist
            return

        del self.data_dict[id]

        self.is_param_changed = True

    def changeParam(self, id, data):
        if not id in self.data_dict: # NOT exist
            return False

        if len(data) > self.data_length: # input data is longer than set
            return False

        self.data_dict[id] = data

        self.is_param_changed = True
        return True

    def clearParam(self):
        self.data_dict.clear()

    def txPacket(self):
        if len(self.data_dict.keys()) == 0:
            return COMM_NOT_AVAILABLE

        if self.is_param_changed == True or not self.param:
            self.makeParam()

        return self.ph.syncWriteTxOnly(self.port, self.start_address, self.data_length, self.param, len(self.data_dict.keys()) * (1 + self.data_length))

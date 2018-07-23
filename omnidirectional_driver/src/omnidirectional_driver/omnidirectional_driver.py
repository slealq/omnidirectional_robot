#!/usr/bin/env python

"""
This file is a generic driver for the Arcos Lab
Omnidirectional Robot.
"""

__author__ = "stuart.leal@ucr.ac.cr (Stuart Leal)"

import serial
import time
import struct
import crc16

BASE_WIDTH = 290 # milimiters
MAX_SPEED = 282.74 # milimeters / second

class omni():

    def __init__(self, port="/dev/ttyACM0"):
        self.port = serial.Serial(port, 115200, timeout=0.01)
        self.temp_values = [];
        self.buff = None;
        self.crc = None;

    def read_checksum(self):
        data = self.port.read(2)
        if len(data)==2:
            crc = (data[0]<<8) | data[1]
            return crc
        return 0

    def write_checksum(self):
        self.port.write(struct.pack('H', self.crc))

    def calculate_checksum(self):
        self.crc = crc16.crc16xmodem(self.buff)

        return self.crc

    def test_connection(self):
        """Test connection"""

    def set_motors(self, x, y, omega):
        """Basic function that will send commands"""

        # flush clean the port
        self.port.flush()

        # write complete command
        self.buff = "m".encode('utf-8')
        self.buff += struct.pack('fff', x, y, omega)
        self.buff += struct.pack('B', 10)
        self.buff += struct.pack('B', 0)

        # write hole command to stm
        self.port.write(self.buff)

        # calculate checksum and write it
        self.calculate_checksum()
        self.write_checksum()

        # receive back crc
        received_crc = self.read_checksum()

        if (received_crc == self.crc):
            return 1
        else :
            return 0

    def read_global_pos(self):
        """Read global pos odometry,
        returns global x pos, global y pos, and global thetha pos"""

        # flush clean the port
        self.port.flush()

        # write o code with end and checksum
        self.buff = "o".encode('utf-8')
        self.buff += struct.pack('B', 10)
        self.buff += struct.pack('B', 0)

        # write instruction
        self.port.write(self.buff)

        # calculate checksum and write it
        self.calculate_checksum()
        self.write_checksum()

        # listen to the checksum that comes back
        received_crc = self.read_checksum()

        # now read the instructions in buff
        self.buff += self.port.read(12)

        # calculate local checksum
        self.calculate_checksum()

        # receive checksum from stm
        received_crc = self.read_checksum()

        if (received_crc == self.crc):
            return list(struct.unpack('fff', self.buff[3:]))

        else:
            return []

    def read_global_vel(self):
        """Read global vel of the robot,
        this should return vel x, vel y, vel theta """

        # flush clean the port
        self.port.flush()

        # write v code
        self.port.write("v".encode('utf-8'))

        # receive the three packs of bytes, containning pos
        self.buff = self.port.read(12)

        # calculate local checksum
        self.calculate_checksum()

        # receive checksum from stm
        received_crc = self.read_checksum()

        if (received_crc == self.crc):
            return list(struct.unpack('fff', self.buff))

        else:
            return []

    def reset_robot(self):
        """ Reset pid values, and global pos """

        # flush clean the port
        self.port.flush()

        # write r code
        self.port.write("r".encode('utf-8'))

        # # reveice the ack code for end
        # input = self.port.read(2)

        # try:
        #     ack_code = input.decode('utf8');
        # except UnicodeDecodeError:
        #     print("reset_robot failed...")
        #     ack_code = ""

        # # check ack code
        # if (ack_code == "13"):
        #     return 1

        # # clean the port if error occurs
        # print(self.port.read(4))

        # # return error if false
        # return 0

        return 1

    def close_connection(self):
        """Close the connection"""
        if(self.port.isOpen()):
            self.port.close()

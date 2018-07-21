#!/usr/bin/env python

"""
This file is a generic driver for the Arcos Lab
Omnidirectional Robot.
"""

__author__ = "stuart.leal@ucr.ac.cr (Stuart Leal)"

import serial
import time
import struct

BASE_WIDTH = 290 # milimiters
MAX_SPEED = 282.74 # milimeters / second

class omni():

    def __init__(self, port="/dev/ttyACM0"):
        self.port = serial.Serial(port, 115200, timeout=0.0002)

        self.temp_values = [];
        self.buff = None;

    def test_connection(self):
        """Test connection"""

    def set_motors(self, x, y, omega):
        """Basic function that will send commands"""

        # flush clean the port
        self.port.flush()

        # write m code for motors
        self.port.write("m".encode('utf-8'))

        # write vels commands in bytes

        self.buff = struct.pack('f', x)
        self.port.write(self.buff)

        self.buff = struct.pack('f', y)
        self.port.write(self.buff)

        self.buff = struct.pack('f', omega)
        self.port.write(self.buff)

        # capture the ack code
        input = self.port.read(2)# capture the ack number which should be 10

        try:
            ack_code = input.decode('utf8');
        except UnicodeDecodeError:
            print("set_motors failed...")
            ack_code = ""

        # if ack is correct
        if (ack_code == "10"):
            return 1

        # return error
        return 0

    def read_global_pos(self):
        """Read global pos odometry,
        returns global x pos, global y pos, and global thetha pos"""

        # flush clean the port
        self.port.flush()

        # write o code
        self.port.write("o".encode('utf-8'))

        # reset temp values
        self.temp_values = []

        # receive the three packs of bytes, containning pos
        for x in range(3):
            # read 4 byes of float number
            input = self.port.read(4)

            # unpack and save values to list
            self.temp_values.append(struct.unpack('f', input)[0])

        # capture the acknowledge code
        input = self.port.read(2)
       #try:
        ack_code = input.decode('utf8');

        input = self.port.read(1)
        print(input)


        #except UnicodeDecodeError:
        #    print("read_global_pos failed...")
        #    ack_code = ""

        if ack_code == "11":
            return self.temp_values

        return []

    def read_global_vel(self):
        """Read global vel of the robot,
        this should return vel x, vel y, vel theta """

        # flush clean the port
        self.port.flush()

        # write v code
        self.port.write("v".encode('utf-8'))

        # reset temp values
        self.temp_values = []

        # receive the three packs of bytes, containning pos
        for x in range(3):
            # read 4 byes of float number
            input = self.port.read(4)

            # unpack and save values to list
            self.temp_values.append(struct.unpack('f', input)[0])

        # capture the acknowledge code
        input = self.port.read(2)


        try:
            ack_code = input.decode('utf8');
        except UnicodeDecodeError:
            print("read_global_vel failed...")
            ack_code = ""

        input = self.port.read(1)
        print(input)

        if ack_code == "12":
            return self.temp_values

        return []

    def reset_robot(self):
        """ Reset pid values, and global pos """

        # flush clean the port
        self.port.flush()

        # write r code
        self.port.write("r".encode('utf-8'))

        # reveice the ack code for end
        input = self.port.read(2)

        try:
            ack_code = input.decode('utf8');
        except UnicodeDecodeError:
            print("reset_robot failed...")
            ack_code = ""

        # check ack code
        if (ack_code == "13"):
            return 1

        # return error if false
        return 0

    def close_connection(self):
        """Close the connection"""
        if(self.port.isOpen()):
            self.port.close()

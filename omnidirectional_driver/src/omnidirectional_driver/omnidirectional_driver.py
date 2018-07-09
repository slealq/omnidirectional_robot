#!/usr/bin/env python

"""
This file is a generic driver for the Arcos Lab
Omnidirectional Robot.
"""

__author__ = "stuart.leal@ucr.ac.cr (Stuart Leal)"

import serial
import time

BASE_WIDTH = 290 # milimiters
MAX_SPEED = 282.74 # milimeters / second

class omni():

    def __init__(self, port="/dev/ttyACM0"):
        self.port = serial.Serial(port, 115200, timeout=1)

    def test_connection(self):
        """Test connection"""

    def set_motors(self, x, y, omega):
        """Basic function that will send commands"""

        # flush clean the port
        self.port.flush()

        # write m code for motos
        self.port.write("m".encode('utf-8'))
        ack_code = self.port.read().decode('utf-8') # this should be 0

        # write vel commands
        self.port.write(str(str(x) + " " + str(y) + " " + str(omega) +"\n").encode('utf-8'))
        ack_code = self.port.readline().decode('utf-8')

    def read_global_pos(self):
        """Read global pos odometry,
        returns global x pos, global y pos, and global thetha pos"""

        # flush clean the port
        self.port.flush()

        # write o code
        self.port.write("o".encode('utf-8'))
        ack_code = self.port.read().decode('utf-8') # this should be 1

        # receive the line
        line = self.port.readline().decode('utf-8')

        # divide the result by spaces
        values = line.split(" ")

        return values

    def close_connection(self):
        """Close the connection"""
        if(self.port.isOpen()):
            self.port.close()

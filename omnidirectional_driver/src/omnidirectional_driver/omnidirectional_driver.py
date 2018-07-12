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

        # write m code for motors
        self.port.write("m".encode('utf-8'))

        # write vel commands
        command = str(str("{0:.4f}".format(x))
                      + " "
                      + str("{0:.4f}".format(y))
                      + " "
                      + str("{0:.4f}".format(omega))
                      + "\r\n").encode('utf-8')

        self.port.write(command)

        # capture the ack code
        input = self.port.read(2)# capture the ack number which should be 10
        ack_code = input.decode('utf-8')

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

        # receive the line
        line = self.port.readline().decode('utf-8')

        # receive the rest
        input = self.port.read(1) # capture \r garbage

        # divide the result by spaces
        values = line.split(" ")

        # ack code is the last value
        ack_code = values[-2]

        # remove the last from the list
        values = values[:-2]

        # check ack code
        if (ack_code == "11"):
            return [float(x) for x in values]

        # if not ack, the send empty list
        return []

    def read_global_vel(self):
        """Read global vel of the robot,
        this should return vel x, vel y, vel theta """

        # flush clean the port
        self.port.flush()

        # write v code
        self.port.write("v".encode('utf-8'))

        # receive the line
        line = self.port.readline().decode('utf-8')

        # receive the rest
        input = self.port.read(1) # capture \r garbage

        # divide the result by spaces
        vels = line.split(" ")

        # take the last as ack code
        ack_code = vels[-2]

        # resize vels
        vels = vels[:-2]

        if (ack_code == "12"):
            return [float(x) for x in vels]

        return []

    def reset_robot(self):
        """ Reset pid values, and global pos """

        # flush clean the port
        self.port.flush()

        # write r code
        self.port.write("r".encode('utf-8'))

        # reveice the ack code for end
        input = self.port.read(2)

        # ack code
        ack_code = input.decode('utf-8')

        # check ack code
        if (ack_code == "13"):
            return 1

        # return error if false
        return 0

    def close_connection(self):
        """Close the connection"""
        if(self.port.isOpen()):
            self.port.close()

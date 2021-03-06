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
        self.port = serial.Serial(port, 500000, timeout=0.02)
        self.port.write_timeout = 0.02
        self.temp_values = [];
        self.buff = None;
        self.crc = None;

    def cleanup(self):
        self.port.flushInput()
        self.port.flushOutput()

    def read_checksum(self):
        input = self.port.read(2)

        if len(input)==2:
            data = struct.unpack('BB', input)
            # print(data)

            crc = (data[0]<<8) | data[1]
            return crc
        return 0

    def write_checksum(self):
        self.write_wtimeout(struct.pack('H', self.crc))

    def write_wtimeout(self, data):
        try:
            self.port.write(data)
        except serial.serialutil.SerialTimeoutException:
            print("\t\t\t\t\t\tException for write")
           # self.write_wtimeout(data)

    def calculate_checksum(self):
        self.crc = crc16.crc16xmodem(self.buff)

        return self.crc

    def test_connection(self):
        """Test connection"""

    def set_motors(self, x, y, omega):
        """Basic function that will send commands"""

        # write complete command
        self.buff = "m".encode('utf-8')
        self.buff += struct.pack('fff', x, y, omega)
        self.buff += struct.pack('B', 10)
        self.buff += struct.pack('B', 0)

        # buffer that is going to be writed
        print("In set motors, writed buff:")
        print(repr("%s" %self.buff))

        # write hole command to stm
        self.write_wtimeout(self.buff)

        # calculate checksum and write it
        self.calculate_checksum()
        self.write_checksum()

        # receive back crc
        received_crc = self.read_checksum()
        print("In set motors, received checksum")
        print(repr("%s" %received_crc))
        print("In set motors, own checksum")
        print(repr("%s" %self.crc))

        if (received_crc == self.crc):
            return 1
        else :
            self.cleanup()
            return 0

    def read_global_pos(self):
        """Read global pos odometry,
        returns global x pos, global y pos, and global thetha pos"""

        # flush clean the port
        #self.port.flush()

        # write o code with end and checksum
        self.buff = "o".encode('utf-8')
        self.buff += struct.pack('B', 10)
        self.buff += struct.pack('B', 0)

        print("In read global pos, sent buff")
        print(repr("%s" %self.buff))

        # write instruction
        self.write_wtimeout(self.buff)

        # calculate checksum and write it
        self.calculate_checksum()
        self.write_checksum()

        # listen to the checksum that comes back
        received_crc = self.read_checksum()

        # print received crc
        print("In read global pos, received crc")
        print(repr("%s" %received_crc))

        print("In read global pos, own crc")
        print(repr("%s" %self.crc))

        # check if it was sent ok, if not break here
        if not (received_crc == self.crc):
            print("Error in pos before hand...")
            self.cleanup()
            return []

        # now read the instructions in buff
        self.buff += self.port.read(12)

        print("In read global pos, read 12 bytes")
        print(repr("%s" %self.buff))

        # calculate local checksum
        self.calculate_checksum()

        print("In read global pos, own checksum")
        print(repr("%s" %self.crc))

        # receive checksum from stm
        received_crc = self.read_checksum()

        print("In read global pos, received crc")
        print(repr("%s" %received_crc))

        if (received_crc == self.crc):
            return list(struct.unpack('fff', self.buff[3:]))

        else:
            self.cleanup()
            return []

    def read_global_vel(self):
        """Read global vel of the robot,
        this should return vel x, vel y, vel theta """

        # flush clean the port
        #self.port.flush()

        # write v code with end and checksum
        self.buff = "v".encode('utf-8')
        self.buff += struct.pack('B', 10)
        self.buff += struct.pack('B', 0)

        # write instruction
        self.write_wtimeout(self.buff)

        # print buff
        print("In read global vel, own buffer")
        print(repr("%s" %self.buff))

        # calculate checksum of sent
        self.calculate_checksum()
        self.write_checksum()

        print("In read global vel, own checksum")
        print(repr("%s" %self.crc))

        # listen to the checksum that comes back
        received_crc = self.read_checksum()

        print("In read global vel, received crc")
        print(repr("%s" %received_crc))

        # check it was sent ok, if not break here
        if not (received_crc == self.crc) :
            print("Error in vel before hand...")
            self.cleanup()
            return []

        # now read the instruction in buff
        self.buff += self.port.read(12)

        # received buff
        print("In read global vel, received buff")
        print(repr("%s" %self.buff))

        # calculate local checksum
        self.calculate_checksum()

        print("In read global vel, own checksum ")
        print(repr("%s" %self.crc))

        # calculate local checksum with hole instruction
        received_crc = self.read_checksum()

        print("In read global vel, received crc")
        print(repr("%s" %received_crc))


        if (received_crc == self.crc):
            return list(struct.unpack('fff', self.buff[3:]))

        else:
            self.cleanup()
            return []

    def read_encoder_vel(self):
        """Read each encoder vel,
        this should return vel motorrl, vel motorfr, vel motorrr, vel motorfl """

        # flush clean the port
        #self.port.flush()

        # write v code with end and checksum
        self.buff = "e".encode('utf-8')
        self.buff += struct.pack('B', 10)
        self.buff += struct.pack('B', 0)

        # write instruction
        self.write_wtimeout(self.buff)

        # print buff
        print("In read encoder vel, own buffer")
        print(repr("%s" %self.buff))

        # calculate checksum of sent
        self.calculate_checksum()
        self.write_checksum()

        print("In read encoder vel, own checksum")
        print(repr("%s" %self.crc))

        # listen to the checksum that comes back
        received_crc = self.read_checksum()

        print("In read encoder vel, received crc")
        print(repr("%s" %received_crc))

        # check it was sent ok, if not break here
        if not (received_crc == self.crc) :
            print("Error in vel before hand...")
            self.cleanup()
            return []

        # now read the instruction in buff
        self.buff += self.port.read(16)

        # received buff
        print("In read encoder vel, received buff")
        print(repr("%s" %self.buff))

        # calculate local checksum
        self.calculate_checksum()

        print("In read encoder vel, own checksum ")
        print(repr("%s" %self.crc))

        # calculate local checksum with hole instruction
        received_crc = self.read_checksum()

        print("In read encoder vel, received crc")
        print(repr("%s" %received_crc))


        if (received_crc == self.crc):
            return list(struct.unpack('ffff', self.buff[3:]))

        else:
            self.cleanup()
            return []

    def read_pid_fails(self):
        """This reads the serial comm amount of fails for each encoder """

        # flush clean the port
        #self.port.flush()

        # write v code with end and checksum
        self.buff = "t".encode('utf-8')
        self.buff += struct.pack('B', 10)
        self.buff += struct.pack('B', 0)

        # write instruction
        self.write_wtimeout(self.buff)

        # print buff
        print("In read pid fails, own buffer")
        print(repr("%s" %self.buff))

        # calculate checksum of sent
        self.calculate_checksum()
        self.write_checksum()

        print("In read pid fails, own checksum")
        print(repr("%s" %self.crc))

        # listen to the checksum that comes back
        received_crc = self.read_checksum()

        print("In read pid fails, received crc")
        print(repr("%s" %received_crc))

        # check it was sent ok, if not break here
        if not (received_crc == self.crc) :
            print("Error in vel before hand...")
            self.cleanup()
            return []

        # now read the instruction in buff
        self.buff += self.port.read(16)

        # received buff
        print("In read pid fails, received buff")
        print(repr("%s" %self.buff))

        # calculate local checksum
        self.calculate_checksum()

        print("In read pid fails, own checksum ")
        print(repr("%s" %self.crc))

        # calculate local checksum with hole instruction
        received_crc = self.read_checksum()

        print("In read pid fails, received crc")
        print(repr("%s" %received_crc))


        if (received_crc == self.crc):
            return list(struct.unpack('ffff', self.buff[3:]))

        else:
            self.cleanup()
            return []

    def read_encoder_ticks(self):
        """This reads the serial comm amount of fails for each encoder """

        # flush clean the port
        #self.port.flush()

        # write v code with end and checksum
        self.buff = "h".encode('utf-8')
        self.buff += struct.pack('B', 10)
        self.buff += struct.pack('B', 0)

        # write instruction
        self.write_wtimeout(self.buff)

        # print buff
        print("In read encoder ticks, own buffer")
        print(repr("%s" %self.buff))

        # calculate checksum of sent
        self.calculate_checksum()
        self.write_checksum()

        print("In read encoder ticks, own checksum")
        print(repr("%s" %self.crc))

        # listen to the checksum that comes back
        received_crc = self.read_checksum()

        print("In read encoder ticks, received crc")
        print(repr("%s" %received_crc))

        # check it was sent ok, if not break here
        if not (received_crc == self.crc) :
            print("Error in vel before hand...")
            self.cleanup()
            return []

        # now read the instruction in buff
        self.buff += self.port.read(32)

        # received buff
        print("In read encoder ticks, received buff")
        print(repr("%s" %self.buff))

        # calculate local checksum
        self.calculate_checksum()

        print("In read encoder ticks, own checksum ")
        print(repr("%s" %self.crc))

        # calculate local checksum with hole instruction
        received_crc = self.read_checksum()

        print("In read encoder ticks, received crc")
        print(repr("%s" %received_crc))


        if (received_crc == self.crc):
            return list(struct.unpack('qqqq', self.buff[3:]))

        else:
            self.cleanup()
            return []

    def reset_robot(self):
        """ Reset pid values, and global pos """

        # flush clean the port
        #self.port.flush()

        # write r code
        self.buff = "r".encode('utf-8')
        self.buff += struct.pack('B', 10)
        self.buff += struct.pack('B', 0)

        # write instruction
        self.write_wtimeout(self.buff)

        # calculate local checksum
        self.calculate_checksum()
        self.write_checksum()

        # listen to the checksum that comes back
        received_crc = self.read_checksum()

        # check if it was ok or not
        if (received_crc == self.crc) :
            return 1

        else:
            self.cleanup()
            return 0

    def close_connection(self):
        """Close the connection"""
        if(self.port.isOpen()):
            self.port.close()

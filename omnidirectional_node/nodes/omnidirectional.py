#!/usr/bin/env python

"""
Ros node for Arcos Lab Omnidirectional robot.
"""

__author__ = "stuart.leal@ucr.ac.cr (Stuart Leal)"

import roslib; roslib.load_manifest("omnidirectional_node")
import rospy

from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.broadcaster import TransformBroadcaster

from omnidirectional_driver.omnidirectional_driver import omni, BASE_WIDTH, MAX_SPEED

class OmnidirectionalNode:

    def __init__(self):
        """ Start up configuration for Omnidirectional Robot. """
        rospy.init_node('omnidirectional')

        # set communication port
        self.port = rospy.get_param("port", "/dev/ttyACM0")
        rospy.loginfo("Using port: %s"%(self.port))

        # use driver for robot operation
        self.robot = omni(self.port)

        # subscribe to cmd vel topic
        rospy.Subscriber("cmd_vel", Twist, self.cmd_vel_cb)

        # set vels list to 0
        self.cmd_vel = [0,0,0]

    def spin(self):
        """ Spin function to call """

        # first time stamp
        then = rospy.Time.now()

        # init the odom message
        odom = Odometry(header=rospy.Header(frame_id="odom"), child_frame_id='base_link')

        # set rate to mantain for scans
        r = rospy.Rate(5)

        while not rospy.is_shutdown():
            # get motor encoder values

            # send updated movement codes
            self.robot.set_motors(self.cmd_vel[0], self.cmd_vel[1], self.cmd_vel[2])

            #

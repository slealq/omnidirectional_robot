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

# import from serial com script

class OmnidirectionalNode:

    def __init__(self):
        """ Start up configuration for Omnidirectional Robot. """
        rospy.init_node('omnidirectional')

        self.port = rospy.get_param("port", "/dev/ttyACM0")
        rospy.loginfo("Using port: %s"%(self.port))


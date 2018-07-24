#!/usr/bin/env python

"""
Ros node for Arcos Lab Omnidirectional robot.
"""

__author__ = "stuart.leal@ucr.ac.cr (Stuart Leal)"

import roslib; roslib.load_manifest("omnidirectional_node")
import rospy
from math import sin,cos

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
        self.odomPub = rospy.Publisher('odom', Odometry, queue_size=10)
        self.odomBroadcaster = TransformBroadcaster()

        # set vels list to 0
        self.cmd_vel = [0,0,0]

    def spin(self):
        """ Spin function to call """

        # first time stamp
        self.x = 0
        self.y = 0
        self.th = 0

        # init the odom message
        odom = Odometry(header=rospy.Header(frame_id="odom"), child_frame_id='base_link')

        # set rate to mantain for scans
        r = rospy.Rate(1000)

        while not rospy.is_shutdown():
            # update then stamp
            current_time = rospy.Time.now()

            # send updated movement codes
            ack_code = self.robot.set_motors(self.cmd_vel[0], self.cmd_vel[1], self.cmd_vel[2])
            if (ack_code == 0) :
                continue

            # update global pos
            pos = []
            pos = self.robot.read_global_pos() # get current pos
            if not pos:
                continue

            # update global vel
            vels = []
            vels = self.robot.read_global_vel() # get current vel
            if not vels:
                continue

            # update x,y,th
            self.x = pos[0]
            self.y = pos[1]
            self.th = pos[2]

            # prepare tf from base_link to odom
            quaternion = Quaternion()
            quaternion.z = sin(self.th/2.0)
            quaternion.w = cos(self.th/2.0)

            # prepare odometry
            odom.header.stamp = current_time
            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.position.z = 0
            odom.pose.pose.orientation = quaternion
            odom.twist.twist.linear.x = vels[0];
            odom.twist.twist.linear.y = vels[1];
            odom.twist.twist.angular.z = vels[2];

            # publish everything
            self.odomBroadcaster.sendTransform( (self.x, self.y, 0),
                                                (quaternion.x, quaternion.y, quaternion.z, quaternion.w),
                                                current_time,
                                                "base_link",
                                                "odom" )
            self.odomPub.publish(odom)

            # wait, then do it again
            r.sleep()

        # shutdown
        self.robot.reset_robot()
        self.robot.close_connection()

    def cmd_vel_cb(self, req):
        self.cmd_vel = [req.linear.x, req.linear.y, req.angular.z]

if __name__ == "__main__":
    robot = OmnidirectionalNode()
    robot.spin()

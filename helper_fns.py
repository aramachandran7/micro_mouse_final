#!/usr/bin/env python3

import rospy
import time
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Vector3


class DriveConstrianed(object):
    """docstring forDriveConstrianed - drive constrained within 2 walls on the sides."""

    def __init__(self, speed=3.0):

        rospy.init_node()
        
        # init global vars
        self.speed = speed
        self.left = None
        self.right = None
        # setup pub sub
        rospy.Subscriber('/scan', LaserScan,self.scan_recieved)
        self.speed_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.scan = []

    def drive(self):
        """
        funciton called in every loop of main
        """
        pass

    def compute_side_distances(self):
        """
        computes distances from sides based on self.scan list
        """

        self.left = self.scan[270]
        self.right = self.scan[90]

    def scan_recieved(self, msg):
        self.scan = msg.ranges

    def drive_main(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown(): # isolate obstacle detection in seperate node
            self.drive()
            rospy.sleep(r)




class HelperObject(object):
    """docstring HelperObject."""

    def __init__(self):
        pass

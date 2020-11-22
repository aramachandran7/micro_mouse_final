#!/usr/bin/env python3
"""
Locally sourced Python functions!! Keep it fresh!
minimize drive error via a function that only has a 'local' understanding.


drive API usage -
driver = DriveStep()
driver.drive(direction_to_drive, current_heading, speed)
"""
import rospy
import time
import math
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

from geometry_msgs.msg import Twist, Vector3
from tf.transformations import euler_from_quaternion


class DriveStep(object):
    """Drive 1 unit in 1 of 4 primary directions - N E S W, at a speed
    Use /odom to traverse forward and backwards
    Use /scan to check left / right spacing / corrections
    assume initial heading is given

    """

    def __init__(self):

        rospy.init_node('DriveStep')

        # odom & distance vars
        self.x_odom = 0.0
        self.y_odom = 0.0
        self.yaw_odom = 0.0
        self.distance_traveled = None
        self.theta_turned = None
        self.odom_start = None



        # lidar vars
        self.scan = []
        self.wall_distances = {
        "left_wall": None,
        "right_wall": None,
        }


        self.state = self.turn

        # setup pub sub
        rospy.Subscriber('/scan', LaserScan,self.scan_recieved)
        rospy.Subscriber('/odom', Odometry,self.odom_recieved)


        self.speed_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # call params
        self.direction_to_drive = None
        self.current_heading = None
        self.speed = None

        # error thresholds and constants
        self.unit_length = .18 # universal 1 meter unit length?
        self.turn_error = .01
        self.path_center = .084
        self.angular_scaler = 0.5

        self.angles = {
            'N':0,
            'E':3*math.pi/2,
            'S':math.pi,
            'W':math.pi/2,
        }

    def turn(self):
        print("Turning...")
        if self.direction_to_drive == self.current_heading:
            return self.drive_forwards
        else:
            # compute angle to turn (closest rotation angle from a to b)
            angle_to_turn = self.angle_diff(self.angles[self.direction_to_drive], self.angles[self.current_heading])
            print("turning: ", angle_to_turn, " degrees")
            motion = Twist()
            motion.angular.z = self.angular_scaler*((angle_to_turn-self.theta_turned)/(math.pi)) # check signs
            motion.linear.x = 0
            self.speed_pub.publish(motion)

            if math.fabs((angle_to_turn-self.theta_turned))<.05: # TODO variablize
                return self.drive_forwards
            else:
                return self.turn

    def drive_forwards(self):
        """
        state for drive forwards

        if both walls are present use both to drive centered
        if only 1 use that and 'wall follow', keep half
        the unit distance between that wall and bot center on both sides
        if no walls, only use the /odom
        """
        print("Driving...")
        if self.wall_distances['left_wall'] is not None and self.wall_distances['right_wall'] is not None:
            skew = (self.wall_distances['left_wall'] - self.wall_distances['right_wall']) /  .048

        elif (self.wall_distances['left_wall']) is not None:
            skew = (self.wall_distances['left_wall'] - self.path_center)/.024

            pass
        elif (self.wall_distances['right_wall']) is not None:
            skew = (self.path_center - self.wall_distances['right_wall'])/.024

        else: # if there's no walls and you're fucked
            print('You have no walls to go off of! Using only odom for movement')
            # compute skew from odom?
            skew = 0 # TODO: FIX THAT SHIT

        motion_twist = Twist()
        motion_twist.linear.x = self.speed # TODO: maybe add proporaitonal control later
        # set angular component of motion based on calculated skew
        motion_twist.angular.z = (skew) * self.angular_scaler

        self.speed_pub.publish(motion_twist)
        print("distance traveled: ", self.distance_traveled, " , self.unit_length: " , self.unit_length)
        if math.fabs(self.unit_length - self.distance_traveled) < .02:
            return self.idle
        else:
            return self.drive_forwards




    def idle(self):
        print("you shouldn't be here. Robot should be idle. ")
        # TODO: publish no movement
        return self.idle



    def compute_side_distances(self):
        """
        computes distances from sides based on self.scan LR averages
        sets self.wall_distances dictionary
        """
        # TODO: compute distance ahead.
        # TODO: compartmentalize this code, also add vars for scan ranges
        if all((i >= .055 and i <= .12) for i in self.scan[88:93]):
            # you have a good left wall!
            self.wall_distances['left_wall'] = sum(self.scan[88:93])/5
        else:
            self.wall_distances['left_wall'] = None

        if all((i >= .055 and i <= .12) for i in self.scan[268:273]):
            # you have a good left wall!
            self.wall_distances['right_wall'] = sum(self.scan[268:273])/5
        else:
            self.wall_distances['right_wall'] = None



    def scan_recieved(self, msg):
        """ callback for /scan"""
        self.scan = msg.ranges
        self.compute_side_distances()


    def odom_recieved(self, msg):
        """ callback for /odom"""
        self.x_odom, self.y_odom, self.yaw_odom = self.convert_pose_to_xy_and_theta(msg.pose.pose)
        # compute distance traveled in the direction you care about.

        if self.direction_to_drive == 'N' or self.direction_to_drive == 'S':
            self.distance_traveled = self.y_odom - self.odom_start[1]
        else:
            self.distance_traveled = self.x_odom - self.odom_start[0]

        # compute theta turned based off original heading.
        self.theta_turned = self.angle_diff(self.yaw_odom, self.odom_start[2])


    def reset_function(self):
        print('reseting')
        self.state = self.turn
        self.odom_start = (self.x_odom, self.y_odom, self.yaw_odom)
        self.theta_turned = 0
        self.distance_traveled = 0

    def drive_main(self, direction_to_drive, current_heading, speed):
        # set global driving conditions based on params
        self.direction_to_drive = direction_to_drive
        self.current_heading = current_heading
        self.speed = speed

        # initial state of operation upon function call is turn
        self.reset_function()


        r = rospy.Rate(10)
        while not rospy.is_shutdown() and self.state != self.idle:
            self.state = self.state()
            print("operating. ")
            # rospy.sleep(r)
        print("completed DriveStep. Idle State. Awaiting future command ...")



    def convert_pose_to_xy_and_theta(self,pose):
        """ Convert pose (geometry_msgs.Pose) to a (x,y,yaw) tuple
        From helper_fns in warmup project
        """
        orientation_tuple = (pose.orientation.x,
                                pose.orientation.y,
                                pose.orientation.z,
                                pose.orientation.w)
        angles = euler_from_quaternion(orientation_tuple)
        return (pose.position.x, pose.position.y, angles[2])

    def angle_diff(self, direction_to_drive, current_heading):
        """
        Calculates the difference between angle a and angle b (both should be in radians)
        Difference based on the closest rotation from angle a to angle b
        examples:
            angle_diff(.1,.2) -> -.1
            angle_diff(.1, 2*math.pi - .1) -> .2
            angle_diff(.1, .2+2*math.pi) -> -.1
        """
        a = direction_to_drive
        b = current_heading
        d1 = a-b
        d2 = 2*math.pi - math.fabs(d1)
        if d1 > 0:
            d2 *= -1.0
        if math.fabs(d1) < math.fabs(d2):
            return d1
        else:
            return d2




if __name__ == '__main__':
    drive = DriveStep()
    drive.drive_main('E', 'E', 0.1)
    drive.drive_main('E', 'N', 0.1)

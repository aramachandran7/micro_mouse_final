#!/usr/bin/env python3
"""
Locally sourced Python functions!! Keep it fresh!
minimize drive error via a function that only has a 'local' understanding.


drive API usage -
driver = DriveStep()
driver.drive(direction_to_drive, speed)

possible directions of movement - 'F, B, L, R'
# odom overshoots how much it turns?

Changes to make to use lidar better:
1) drive_forwards front and back distance correction
2) angle turning, check all 4 direction 5 degree differences, add as
intermediate turn_quality_checking state. Can jump to drive_forwards if no
relevant lidar data available
--> motivation for 2nd change is extra turn quality checking redundancy,
should be fixed if drive_forwards is good.

"""
import rospy
import time
import math
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from numpy import sign

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
        "front_wall": None, 
        }

        self.angle_off = 0.0


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
        self.angular_scaler = 1.0
        self.max_turn_speed = 1.0
        self.angle_increaser = 1.0
        self.distance_threshold = .02
        self.scan_range = 10


        self.angles = {
            'F':0,
            'B':math.pi,
            'L':math.pi/2,
            'R':-math.pi/2,
        }

    def turn(self):
        print("Turning...")
        if self.direction_to_drive == 'F':
            return self.drive_forwards
        else:
            # compute angle to turn (closest rotation angle from a to b)
            angle_to_turn = self.angle_increaser*self.angles[self.direction_to_drive]
            print("turning: ", angle_to_turn, " degrees")
            motion = Twist()
            x = angle_to_turn-self.theta_turned
            motion.angular.z = self.max_turn_speed*2/(1+math.exp((-2)*x))-self.max_turn_speed
            motion.linear.x = 0
            self.speed_pub.publish(motion)

            if math.fabs((angle_to_turn-self.theta_turned))<.005: # TODO variablize
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
        # print("Driving...")
        if self.wall_distances['left_wall'] is not None and self.wall_distances['right_wall'] is not None:
            skew = (self.wall_distances['left_wall'] - self.wall_distances['right_wall']) /  .048

        elif (self.wall_distances['left_wall']) is not None:
            skew = (self.wall_distances['left_wall'] - self.path_center)/.024

            pass
        elif (self.wall_distances['right_wall']) is not None:
            skew = (self.path_center - self.wall_distances['right_wall'])/.024

        else: # if there's no walls and you're fucked
            # print('You have no walls to go off of! Using only odom for movement')
            # compute skew from odom?
            skew = 0 # TODO: FIX THAT SHIT

        motion = Twist()
        motion.linear.x = self.speed # TODO: maybe add proporaitonal control later
        # set angular component of motion based on calculated skew
        motion.angular.z = (skew) * self.angular_scaler

        self.speed_pub.publish(motion)
        # print("distance traveled: ", self.distance_traveled, " , self.unit_length: " , self.unit_length)

        if self.wall_distances["front_wall"] is not None: 
            # optional control logic if there is a front wall to correct off of
            if math.fabs(self.wall_distances["front_wall"] - self.path_center) < self.distance_threshold:
                return self.idle
            else:
                return self.drive_forwards
        else: 
            if math.fabs(self.unit_length - self.distance_traveled) < self.distance_threshold:
                return self.idle
            else:
                return self.drive_forwards


    def idle(self):
        print("you shouldn't be here")


    def compute_angle_off(self, center): 
        scan_angle = 12
        
        # b = sum(self.scan[center+scan_angle-4:center+scan_angle+5]) / 8
        # c = sum(self.scan[center-scan_angle-4:center-scan_angle+5]) / 8
        b = self.scan[center+scan_angle]
        c = self.scan[center-scan_angle]
        a = math.sqrt(b**2+c**2-(2*b*c*math.cos(math.radians(2*scan_angle))))
        B = math.asin((b*math.sin(math.radians(2*scan_angle)))/a)
        # switch to degrees 
        AOA = 180 - (math.degrees(B) + scan_angle)
        angle = (AOA-90) + center
        print("b,c: ", b,c)
        print("angle_off: ", angle)

        # return angle

    def compute_side_distances(self):
        """
        computes distances from sides based on self.scan LR averages
        sets self.wall_distances dictionary
        we should also compute the angle the robot is off,  if there is a wall.

        Do some angles
        """
        scan_angle = 5
        #print("computing side distances")

        if all((i >= .055 and i <= .12) for i in self.scan[90-scan_angle:90+scan_angle+1]):
            # print("you have a good left wall!")
            self.wall_distances['left_wall'] = sum(self.scan[88:93])/5
        else:
            self.wall_distances['left_wall'] = None
        #    print("you shouldn't have a left wall")

        if all((i >= .055 and i <= .12) for i in self.scan[270-scan_angle:270+scan_angle+1]):
            # you have a good left wall!
            self.wall_distances['right_wall'] = sum(self.scan[268:273])
            
        else:
            self.wall_distances['right_wall'] = None

        if all((i >= .055 and i <= .12) for i in (self.scan[-2:] + self.scan[:3])):
            # you have a good front wall! Check values around the center
            self.wall_distances["front_wall"] = sum(self.scan[-2:] + self.scan[:3])/5
        else:
            self.wall_distances['front_wall'] = None

        # compute angle_off
        self.compute_angle_off(90)
        # if self.wall_distances['front_wall'] is not None: 
        #     # compute angle based on front wall 
        #     self.angle_off = self.compute_angle_off(0, scan_angle)
        # elif self.wall_distances['left_wall'] is not None:
        #     # compute angle based on left wall 
        #     self.angle_off = self.compute_angle_off(90, scan_angle)
        # elif self.wall_distances['right_wall'] is not None:
        #     # compute angle based on right wall 
        #     self.angle_off = self.compute_angle_off(270, scan_angle)
        # else: 
        #     print("no angle_off computation possible") 

        


        

    def scan_recieved(self, msg):
        """ callback for /scan"""
        self.scan = msg.ranges
        self.compute_side_distances()

    def odom_recieved(self, msg):
        """ callback for /odom"""
        self.x_odom, self.y_odom, self.yaw_odom = self.convert_pose_to_xy_and_theta(msg.pose.pose)
        # compute distance traveled in the direction you care about.

        # absolute value distance calc  - TODO: Fix
        self.distance_traveled = math.sqrt((self.y_odom - self.odom_start[1])**2 + (self.x_odom - self.odom_start[0])**2)

        # compute theta turned based off original heading.
        self.theta_turned = self.angle_diff(self.yaw_odom, self.odom_start[2])
        # self.theta_turned = self.yaw_odom-self.odom_start[2]



    def reset_function(self, direction_to_drive, speed):
        #print('reseting')
        # set global driving conditions based on params
        self.direction_to_drive = direction_to_drive
        self.speed = speed
        self.state = self.turn
        self.odom_start = (self.x_odom, self.y_odom, self.yaw_odom)
        self.theta_turned = 0
        self.distance_traveled = 0

    def drive_main(self, direction_to_drive, speed):
        # initial state of operation upon function call is turn
        self.reset_function(direction_to_drive, speed)

        while not rospy.is_shutdown() and self.state != self.idle:
            self.state = self.state()
        #    print("operating. ")
            # rospy.sleep(r)
        motion = Twist()
        motion.linear.x = 0
        motion.angular.z = 0
        self.speed_pub.publish(motion)
    #    print("completed DriveStep. Idle State. Awaiting future command ...")


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
    #for i in range(16):
    # drive.drive_main('F', 0.2)
    #    print('units passed: ', i+1)
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        drive.drive_main('F', 0)
    # drive.drive_main('F', 0.1)
    # drive.drive_main('B', 0.2)

    # drive.drive_main('L', 0.1)
    # drive.drive_main('B', 0.1)

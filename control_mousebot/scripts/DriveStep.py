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
import numpy as np    #import sign, array, dot
from statistics import mean

from geometry_msgs.msg import Twist, Vector3, Point
from tf.transformations import euler_from_quaternion
from visualization_msgs.msg import Marker, MarkerArray


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
        self.skew = None

        self.angle_off = 0.0


        self.state = self.turn

        # setup pub sub
        rospy.Subscriber('/scan', LaserScan,self.scan_recieved)
        rospy.Subscriber('/odom', Odometry,self.odom_recieved)
        self.wall_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)


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
        self.scan_angle = 5


        self.angles = {
            'F':0,
            'B':math.pi,
            'L':math.pi/2,
            'R':-math.pi/2,
        }

    def turn(self):
        #print("Turning...")
        if self.direction_to_drive == 'F':
            return self.drive_forwards
        else:
            # compute angle to turn (closest rotation angle from a to b)
            angle_to_turn = self.angle_increaser*self.angles[self.direction_to_drive]
   
            motion = Twist()
            x = angle_to_turn-self.theta_turned
            motion.angular.z = self.max_turn_speed*2/(1+math.exp((-10)*x))-self.max_turn_speed
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
        if self.skew is not None:
            # If we have at least one wall
            if self.skew[0] > 0.001:
                sideskew = self.skew[0] - 0.084
                angle_correction = ((self.skew[1]-90)**3)/1000
            elif self.skew[0] < 0.001:
                sideskew = 0.084 + self.skew[0]
                angle_correction = ((self.skew[1]+90)**3)/1000
            #    print(sideskew)
            drift_correction = (sideskew**3)*10000
            
            
            angvel = angle_correction + drift_correction

        else: # if there's no walls and you're fucked
            # print('You have no walls to go off of! Using only odom for movement')
            # compute skew from odom?
            angvel = 0

        motion = Twist()
        motion.linear.x = self.speed # TODO: maybe add proporaitonal control later
        # set angular component of motion based on calculated skew
        motion.angular.z = angvel

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


    def pol2cart(self, d, theta):
        """helper function for converting cartesian coordinates to polar coordinates"""
        x = d * math.cos(math.radians(theta))
        y = d * math.sin(math.radians(theta))
        return x, y

    def compute_skew(self, center): 
        scan_angle = 15
        x = np.zeros((scan_angle*2))
        y = np.zeros((scan_angle*2))
        for i in range(scan_angle*2):
            theta = center + (i-scan_angle)
            x[i], y[i] = self.pol2cart(self.scan[theta], theta)
        
        x_adj = x - mean(x)
        y_adj = y - mean(y)
        
        B_num = sum(np.multiply(x_adj, y_adj))
        B_den = sum(np.square(x_adj))
        # opposite reciprocal of calculated slope
        angle = np.rad2deg(math.atan2(-B_den, B_num)) + (270-center)    # add  to fix robot perspective
        distance = self.scan[int(angle)] * np.sign(180-center)          # Changing sign to fit direction
        self.publish_vector(distance, angle)

        return distance, angle

    def compute_side_distances(self):
        """
        computes distances from sides based on self.scan LR averages
        sets self.wall_distances dictionary
        we should also compute the angle the robot is off,  if there is a wall.

        Do some angles
        """
        #print("computing side distances")
        scan_angle = 15
        if all((i >= .055 and i <= .12) for i in self.scan[90-scan_angle:90+scan_angle+1]):
            # Follow Left wall for straight path
            self.skew = self.compute_skew(90)
        elif all((i >= .055 and i <= .12) for i in self.scan[270-scan_angle:270+scan_angle+1]):
            # If no Left wall, follow right wall
            self.skew = self.compute_skew(270)
        else:
            # No walls found, cannot correct orientation
            print('No Side Walls Found')
            self.skew = None

        if all((i >= .055 and i <= .12) for i in (self.scan[-2:] + self.scan[:3])):
            # you have a good front wall! Check values around the center
            self.wall_distances["front_wall"] = sum(self.scan[-2:] + self.scan[:3])/5
        else:
            self.wall_distances['front_wall'] = None

        # compute angle_off
        
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

    def publish_vector(self, d, theta):
        marker = Marker()
        marker.header.frame_id = "mousebot"
        marker.type = marker.LINE_STRIP
        marker.action = marker.ADD

        marker.scale.x = 0.01

        marker.color.a = 1.0
        marker.color.b = 1.0

        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0

        start_point = Point(0, 0, 0)
        end_point = Point()
        end_point.x = self.pol2cart(d, theta)[0]
        end_point.y = self.pol2cart(d, theta)[1]
        end_point.z = 0
        marker.points.append(start_point)
        marker.points.append(end_point)

        self.wall_pub.publish(marker)





if __name__ == '__main__':
    drive = DriveStep()
    for i in range(2):
        drive.drive_main('F', 0.2)
    print("turning")
    drive.drive_main('R', 0.2)
    drive.drive_main('R', 0.2)
    drive.drive_main('F', 0.2)
    drive.drive_main('L', 0.2)

    #r = rospy.Rate(10)
    #while not rospy.is_shutdown():
    #    for i in range(14):
    #        drive.drive_main('F', 0.2)
    #    drive.drive_main('R', 0.2)
    
    

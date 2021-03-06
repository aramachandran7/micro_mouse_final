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


in-maze robustness improvements:
loren - turning parallel to wall skew calcualations
adi - drive straight - chanigng front walls scanning distance, look for
keypoints on left and right to adjust translation position

"""
import rospy
import time
import math
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import numpy as np    #import sign, array, dot
from statistics import mean
from scipy import stats
from geometry_msgs.msg import Twist, Vector3, Point
from tf.transformations import euler_from_quaternion
from visualization_msgs.msg import Marker, MarkerArray


class Helper(object):
    def __init__(self):
        pass

    def pol2cart(self, d, theta):
        #helper function for converting cartesian coordinates to polar coordinates

        x = d * math.cos(math.radians(theta))
        y = d * math.sin(math.radians(theta))
        return x, y

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

    def angle_normalize(self, z):
        """ convenience function to map an angle to the range [-pi,pi] """
        return math.atan2(math.sin(z), math.cos(z))

    def angle_diff(self, direction_to_drive, current_heading):
        """
        Calculates the difference between angle a and angle b (both should be in radians)
        Difference based on the closest rotation from angle a to angle b
        examples:
            angle_diff(.1,.2) -> -.1
            angle_diff(.1, 2*math.pi - .1) -> .2
            angle_diff(.1, .2+2*math.pi) -> -.1
        """
        a = self.angle_normalize(direction_to_drive)
        b = self.angle_normalize(current_heading)
        d1 = a-b
        d2 = 2*math.pi - math.fabs(d1)
        if d1 > 0:
            d2 *= -1.0
        if math.fabs(d1) < math.fabs(d2):
            return d1
        else:
            return d2


class DriveStep(object):
    """Drive 1 unit in 1 of 4 primary directions - N E S W, at a speed
    Use /odom to traverse forward and backwards
    Use /scan to check left / right spacing / corrections
    assume initial heading is given

    """

    def __init__(self, pos=(0,0), unit_length=0.18):

        rospy.init_node('DriveStep')

        # helper object
        self.help = Helper()

        # odom & distance vars
        self.x_odom = 0.0
        self.y_odom = 0.0
        self.yaw_odom = 0.0
        self.distance_traveled = None
        self.theta_turned2 = None
        self.odom_start = None

        # lidar variables
        self.scan = []
        self.ls_object = None
        self.walls = {
            "F": False,
            "B": False,
            "R": False,
            "L": False
        }

        self.front_distance = None # distance to approximate 'front wall'
        self.skew = None
        self.angle_skew = None
        self.r_value = 0
        self.state = self.turn

        # setup pub sub
        rospy.Subscriber('/scan', LaserScan,self.scan_recieved)
        rospy.Subscriber('/odom', Odometry,self.odom_recieved)
        self.wall_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
        self.speed_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.turn_pub = rospy.Publisher('/turn_points', LaserScan, queue_size=10)

        # call params
        self.direction_to_drive = None # this is an angle value to turn
        self.speed = None

        # error thresholds and constants
        self.unit_length = unit_length # universal unit length, was .18 TODO: dynamic UL adjusting?
        self.turn_cutoff = .01
        self.path_center = .084
        self.max_turn_speed = None
        self.angle_increaser = 1.0
        self.distance_threshold = .02
        self.scan_angle = 20
        self.wall_precision_thresh = 0.70

        self.neato_pos = np.array((pos[0], pos[1],0)) #
        self.prev_45 = (None, None)
        self.run_distance = 0 # speedrunning straight distances.

        # advanced drive mechanics 
        self.d2pc = 0
        self.to_drive = self.unit_length
        self.turn_radius = .2

    def turn(self):
        #print("Turning...")
        if self.direction_to_drive == 0.0: # if you're straight up forwards?
            return self.drive_forwards
        else:
            # compute angle to turn (closest rotation angle from a to b)
            completion_percent = (0.9)
            if (math.fabs(self.theta_turned2/self.direction_to_drive) < completion_percent) or self.skew == None: # maybe 85
                angle_to_turn = self.direction_to_drive - self.theta_turned2
                self.publish_turn([], 0, odom=True) # indicate to /turn_points that you're using odom
                # print("Using encoder: ", angle_to_turn)
            elif (self.skew is not None):
                angle_to_turn = -self.skew[1]
                #print("angle_to_turn_lidar:", angle_to_turn, "from: ", self.skew[2], "rValue: ", self.r_value)

            motion = Twist()
            motion.angular.z = self.max_turn_speed*2/(1+math.exp(-3*angle_to_turn))-self.max_turn_speed
            motion.linear.x = 0

            if math.fabs(angle_to_turn) < self.turn_cutoff:
                return self.drive_forwards
            else:
                self.speed_pub.publish(motion)
                return self.turn

    def turn_advanced(self): 
        if self.direction_to_drive == 0.0: # if you're straight up forwards?
            return self.drive_forwards
        elif self.direction_to_drive < (3/4* math.pi): 
            # compute angle to turn (closest rotation angle from a to b)
            completion_percent = (0.9)
            if (math.fabs(self.theta_turned2/self.direction_to_drive) < completion_percent) or self.skew == None: # maybe 85
                angle_to_turn = self.direction_to_drive - self.theta_turned2
                self.publish_turn([], 0, odom=True) # indicate to /turn_points that you're using odom
                # print("Using encoder: ", angle_to_turn)
            elif (self.skew is not None):
                angle_to_turn = -self.skew[1]
                #print("angle_to_turn_lidar:", angle_to_turn, "from: ", self.skew[2], "rValue: ", self.r_value)

            motion = Twist()
            motion.angular.z = self.max_turn_speed*2/(1+math.exp(-3*angle_to_turn))-self.max_turn_speed
            motion.linear.x = self.speed/2
            # print("Turning_advanced | a2t: ", angle_to_turn)
            if ((not self.walls['L']) or (not self.walls['R'])) and ((self.unit_length-self.distance_traveled) < .45*self.unit_length): 
                self.d2pc = self.unit_length-self.distance_traveled
                # self.a2pc = self.
                print("-- (T) In the early scan Zone--", self.d2pc)

            if math.fabs(angle_to_turn) < self.turn_cutoff:
                return self.drive_forwards_advanced
            else:
                self.speed_pub.publish(motion)
                return self.turn_advanced
        
        else: 
            # compute angle to turn (closest rotation angle from a to b)
            completion_percent = (0.9)
            if (math.fabs(self.theta_turned2/self.direction_to_drive) < completion_percent) or self.skew == None: # maybe 85
                angle_to_turn = self.direction_to_drive - self.theta_turned2
                self.publish_turn([], 0, odom=True) # indicate to /turn_points that you're using odom
                # print("Using encoder: ", angle_to_turn)
            elif (self.skew is not None):
                angle_to_turn = -self.skew[1]
                #print("angle_to_turn_lidar:", angle_to_turn, "from: ", self.skew[2], "rValue: ", self.r_value)

            motion = Twist()
            motion.angular.z = self.max_turn_speed*2/(1+math.exp(-3*angle_to_turn))-self.max_turn_speed
            motion.linear.x = 0

            if math.fabs(angle_to_turn) < self.turn_cutoff:
                return self.drive_forwards_advanced
            else:
                self.speed_pub.publish(motion)
                return self.turn_advanced

    def drive_forwards_advanced(self): 
        # print("Driving_Advanced...")
        if self.skew != None:
            # If we have at least one wall
            if self.skew[2] == 90:
                sideskew = self.skew[0] - 0.084
            elif self.skew[2] == 270:
                sideskew = 0.084 - self.skew[0]
            else:
                sideskew = 0
            angle_correction = self.max_turn_speed*2/(1+math.exp(10*self.skew[1])) - self.max_turn_speed
            drift_correction = self.max_turn_speed*2/(1+math.exp(-25*sideskew)) - self.max_turn_speed

            angvel = (angle_correction + drift_correction)/2
        else: # if there's no walls and you're fucked
            # print('You have no walls to go off of! Using only odom for movement')
            angvel = 0

        motion = Twist()

        if self.speedrun_flag:
            # print("distance traveled: ", self.distance_traveled)
            # print("current x,y: ", self.x_odom, self.y_odom, "odom_start x,y: ", self.odom_start[0], self.odom_start[1])
            run_units = round(self.run_distance/ self.unit_length)
            units_traveled = self.distance_traveled/self.unit_length
            motion.linear.x = (self.speed)*math.exp(-3*((2*units_traveled/run_units-1)**(4*run_units)))
            print("computed speed: ", motion.linear.x, "odom completion", self.distance_traveled/self.run_distance)
            # set angular component of motion based on calculated skew
            motion.angular.z = angvel
            self.speed_pub.publish(motion)

        else:
            motion.linear.x = self.speed 
            # set angular component of motion based on calculated skew
            motion.angular.z = angvel

            self.speed_pub.publish(motion)
        # print("distance traveled: ", self.distance_traveled, " , self.unit_length: " , self.unit_length)

        # rely on odom for early action
        if ((not self.walls['L']) or (not self.walls['R'])) and ((self.unit_length-self.distance_traveled) < .4*self.unit_length): 
            self.d2pc = self.unit_length-self.distance_traveled
            print("--(D) In the early scan Zone--", self.d2pc)
            
            # return self.idle

        if self.front_distance is not None: # TODO increase front distance range for advanced with boolean 
            # control logic if there is a front wall to correct off of
            if math.fabs(self.front_distance - self.path_center) < self.distance_threshold: # completed unit
                self.d2pc = 0
                return self.idle
            else: 
                return self.drive_forwards_advanced
        else:
            if self.speedrun_flag: # speedrun logic
                if (math.fabs(self.run_distance - self.distance_traveled) < self.distance_threshold) or motion.linear.x < 0.005:
                    return self.idle
                else:
                    return self.drive_forwards_advanced
            else: # original drivestep logic
                if math.fabs(self.to_drive - self.distance_traveled) < self.distance_threshold:
                    return self.idle
                else:
                    return self.drive_forwards_advanced

    def drive_forwards(self):
        """
        state for drive forwards

        if both walls are present use both to drive centered
        if only 1 use that and 'wall follow', keep half
        the unit distance between that wall and bot center on both sides
        if no walls, only use the /odom
        """
        # print("Driving...")
        if self.skew != None:
            # If we have at least one wall
            if self.skew[2] == 90:
                sideskew = self.skew[0] - 0.084
            elif self.skew[2] == 270:
                sideskew = 0.084 - self.skew[0]
            else:
                sideskew = 0
            angle_correction = self.max_turn_speed*2/(1+math.exp(10*self.skew[1])) - self.max_turn_speed
            drift_correction = self.max_turn_speed*2/(1+math.exp(-25*sideskew)) - self.max_turn_speed

            angvel = (angle_correction + drift_correction)/2
        else: # if there's no walls and you're fucked
            # print('You have no walls to go off of! Using only odom for movement')
            angvel = 0

        motion = Twist()

        if self.speedrun_flag:
            # print("distance traveled: ", self.distance_traveled)
            # print("current x,y: ", self.x_odom, self.y_odom, "odom_start x,y: ", self.odom_start[0], self.odom_start[1])
            run_units = round(self.run_distance/ self.unit_length)
            units_traveled = self.distance_traveled/self.unit_length
            motion.linear.x = (self.speed)*math.exp(-3*((2*units_traveled/run_units-1)**(4*run_units)))
            print("computed speed: ", motion.linear.x, "odom completion", self.distance_traveled/self.run_distance)
            # set angular component of motion based on calculated skew
            motion.angular.z = angvel
            self.speed_pub.publish(motion)

        else:
            motion.linear.x = self.speed # TODO: maybe add proporaitonal control later
            # set angular component of motion based on calculated skew
            motion.angular.z = angvel

            self.speed_pub.publish(motion)
        # print("distance traveled: ", self.distance_traveled, " , self.unit_length: " , self.unit_length)

        if self.front_distance is not None:
            # control logic if there is a front wall to correct off of
            if math.fabs(self.front_distance - self.path_center) < self.distance_threshold:
                return self.idle
            else:
                return self.drive_forwards
        else:
            if self.speedrun_flag: # speedrun logic
                if (math.fabs(self.run_distance - self.distance_traveled) < self.distance_threshold) or motion.linear.x < 0.005:
                    return self.idle
                else:
                    return self.drive_forwards
            else: # original drivestep logic
                if math.fabs(self.unit_length - self.distance_traveled) < self.distance_threshold:
                    return self.idle
                else:
                    return self.drive_forwards

    def idle(self):
        # print("you shouldn't be here")
        pass

    def compute_keypoints(self):
        # this will only be called if there is no front wall.
        # return None
        if self.prev_45==(None, None): # handle base case
            # print("first run, lol")
            return None
        if self.distance_traveled is None:
            return None
        # if self.speedrun_flag:
        #
        #     distance_traveled_in_last_unit = self.distance_traveled % self.unit_length
        #     if math.fabs(distance_traveled_in_last_unit/self.unit_length) < .4: # don't compute prematurely
        #         #print("ignoring walls ahead  ", self.prev_45)
        #         return None
        if not self.speedrun_flag and math.fabs(self.distance_traveled/self.unit_length) < .4: # don't compute prematurely
                #print("ignoring walls ahead  ", self.prev_45)
                return None
        curr_45 = self.compute_prev_45()
        # print("checking keypoints")
        if curr_45 != self.prev_45:
            # approximate 'wall ' ahead, you have reached target
            #print("---FOUND 'WALL' AHEAD,--- previous 45, 325: ", self.prev_45, " new 45, 325: ", curr_45)
            return self.path_center

        else:
            #print("---DIDN'T FIND 'WALL' AHEAD--  ", self.prev_45)
            return None

    def compute_skew(self, center):
        scan_angle = self.scan_angle

        x_y = np.zeros((2, scan_angle*2))
        if center == 0:
            range_of_angles = self.scan[0-scan_angle:] + self.scan[:scan_angle+1]
            self.publish_turn(range_of_angles, 0-scan_angle)
            range_of_angles = range_of_angles[::-1]
        else:
            range_of_angles = self.scan[center-scan_angle:center+scan_angle]
            self.publish_turn(range_of_angles, center-scan_angle)


        for i, distance in enumerate(range_of_angles):
            theta = center + (i-scan_angle)
            if 0.055 <= distance <= 0.16:
                x_y[0, i-1], x_y[1, i-1] = self.help.pol2cart(distance, np.deg2rad(theta-center))
            else:
                np.delete(x_y, i-1, 1)

        # paul r value code
        Xmean0 = x_y.T - x_y.T.mean(axis=0)
        covar = Xmean0.T.dot(Xmean0)
        evals, evecs = np.linalg.eig(covar)
        v = evecs[:,0][:,np.newaxis]
        E = Xmean0.dot(v)*v.T - Xmean0
        self.r_value = 1 - np.sum(np.square(E))/np.sum(np.square(Xmean0))

        #print("Regression points", x_y)
        x_adj = x_y[0,:] - np.nanmean(x_y[0,:])
        y_adj = x_y[1,:] - np.nanmean(x_y[1,:])

        B_num = sum(np.multiply(x_adj, y_adj))
        B_den = sum(np.square(x_adj))
        if np.isnan(B_num) or np.isnan(B_den):
            print("SLOPE IS NAN")

        angle = math.atan2(B_num, B_den)
        deg_angle = np.rad2deg(angle)

        distance = self.scan[int(deg_angle + center)]
        self.publish_vector(distance, angle)
        return distance, angle, center

    def set_walls(self):
        scan_angle = self.scan_angle
        # self.walls["F"] = all((i >= .055 and i <= .16) for i in (self.scan[-2:] + self.scan[:3])) # TODO: tune in scanning range for front
        # self.walls["B"] = all((i >= .055 and i <= .12) for i in self.scan[180-scan_angle:180+scan_angle+1])
        # self.walls["L"] = all((i >= .055 and i <= .12) for i in self.scan[90-scan_angle:90+scan_angle+1])
        # self.walls["R"] = all((i >= .055 and i <= .12) for i in self.scan[270-scan_angle:270+scan_angle+1])
        #
        # print("scanF: ", self.scan[-15:] + self.scan[:15])
        # print("scanR: ", self.scan[255:286])
        btm_range = .055
        slices = []
        for key in self.walls.keys():
            top_range = 0.12
            if key == "F":
                top_range = 0.16
                offset = 12
                slices = self.scan[offset:offset+scan_angle+1] + self.scan[len(self.scan)-offset-scan_angle:len(self.scan)-offset]
                # slices = self.scan[0-scan_angle:] + self.scan[:scan_angle+1]
            elif key == "B":
                slices = (self.scan[180-scan_angle:180+scan_angle+1])
            elif key == "L":
                slices = (self.scan[90-scan_angle:90+scan_angle+1])
            elif key == "R":
                slices = (self.scan[270-scan_angle:270+scan_angle+1])

            total_in_range = 0
            for i in slices:
                if btm_range <= i <= top_range:
                    total_in_range += 1
            # if key =="F":
            #     print("total_in_range F: ", total_in_range, " slices: ", slices)
            self.walls[key] = (total_in_range/len(slices) > self.wall_precision_thresh) if len(slices)!=0 else False

    def compute_prev_45(self):
        # grab scan data and return True for wall @ 45 / 315 and false for no wall at ~45's
        # for i in self.scan[315-1:315+1+1]:
        #     print(i)
        return (all((i >= .055 and i <= .15) for i in self.scan[45-1:45+1+1]),
                all((i >= .055 and i <= .15) for i in self.scan[315-1:315+1+1])
         )

    def scan_recieved(self, msg):
        """ callback for /scan -- > computes self.walls, self.front_distance, self.skew, self.prev_45"""
        self.scan = msg.ranges
        self.ls_object = msg # storing the laser scan object
        self.set_walls()

        if self.walls['L']:
            self.skew = self.compute_skew(90)
        elif self.walls['R']:
            self.skew = self.compute_skew(270)
        elif self.walls['B']:
            self.skew = self.compute_skew(180)
        else:
            self.skew = None

        if self.walls["F"]:
            # you have a good front wall! Check values around the center
            self.front_distance = self.compute_skew(0)[0] # get distance via linear regression
        elif not self.speedrun_flag:
            # set front wall distance based on keypoint measurment
            self.front_distance = self.compute_keypoints() # called (and reset) on every lidar scan
        else:
            # while speedrunning, begin computing keypoints if you're in the final square of your speedrun
            # if math.fabs(self.run_distance - self.distance_traveled)<1.5*self.unit_length:
            if math.fabs(self.distance_traveled/self.run_distance)>(1-(.6*self.unit_length/self.run_distance)):

                self.front_distance = self.compute_keypoints()
            else:
                self.front_distance = None

        self.prev_45 = self.compute_prev_45() # set 45 degree angles

    def odom_recieved(self, msg):
        """ callback for /odom"""
        self.x_odom, self.y_odom, self.yaw_odom = self.help.convert_pose_to_xy_and_theta(msg.pose.pose)
        #print("getting odom", self.x_odom, self.y_odom)
        if self.odom_start is not None: # compute distance traveled in the direction you care about.
            self.distance_traveled = math.sqrt((self.y_odom - self.odom_start[1])**2 + (self.x_odom - self.odom_start[0])**2)
            # print("getting odom, dt: ", self.distance_traveled)
            # compute theta turned based off original heading.
            self.theta_turned2 = self.help.angle_diff(self.yaw_odom, self.odom_start[2])
            #print("theta_turned2: ", self.theta_turned2)

    def reset(self, direction_to_drive, speed, run_distance=1):
        # set global driving conditions based on params
        self.direction_to_drive = direction_to_drive
        self.run_distance = run_distance
        self.speed = speed
        self.state = self.turn
        self.odom_start = (self.x_odom, self.y_odom, self.yaw_odom)
        self.theta_turned2 = 0
        self.distance_traveled = 0
        self.max_turn_speed = 4.0

    def compute_direction(self, future_pos):
        """
        compute direction_to_drive and compute new self.neato_pos
        i.e.:
        future_pos = 1,4
        neato_pos = 1,3,
        """

        heading_vector = np.array(future_pos)-np.array((self.neato_pos[0], self.neato_pos[1]))
        angle_to_turn = self.help.angle_diff(-math.atan2(heading_vector[0], heading_vector[1]), self.neato_pos[2])

        npos = self.help.angle_normalize(self.neato_pos[2]+angle_to_turn)

        if self.speedrun_flag:
            run_distance = math.sqrt((future_pos[1]-self.neato_pos[1])**2 + (future_pos[0]-self.neato_pos[0])**2) * self.unit_length
            print("run_distance:   ", run_distance)

        self.neato_pos =  np.array((future_pos[0], future_pos[1], npos)) # %math.pi

        print("turning:  ", angle_to_turn)
        if self.speedrun_flag:
            return angle_to_turn, run_distance
        else:
            return angle_to_turn

    def return_walls(self, first=False):
        """ returns walls list [] global F B L R """
        #print("self.neato_orient", self.neato_pos[2]/math.pi)
        walls = []
        Angular_error = 0.1
        if first:
            self.set_walls()
            # always forwards
            # walls = [self.walls["F"]=False, True, self.walls["L"]=False, self.walls["R"]=True]
            walls = [False, True, True, True] #TODO: (6,1)
            # walls = [False, self.walls["F"], self.walls["R"], self.walls["L"]]
            return walls

        else:
            # must return walls (F B L R) in Bools
            if 0.0-Angular_error < self.neato_pos[2] < 0.0+Angular_error:
                print("forwards")          # Facing Forwards
                walls = [self.walls["F"], False, self.walls["L"], self.walls["R"]]
            elif -math.pi/2-Angular_error < self.neato_pos[2] < -math.pi/2+Angular_error:
                print("right")   # Facing Right
                walls = [self.walls["L"], self.walls["R"], False, self.walls["F"]]
            elif math.pi-Angular_error < math.fabs(self.neato_pos[2]) < math.pi+Angular_error:
                print("back") # Facing Backwards, with some play
                walls = [False, self.walls["F"], self.walls["R"], self.walls["L"]]
            elif math.pi/2-Angular_error < self.neato_pos[2] < math.pi/2+Angular_error:
                print("left")   # Facing Left
                walls = [self.walls["R"], self.walls["L"], self.walls["F"], False]
        return walls


    def drive_advanced(self, future_pos, speed): 
        # initial state of operation upon function call is turn
        self.speedrun_flag = False
        direction_to_drive = self.compute_direction(future_pos)
        self.reset(direction_to_drive, speed)

        # more reset functionality 
        self.state = self.turn_advanced # start off with a different state machine + flow
        self.to_drive = self.unit_length + self.d2pc

        while not rospy.is_shutdown() and self.state != self.idle:
            self.state = self.state()

        # setting final speed
        motion = Twist()
        motion.linear.x = 0
        motion.angular.z = 0
        self.speed_pub.publish(motion)
        walls = self.return_walls()
        return (walls) 


    def drive(self, future_pos, speed):
        # initial state of operation upon function call is turn
        self.speedrun_flag = False
        direction_to_drive = self.compute_direction(future_pos)
        self.reset(direction_to_drive, speed)

        while not rospy.is_shutdown() and self.state != self.idle:
            self.state = self.state()

        # setting final speed
        motion = Twist()
        motion.linear.x = 0
        motion.angular.z = 0
        self.speed_pub.publish(motion)
        walls = self.return_walls()
        return walls

    def drive_speedrun(self, future_pos, speed):
        # initial state of operation upon function call is turn
        self.speedrun_flag = True
        direction_to_drive, run_distance = self.compute_direction(future_pos)
        print('direction_to_drive: ', direction_to_drive)
        self.reset(direction_to_drive, speed, run_distance=run_distance)

        while not rospy.is_shutdown() and self.state != self.idle:
            self.state = self.state()

        # setting final speed
        motion = Twist()
        motion.linear.x = 0
        motion.angular.z = 0
        self.speed_pub.publish(motion)

    def publish_turn(self, vals, starting_index, odom=False):
        if odom: # indicate to /turn_points that you're using odom, publishes empty
            points_to_publish = [float("Inf")]*len(self.scan)
            self.ls_object.ranges= points_to_publish
            self.turn_pub.publish(self.ls_object) # TODO: need to copy?
            return

        points_to_publish = [float("Inf")]*len(self.scan)
        pointer = starting_index
        if len(vals) + starting_index > len(self.scan):
            # dealing with center
            for i in vals:
                if pointer > len(points_to_publish)-1:
                    pointer = 0
                points_to_publish[pointer] = i
                pointer += 1
        else:
            for i in vals:
                points_to_publish[pointer] = i
                pointer += 1

        self.ls_object.ranges = points_to_publish
        self.turn_pub.publish(self.ls_object) # TODO: need to copy?

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
        end_point.x = self.help.pol2cart(d, theta)[0]
        end_point.y = self.help.pol2cart(d, theta)[1]
        end_point.z = 0
        marker.points.append(start_point)
        marker.points.append(end_point)

        self.wall_pub.publish(marker)


if __name__ == '__main__':
    drive = DriveStep()
#    for i in range(2):
#        drive.drive('F', 0.2)
#    print("turning")
#    drive.drive('R', 0.2)
#    drive.drive('R', 0.2)
    drive.drive((0,1), 0.2)

    drive.drive((0,0), 0.2)
    drive.drive((0,1), 0.2)
    drive.drive((0,0), 0.2)
    drive.drive((0,1), 0.2)
    drive.drive((0,0), 0.2)
    drive.drive((0,1), 0.2)
    drive.drive((0,0), 0.2)

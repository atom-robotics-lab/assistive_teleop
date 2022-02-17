#! /usr/bin/env python3

# import ros stuff
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf import transformations
from tf.transformations import euler_from_quaternion

import numpy as np
import actionlib
import roslib
from assistive_teleop.msg import AvoidObstacleAction, AvoidObstacleFeedback, AvoidObstacleResult

import math

class Object_Avoider:

    def __init__(self):
        self.pub_ = None
        self.regions = {
            'right': 0,
            'fright': 0,
            'front': 0,
            'fleft': 0,
            'left': 0,
        }
        self.pose =[]
        self.state_ = 0
        self.state_dict_ = {
            0: 'find the wall',
            1: 'turn left',
            2: 'follow the wall',
        }
        self.message = Twist()
        self.d = 1.0
        self.footprint = 1.0
        rospy.init_node('Object_Avoider')
        self.pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)        
        self.sub = rospy.Subscriber('/laser/scan', LaserScan, self.clbk_laser)
        rospy.wait_for_message("/laser/scan", LaserScan)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)

        self.clearance_angle = self.angle_calculate()
        self.server = actionlib.SimpleActionServer('Avoid_Obstacle_server', AvoidObstacleAction, self.execute, False)
        self.server.start()
        print('**** Obstacle Avoider action server starting ****')



    def angle_calculate(self): #for calculating a threshold angle.
        pi = 3.1415926
        theta = round((math.atan((self.footprint/2)/self.d) * 180)/pi)
        return theta


    def angle_checker(self, phi):    #returns True or False depending on whether an obstacle is in path, takes angle as an input.
        pi = 3.1415926 
        phi = round(phi - (self.pose[2]*180/pi))
        print("Angle to Goal: ", phi)

        if phi + self.clearance_angle > 360:
            angle = phi + self.clearance_angle - 360
            print(" 1 laser value: ", min(self.all_regions[0:phi+self.clearance_angle]+self.all_regions[angle:360]), "angle: ", phi+self.clearance_angle)
            if min(min(self.all_regions[0:angle]+self.all_regions[phi-self.clearance_angle:360]), 10) < 10:
                print("True")
                return True
            else:
                print("False, 1")
                return False

        elif phi - self.clearance_angle < 0:
            angle = 360 -(abs(phi - self.clearance_angle))
            print(" 2 laser value: ", min(self.all_regions[0:phi+self.clearance_angle]+self.all_regions[angle:360]), "angle: ", phi+self.clearance_angle)
            if min(min(self.all_regions[0:phi+self.clearance_angle]+self.all_regions[angle:360]), 10) < 10:
                print("True")
                return True
            else:
                print("False, 2")
                return False

        else:
            print(" 3 laser value: ", min(self.all_regions[phi-self.clearance_angle:phi+self.clearance_angle]), "angle: ", phi+self.clearance_angle)
            if min(min(self.all_regions[phi-self.clearance_angle:phi+self.clearance_angle]), 10) < 10:
                print("True")
                return True
            else:
                print("False, 3")
                return False


    def odom_callback(self,data):
      x = data.pose.pose.orientation.x
      y = data.pose.pose.orientation.y
      z = data.pose.pose.orientation.z
      w = data.pose.pose.orientation.w
      self.pose = [data.pose.pose.position.x, data.pose.pose.position.y, euler_from_quaternion([x,y,z,w])[2]]

    def clbk_laser(self, msg):
        self.regions = {
            'right':  min(min(msg.ranges[235:285]), 10), 
            'fright': min(min(msg.ranges[286:335]), 10), 
            'front':  min(min(msg.ranges[0:24] + msg.ranges[336:359]), 10), 
            'fleft':  min(min(msg.ranges[25:75]), 10), 
            'left':   min(min(msg.ranges[76:125]), 10),
        }
        self.all_regions = msg.ranges
        #self.take_action()

    def change_state(self, state):
        if state is not self.state_:
            print ('Wall follower - [%s] - %s' % (state, self.state_dict_[state]))
            self.state_ = state

    def move(self,linear,angular):
        velocity_msg = Twist()
        velocity_msg.linear.x = linear
        velocity_msg.angular.z = angular 
        self.pub_.publish(velocity_msg)

    def take_action(self):
        state_description = ''
        
        if self.regions['front'] > self.d and self.regions['fleft'] > self.d and self.regions['fright'] > self.d:
            state_description = 'case 1 - nothing'
            self.change_state(0)
        elif self.regions['front'] < self.d and self.regions['fleft'] > self.d and self.regions['fright'] > self.d:
            state_description = 'case 2 - front'
            self.change_state(1)
        elif self.regions['front'] > self.d and self.regions['fleft'] > self.d and self.regions['fright'] < self.d:
            state_description = 'case 3 - fright'
            self.change_state(2)
        elif self.regions['front'] > self.d and self.regions['fleft'] < self.d and self.regions['fright'] > self.d:
            state_description = 'case 4 - fleft'
            self.change_state(0)
        elif self.regions['front'] < self.d and self.regions['fleft'] > self.d and self.regions['fright'] < self.d:
            state_description = 'case 5 - front and fright'
            self.change_state(1)
        elif self.regions['front'] < self.d and self.regions['fleft'] < self.d and self.regions['fright'] > self.d:
            state_description = 'case 6 - front and fleft'
            self.change_state(1)
        elif self.regions['front'] < self.d and self.regions['fleft'] < self.d and self.regions['fright'] < self.d:
            state_description = 'case 7 - front and fleft and fright'
            self.change_state(1)
        elif self.regions['front'] > self.d and self.regions['fleft'] < self.d and self.regions['fright'] < self.d:
            state_description = 'case 8 - fleft and fright'
            self.change_state(0)
        else:
            state_description = 'unknown case'
            rospy.loginfo(self.regions)

    def find_wall(self):
        self.move(0.2, -0.3)

    def turn_left(self):
        self.move(0, 0.3)

    def follow_the_wall(self):
        self.move(0.5, 0)

    def distance_shorter(self):
        dist1 = sqrt( (x2 - x1)**2 + (y2 - y1)**2 )
        dist2 = sqrt( (x2 - x1)**2 + (y2 - y1)**2 )

    def AvoidObstacle(self):
        pi = 3.1415926
        self.result = AvoidObstacleResult()
        self.theta = int(round(np.arctan((self.goal[1] - self.pose[1])/(self.goal[0] - self.pose[0]))*180/pi))

        print(self.theta)
        rate = rospy.Rate(15)
        count = 0
        print("Avoiding Obstacle")
        while self.angle_checker(self.theta):
            self.take_action()
            if self.state_ == 0:
                self.find_wall()
            elif self.state_ == 1:
                self.turn_left()
            elif self.state_ == 2:
                self.follow_the_wall()
            else:
                rospy.logerr('Unknown state!')

            self.theta = int(round(np.arctan((self.goal[1] - self.pose[1])/(self.goal[0] - self.pose[0]))*180/pi))           
            rate.sleep()

        print("angle_checker: ", self.angle_checker(self.theta), "left: ", self.regions['left'], "fleft: ", self.regions['fleft'])
        print("state:", self.state_)
        print("----Obstacle Avoided----")
        self.send_feedback(False)
        self.result.obstacle_clearance = True
        self.server.set_succeeded(self.result)


    def check_obstacle(self):
        if self.regions['front'] < self.d:
            return True
        else:
            return False

    def send_feedback(self, value):
        self.feedback.obstacle_presence = value
        self.server.publish_feedback(self.feedback)


    def execute(self, goal):
        print("self.goal")
        self.goal = goal.pose
        print(self.goal)
        rate = rospy.Rate(10)
        self.feedback = AvoidObstacleFeedback()

        while not self.check_obstacle():
            print("No Obstacle Detected")
            self.send_feedback(False)

            rate.sleep()
        print("!!! Obstacle Detected !!!")
        self.send_feedback(True)

        self.AvoidObstacle()


if __name__ == '__main__':
    obs_avoider = Object_Avoider()
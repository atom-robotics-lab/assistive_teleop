#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math

class obstacle_avoider:

    def __init__(self):

        self.regions = {
            'right': 0,
            'fright': 0, 
            'front': 0, 
            'fleft': 0, 
            'left': 0, 
            }
        rospy.init_node('obstacle_avoider')
        rospy.Subscriber('/laser/scan', LaserScan, self.laser_callback)
        self.dist = 0.7
        self.wall_dist = 0.5
        self.avoid = False
        self.message = Twist()
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    def turn(self, z):
        self.move(0.2 , z)

    def laser_callback(self, msg): 
        self.regions = { 
            'right':  min(min(msg.ranges[235:285]), 10), 
            'fright': min(min(msg.ranges[286:335]), 10), 
            'front':  min(min(msg.ranges[0:24] + msg.ranges[336:359]), 10), 
            'fleft':  min(min(msg.ranges[25:75]), 10), 
            'left':   min(min(msg.ranges[76:125]), 10), 
            } 
        #print(self.regions)

    def obstacle_avoid(self):

        while self.avoid != True:
            if self.regions['fright'] > self.dist and self.regions['front'] > self.dist and self.regions['fleft'] > self.dist: #no wall detected
                print("Wall Avoided")
                self.avoid = True
            elif self.regions['fright'] < self.dist and self.regions['front'] > self.dist and self.regions['fleft'] > self.dist: #wall on front-right
                print("Turning Left")
                self.turn(math.pi/4)
                self.avoid = False
            elif self.regions['fright'] > self.dist and self.regions['front'] < self.dist and self.regions['fleft'] > self.dist: #wall in front
                print("Turning left")
                self.turn(math.pi/2)
                self.avoid = False
            elif self.regions['fright'] > self.dist and self.regions['front'] > self.dist and self.regions['fleft'] < self.dist: #wall on front-left
                print("Turning right")
                self.turn(-math.pi/4)
                self.avoid = False
            elif self.regions['fright'] < self.dist or self.regions['front'] < self.dist or self.regions['fleft'] > self.dist: #wall either in front and front-right
                print("Turning left")
                self.turn(math.pi/4)
                self.avoid = False
            elif self.regions['fright'] > self.dist or self.regions['front'] < self.dist or self.regions['fleft'] < self.dist: #wall either in front and front-left
                print("Turning right")
                self.turn(-math.pi/4)
                self.avoid = False   
        
        if self.avoid == True:
            #call the goto function as the wall is avoided
            print("calling the goto function now")

    def check_obstacle(self):
        print("front = " + str(self.regions['front']))
        print("fleft = " + str(self.regions['fleft']))
        print("fright = " + str(self.regions['fright']))
        if self.regions['front'] < self.wall_dist or self.regions['fleft'] < self.wall_dist or self.regions['fright'] < self.wall_dist:
            print("obstacle detected")
            self.move(0.0, 0.0)
            self.obstacle_avoid()

    def demo_controller(self):
        self.rate = rospy.Rate(15)

        while not rospy.is_shutdown():
            self.move(0.3, 0.0)
            self.check_obstacle()
            self.rate.sleep()

    def move(self, linear, angular):
        self.message.linear.x = linear
        self.message.angular.z = angular
        self.pub.publish(self.message)

if __name__ == "__main__":
    avoider = obstacle_avoider()
    avoider.demo_controller()
    #rospy.spin()

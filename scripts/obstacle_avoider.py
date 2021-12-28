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
        self.dist = 0.5
        self.wall_dist = 0.3
        self.avoid = False

    def turn(self, z):
        self.message
        self.message.linear.x = 0.5
        self.message.angular.z = z
        self.pub.publish(self.message)

    def laser_callback(self, msg): 
        self.regions = { 
            'right':  min(min(msg.ranges[0:143]), 10), 
            'fright': min(min(msg.ranges[144:287]), 10), 
            'front':  min(min(msg.ranges[288:431]), 10), 
            'fleft':  min(min(msg.ranges[432:575]), 10), 
            'left':   min(min(msg.ranges[576:713]), 10), 
            } 
        print(self.regions)
        print("regions")

    def obstacle_avoid(self):

        while self.avoid != True:
            if self.regions['fright'] < self.dist and self.regions['front'] < self.dist and self.regions['fleft'] < self.dist: #no wall detected
                print("Wall Avoided")
                self.avoid = True
            elif self.regions['fright'] > self.dist and self.regions['front'] < self.dist and self.regions['fleft'] < self.dist: #wall on front-right
                print("Turning Left")
                self.turn(math.pi/4)
                self.avoid = False
            elif self.regions['fright'] < self.dist and self.regions['front'] > self.dist and self.regions['fleft'] < self.dist: #wall in front
                print("Turning left")
                self.turn(math.pi/2)
                self.avoid = False
            elif self.regions['fright'] < self.dist and self.regions['front'] < self.dist and self.regions['fleft'] > self.dist: #wall on front-left
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
                self. avoid = False

        if self.avoid == True:
            #call the goto function as the wall is avoided
            print("calling the goto function now")

    def check_obstacle(self):
        print("front = " + str(self.regions['front']))
        print("fleft = " + str(self.regions['fleft']))
        print("fright = " + str(self.regions['fright']))
        if self.regions['front'] < self.wall_dist:
            print("obstacle detected")
            self.message.linear.x = 0
            self.pub.publish(self.message)
            self.obstacle_avoid()

    def demo_controller(self):
        self.message = Twist()
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(15)

        while not rospy.is_shutdown():
            self.message.linear.x = 1.0
            self.pub.publish(self.message)
            self.check_obstacle()
            self.rate.sleep()


if __name__ == "__main__":
    avoider = obstacle_avoider()
    avoider.demo_controller()
    rospy.spin()

#! /usr/bin/env python3
import rospy
from geometry_msgs.msg import Point,Twist
from nav_msgs.msg import Odometry
import numpy as np
from tf.transformations import euler_from_quaternion
#from obstacle_avoider import obstacle_avoider

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

class Robot_Controller:
    #initialised values 
    def __init__(self):

        rospy.init_node('controller')
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        print("odom subs")
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.pose = []
        self.state = 0
        self.velocity_msg = Twist()
        #self.avoid = obstacle_avoider()

#obstace avoidence function
    def pose_provider(self):
        rospy.wait_for_message("/odom",Odometry)
        return self.pose

    #odmom callack function
    def odom_callback(self,data):
        x = data.pose.pose.orientation.x
        y = data.pose.pose.orientation.y
        z = data.pose.pose.orientation.z
        w = data.pose.pose.orientation.w
        self.pose = [data.pose.pose.position.x, data.pose.pose.position.y, euler_from_quaternion([x,y,z,w])[2]]
    
    #move function to move robot
    def move(self,linear,angular):
        self.velocity_msg.linear.x = linear
        self.velocity_msg.angular.z = angular 
        self.pub.publish(self.velocity_msg)
        # print("boom")

    #to correct postion of robot 
    def fix_yaw(self,error_a, P):
        self.move(0.1 * np.abs(error_a), P * -error_a)
    
    #Move straight to path 
    def move_straight(self,error, P):        
        self.move(P * error, 0)

    #//Go to function
    def goto(self,dest_x, dest_y):
     '''
      This function moves the bot towards the goal coordinates.
      The function uses a state machine with three states: 
      1) state = 0; fixing yaw 
      2) state = 1; moving straight
      3) state = 2; goal reached '''   
     theta_precision = 0.16  
     dist_precision = 0.35
     #wait for message
     rospy.wait_for_message("/odom",Odometry)
     while self.state != 2:
        theta_goal = np.arctan((dest_y - self.pose[1])/(dest_x - self.pose[0]))   #slope
        if theta_goal>0:
            theta_goal+=0.04
        elif theta_goal<0:
            theta_goal-=0.04
        bot_theta=self.pose[2]   
        theta_error = round(bot_theta - theta_goal, 2)
        rospy.loginfo("STATE: " + str(self.state))
        rospy.loginfo("THETA ERROR:" + str(theta_error))
        #self.avoid.avoider()
        if self.state == 0:
            # if theta_error is greated than the required precision then fix the yaw by rotating the bot
            # if required precision is reached then change current state to 1
            if np.abs(theta_error) > theta_precision:   
                rospy.loginfo("Fixing Yaw")
                self.fix_yaw(theta_error, 1.7)
            else:
                rospy.loginfo("Yaw Fixed!!")
                self.state=1
        elif self.state==1:
            # calculate error w.r.t to destination
            position_error = np.sqrt(pow(dest_y - self.pose[1], 2) + pow(dest_x - self.pose[0], 2)) #distance formula
            rospy.loginfo("POSITION ERROR: " + str(position_error))
            # if position error is less than required precision & bot is facing the goal, move towards goal in straight line
            # else if it is not correctly oriented change state to 1
            # if theta_precision and dist_precision are reached change state to 2 (goal reached)
            if position_error > dist_precision and np.abs(theta_error) < theta_precision:
                rospy.loginfo("Moving Straight")
                self.move_straight(position_error, 0.8)
            elif np.abs(theta_error) > theta_precision:
                rospy.loginfo("Going out of line!")
                self.state = 0
            elif position_error < dist_precision:
                rospy.loginfo("GOAL REACHED")
                self.move(0,0)
                self.state=2


class obstacle_avoider:
    def __init__(self):
        self.robot = Robot_Controller()
        self.pose = self.robot.pose_provider()
        self.regions = {
            'right': 0,
            'fright': 0, 
            'front': 0, 
            'fleft': 0, 
            'left': 0, 
            }
        #rospy.init_node('obstacle_avoider')
        rospy.Subscriber('/laser/scan', LaserScan, self.laser_callback)
        self.dist = 0.8
        self.dist2 = 0.7
        self.wall_dist = 0.8
        self.message = Twist()
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.velocity_msg = Twist()

        self.meh = True
        self.bruh = True

    def laser_callback(self, msg): 
        self.regions = { 
            'right':  min(min(msg.ranges[235:285]), 10), 
            'fright': min(min(msg.ranges[286:335]), 10), 
            'front':  min(min(msg.ranges[0:24] + msg.ranges[336:359]), 10), 
            'fleft':  min(min(msg.ranges[25:75]), 10), 
            'left':   min(min(msg.ranges[76:125]), 10), 
            }

    def avoider(self,a):
        obstacle = 0 # a variable to check wether to go right or left, 1 = right, 2 = left, 0 = default value for checking  
        while self.meh == True:

            theta_goal = np.arctan((dest_y - self.pose[1])/(dest_x - self.pose[0]))
            if theta_goal>0:
                theta_goal+=0.04
            elif theta_goal<0:
                theta_goal-=0.04

            bot_theta = self.pose[2]   
            theta_error = round(bot_theta - theta_goal, 2)
            rospy.loginfo("STATE: " + str(self.state))
            rospy.loginfo("THETA ERROR:" + str(theta_error))
            try:
                if self.regions(min(min(msg.ranges[:], 10))) != 10:
                    print("obstacle in between the straight path to goal")
            except:
                print("Error here something meh......")

            if self.regions['front'] > 1: # 1 checked by trial and error tells how close the obstacle is, greater the value further away the obstacle is 
                print("going to the location")
                self.robot.goto(1, 0) 

            else:
                if self.regions['right'] > 1:
                    print("right clear")
                    obstacle = 1
                    print(self.pose)

                    while self.bruh == True:
                        self.velocity_msg.linear.x = 0
                        self.velocity_msg.angular.z = -1 
                        self.pub.publish(self.velocity_msg)

                        print("moving.....right")
                        if self.regions['front'] > 1 :
                            self.velocity_msg.linear.x = 0
                            self.velocity_msg.angular.z = 0 
                            self.pub.publish(self.velocity_msg)
                            self.bruh = False
                    
                    while self.regions['left'] < 1:
                        self.velocity_msg.linear.x = 0.6
                        self.velocity_msg.angular.z = 0 
                        self.pub.publish(self.velocity_msg)

                        print("moving..... straight now")
                        if self.regions['left'] > 1:
                            self.velocity_msg.linear.x = 0
                            self.velocity_msg.angular.z = 0 
                            self.pub.publish(self.velocity_msg)



                    self.avoider(1)

                elif self.regions['left'] > 1:
                    print("left clear")
                    obstacle = 2
                    print(self.pose)

                    while self.bruh == True:
                        self.velocity_msg.linear.x = 0
                        self.velocity_msg.angular.z = 1 
                        self.pub.publish(self.velocity_msg)

                        print("moving.....left")
                        if self.regions['front'] > 1 :
                            self.velocity_msg.linear.x = 0
                            self.velocity_msg.angular.z = 0 
                            self.pub.publish(self.velocity_msg)
                            self.bruh = False

                    self.avoider(1)

                else:
                    print("surrounded by obstacle pls help :3")

                 

if __name__ == "__main__":
    hi = obstacle_avoider()
    hi.avoider(1)
    """
    Robot = Robot_Controller()
    Robot.goto(3,2)
    """
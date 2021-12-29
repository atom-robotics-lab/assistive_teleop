#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
from tf.transformations import euler_from_quaternion

class Robot_Controller:
    #initialised values 
    def __init__(self):

        rospy.init_node('controller')
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.pose = []
        self.state = None
        self.velocity_msg = Twist()
        self.pub = None
        self.d = 1.0
        self.dmax = 2.5
        self.d_state = 0
        self.regions = {'right': 0,'fright': 0,'front': 0,'fleft': 0,'left': 0}
        self.w_state=0

#obstace avoidence function
    def obstackle_avoidence() :
        pass

    #odmom callack function
    def odom_callback(self,data):
      x  = data.pose.pose.orientation.x
      y  = data.pose.pose.orientation.y
      z = data.pose.pose.orientation.z
      w = data.pose.pose.orientation.w
      self.pose = [data.pose.pose.position.x, data.pose.pose.position.y, euler_from_quaternion([x,y,z,w])[2]]
    #move function to move robot
    def move(linear,angular,self):
        self.velocity_msg.linear.x = linear
        self.velocity_msg.angular.z = angular #checkout
        self.pub.publish(self.velocity_msg)

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
     self.theta_precision = 0.16  
     self.dist_precision = 0.35
     while self.state != 2:
        theta_goal = np.arctan((dest_y - self.pose[1])/(dest_x - self.pose[0]))
        if theta_goal>0:
            theta_goal+=0.04
        elif theta_goal<0:
            theta_goal-=0.04
        bot_theta=self.pose(2)    
        theta_error = round(bot_theta - theta_goal, 2)
        rospy.loginfo("STATE: " + str(self.state))
        rospy.loginfo("THETA ERROR:" + str(theta_error))
        if self.state == 0:
            # if theta_error is greated than the required precision then fix the yaw by rotating the bot
            # if required precision is reached then change current state to 1
            if np.abs(theta_error) > self.theta_precision:   
                rospy.loginfo("Fixing Yaw")
                self.fix_yaw(theta_error, 1.7)
            else:
                rospy.loginfo("Yaw Fixed!!")
                self.state=1
        elif self.state==1:
            # calculate error w.r.t to destination
            position_error = np.sqrt(pow(dest_y - self.pose[1], 2) + pow(dest_x - self.pose[0], 2))
            rospy.loginfo("POSITION ERROR: " + str(position_error))
            # if position error is less than required precision & bot is facing the goal, move towards goal in straight line
            # else if it is not correctly oriented change state to 1
            # if theta_precision and dist_precision are reached change state to 2 (goal reached)
            if position_error > self.dist_precision and np.abs(theta_error) < self.theta_precision:
                rospy.loginfo("Moving Straight")
                self.move_straight(position_error, 0.8)
            elif np.abs(theta_error) > self.theta_precision:
                rospy.loginfo("Going out of line!")
                self.state = 0
            elif position_error < self.dist_precision:
                rospy.loginfo("GOAL REACHED")
                self.state=2


if __name__ == "__main__":
    Robot = Robot_Controller()
    Robot.goto(5,5)
           
                   

                
              
        
        

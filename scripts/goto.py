#! /usr/bin/env python3
import rospy
from geometry_msgs.msg import Point,Twist
from nav_msgs.msg import Odometry
import numpy as np
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import LaserScan

class Robot_Controller:
    #initialised values
    def __init__(self):

        rospy.init_node('controller')
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.pose = []
        self.state = 0
        self.velocity_msg = Twist()
        self.avoid = obstacle_avoider()

#obstace avoidence function
    def obstacle_avoidence(self) :
        pass

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
        self.avoid.check_obstacle()
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
        self.avoid1 = False
        self.avoid2 = False
        self.message = Twist()
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    def laser_callback(self, msg): 
        self.regions = { 
            'right':  min(min(msg.ranges[235:285]), 10), 
            'fright': min(min(msg.ranges[286:335]), 10), 
            'front':  min(min(msg.ranges[0:24] + msg.ranges[336:359]), 10), 
            'fleft':  min(min(msg.ranges[25:75]), 10), 
            'left':   min(min(msg.ranges[76:125]), 10), 
            }         

    def obstacle_avoid(self):
        print("obstacle avoider started")
        self.avoid1 = False

        while self.avoid1 == False:
            print(self.regions)
            #print("into while loop")
            if self.regions['fright'] > self.dist2 and self.regions['front'] > self.dist and self.regions['fleft'] > self.dist2: #no wall detected
                print("Wall Avoided")
                self.avoid1 = True
            elif self.regions['fright'] < self.dist2 and self.regions['front'] > self.dist and self.regions['fleft'] > self.dist2: #wall on front-right
                print("Turning Left for front-right")
                self.move(0.0, 0.3)
                self.avoid1 = False
            elif self.regions['fright'] > self.dist2 and self.regions['front'] < self.dist and self.regions['fleft'] > self.dist2: #wall in front
                print("Turning left for front")
                self.move(0.0, 0.3)
                self.avoid1 = False
            elif self.regions['fright'] > self.dist2 and self.regions['front'] > self.dist and self.regions['fleft'] < self.dist2: #wall on front-left
                print("Turning right for front-left")
                self.move(0.0, -0.3)
                self.avoid1 = False
            elif self.regions['fright'] < self.dist2 and self.regions['front'] < self.dist and self.regions['fleft'] > self.dist2: #wall in front and front-right
                print("Turning left for front and front-right")
                self.move(0.0, 0.3)
                self.avoid1 = False
            elif self.regions['fright'] > self.dist2 and self.regions['front'] < self.dist and self.regions['fleft'] < self.dist2: #wall in front and front-left
                print("Turning right for front and front-left")
                self.move(0.0, -0.3)
                self.avoid1 = False 
            elif self.regions['fright'] < self.dist2 and self.regions['front'] < self.dist and self.regions['fleft'] < self.dist2: #wall everywhere
                if self.regions['fleft'] < self.regions['fright']:
                    print("Turning right for front and front-left and ront-right")
                    self.move(0.0, -0.3)
                    self.avoid1 = False   
                elif self.regions['fleft'] > self.regions['fright']:
                    print("Turning left for front and front-left and ront-right")
                    self.move(0.0, 0.3)
                    self.avoid1 = False
        
        if self.avoid1 == True:
            #call the goto function as the wall is avoided
            #print("calling the goto function now")
            self.avoid2 = False
            while self.avoid2 == False:
                if self.regions['right'] <= 0.7 or self.regions['left'] <= 0.7: #wall on either left or right
                    self.move(1.0, 0.0)
                    self.avoid2 = True

    def check_obstacle(self):
        print("front = " + str(self.regions['front']))
        print("fleft = " + str(self.regions['fleft']))
        print("fright = " + str(self.regions['fright']))
        if self.regions['front'] < self.wall_dist or self.regions['fleft'] < self.dist2 or self.regions['fright'] < self.dist2:
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
    Robot = Robot_Controller()
    Robot.goto(3,2)
    
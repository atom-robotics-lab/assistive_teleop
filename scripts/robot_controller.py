#! /usr/bin/env python3
import rospy
from geometry_msgs.msg import Point,Twist
from nav_msgs.msg import Odometry
import numpy as np
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import LaserScan
import actionlib
from assistive_teleop.msg import AvoidObstacleAction, AvoidObstacleGoal, AvoidObstacleFeedback
#from assistive_teleop import obstacle_presence

                                        # ------ class ------ #

class Robot_Controller:
    
    def __init__(self):
        rospy.init_node('controller')
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.pose = []
        self.dist_precision  = 0.1
        self.theta_precision = 0.1
        self.state = 0
        self.velocity_msg = Twist() 
        self.ob_status = False
        self.client = actionlib.SimpleActionClient('Avoid_Obstacle_server', AvoidObstacleAction)
        print('waiting for avoid obstacle server')
        self.client.wait_for_server()
        print('server started')
        
        
                                     # ------- object call back --------- #

    def object_cb(self, feedback):
        self.ob_status = feedback.obstacle_presence
         
                                     # ------- request --------- #

    def request(self, feedback):
        if self.ob_status == False:
            print("sending request to server/ obstacle avoider")      
        else:
            pass


                                    # ------- odom_callback --------- #  
                                     
    def odom_callback(self,data):
      x = data.pose.pose.orientation.x
      y = data.pose.pose.orientation.y
      z = data.pose.pose.orientation.z
      w = data.pose.pose.orientation.w
      self.pose = [data.pose.pose.position.x, data.pose.pose.position.y, euler_from_quaternion([x,y,z,w])[2]]
              
                                   # ---------- move ----------- #
              
    def move(self,linear,angular):
        self.velocity_msg.linear.x = linear
        self.velocity_msg.angular.z = angular 
        self.pub.publish(self.velocity_msg)
        
                                  # ------------ fix error ------------- #
                                  
    def fix_error(self, linear_error, orien_error):
        if linear_error != 0 and orien_error != 0:
            self.move(2*linear_error, 2*-1*orien_error)
        if orien_error != 0 and linear_error == 0:            
             self.move(0,2*-1*orien_error)

                                     
                                    # ------- goto  ---------- #
                                     
    def goto(self,dest_x, dest_y):
        
        
        self.dest_x = dest_x
        self.dest_y = dest_y
        self.goal = AvoidObstacleGoal()
        self.goal.pose.append(dest_x)
        self.goal.pose.append(dest_y)
        self.client.send_goal(self.goal, feedback_cb = self.object_cb) 
        rospy.wait_for_message("/odom",Odometry)
        
        util = self.utils()
        bot_theta_error = util[0]
        bot_position_error = util[1]
        

        
        while (np.abs(bot_theta_error) >= self.theta_precision or np.abs(bot_position_error) >= self.dist_precision ) : 
        
            if self.ob_status == True:
                print("Obstacle Detected, Stopping the BOT")
                self.move(0,0)
                self.client.wait_for_result()
                
            util = self.utils()
            bot_theta_error = util[0]
            bot_position_error = util[1]
            
            rospy.loginfo("position error: {:.2f} m ".format(bot_position_error))  
            rospy.loginfo("THETA ERROR: {:.2f}° ".format((180/3.14)*bot_theta_error))
            
            if np.abs(bot_theta_error) <= 0.3 : 
                      print(" ")
                      print("*********************************")
                      print("* FIXING YAW && MOVING STRAIGHT *")
                      print("*********************************")    
                      print(" ") 
                      self.fix_error(0.5, bot_theta_error) 
                      if self.ob_status == True:
                            print("Obstacle Detected, Stopping the BOT")
                            self.move(0,0)
                            self.client.wait_for_result()
                            
                      util = self.utils()                    
                      bot_theta_error = util[0]
                      bot_position_error = util[1]
                      
            elif  np.abs(bot_theta_error) > 0.3:
                      print(" ")
                      print("************************")
                      print("*** FIXING YAW ****")
                      print("************************") 
                      print(" ") 
                      self.fix_error(0, bot_theta_error) 
                      
                      if self.ob_status == True:
                            print("Obstacle Detected, Stopping the BOT")
                            self.move(0,0)
                            self.client.wait_for_result()
                            
                      utils = self.utils()      
                      bot_theta_error = utils[0]
                      bot_position_error = utils[1]
                      
        if bot_position_error <= self.dist_precision and bot_theta_error <= self.theta_precision :
                      print(" ")
                      print("*****************************")
                      print("** HURRAY !! GOAL REACHED ***")
                      print("*****************************")  
                      print(" ")                                                         
                      print("cancelling Obstacle Avoider Goal")
                      self.move(0,0)
                      self.client.cancel_all_goals() 
                      
                     
                  # ---------- utils ---------- # 
                  
    def utils(self):
        bot_position_error = np.sqrt(pow(self.dest_y - self.pose[1], 2) + pow(self.dest_x - self.pose[0], 2)) 
        bot_theta_goal = np.arctan((self.dest_y - self.pose[1])/(self.dest_x - self.pose[0])) 
        bot_theta= self.pose[2]   
        bot_theta_error = round(bot_theta - bot_theta_goal, 2) 
        
        return [bot_theta_error,bot_position_error]
        

if __name__ == "__main__":
    Robot = Robot_Controller()
    Robot.goto(2,6)
    Robot.move(0,0)
    

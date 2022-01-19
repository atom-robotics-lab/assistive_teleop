#! /usr/bin/env python3

# import ros stuff
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf import transformations
from tf.transformations import euler_from_quaternion

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
        self.state_ = 0
        self.state_dict_ = {
            0: 'find the wall',
            1: 'turn left',
            2: 'follow the wall',
        }
        self.message = Twist()
        self.d = 1.0
        self.dist = 0.8
        self.dist2 = 0.7
        self.wall_dist = 0.8
        rospy.init_node('reading_laser')
        self.pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)        
        self.sub = rospy.Subscriber('/laser/scan', LaserScan, self.clbk_laser)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)

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
        self.take_action()

    def change_state(self, state):
        self.state_
        self.state_dict_
        if state is not self.state_:
            print ('Wall follower - [%s] - %s' % (state, self.state_dict_[state]))
            self.state_ = state

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
        msg = Twist()
        msg.linear.x = 0.2
        msg.angular.z = -0.3
        return msg

    def turn_left(self):
        msg = Twist()
        msg.angular.z = 0.3
        return msg

    def follow_the_wall(self):
        self.regions
        
        msg = Twist()
        msg.linear.x = 0.5
        return msg

    def _state_(self):
        
        rate = rospy.Rate(15)
        while not rospy.is_shutdown():
            print("front :" + str(self.regions['front']))
            print("fright :" + str(self.regions['fright']))
            print("fleft :" + str(self.regions['fleft']))
            msg = Twist()
            if self.state_ == 0:
                msg = self.find_wall()
            elif self.state_ == 1:
                msg = self.turn_left()
            elif self.state_ == 2:
                msg = self.follow_the_wall()
                pass
            else:
                rospy.logerr('Unknown state!')
            
            self.pub_.publish(msg)
            
            rate.sleep()

    def check_obstacle(self):
        print("front = " + str(self.regions['front']))
        print("fleft = " + str(self.regions['fleft']))
        print("fright = " + str(self.regions['fright']))
        if self.regions['front'] < self.wall_dist or self.regions['fleft'] < self.dist2 or self.regions['fright'] < self.dist2:
            print("obstacle detected")
            self.move(0.0, 0.0)

if __name__ == '__main__':
    obs_state = follow_wall()
    obs_state._state_()

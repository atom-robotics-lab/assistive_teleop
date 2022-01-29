
#! /usr/bin/env python3

import roslib
import rospy
import actionlib

from assistive_teleop.msg import AvoidObstacleAction, AvoidObstacleActionGoal

  if __name__ == '__main__':
      rospy.init_node('Obstacle_Client_test')
      client = actionlib.SimpleActionClient('Avoid_Obstacle', AvoidObstacleAction)
      client.wait_for_server()
  
      goal = AvoidObstacleActionGoal()
      goal.angle = 5
      # Fill in the goal here
      client.send_goal(goal)
      client.wait_for_result(rospy.Duration.from_sec(5.0))

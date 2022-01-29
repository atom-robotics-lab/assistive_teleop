
#! /usr/bin/env python

import roslib
import rospy
import actionlib

from assistive_teleop.msg import AvoidObstacleAction

class ObstacleAvoidanceServer:
  def __init__(self):
    self.server = actionlib.SimpleActionServer('Avoid_Obstacle', AvoidObstacleAction, self.execute, False)
    self.server.start()

  def execute(self, goal):
    # Do lots of awesome groundbreaking robot stuff here
    self.server.set_succeeded()


if __name__ == '__main__':
  rospy.init_node('Obstacle_Server_test')
  server = ObstacleAvoidanceServer()
  rospy.spin
#! /usr/bin/env python3

import roslib
import rospy
import actionlib

from assistive_teleop.msg import AvoidObstacleAction, AvoidObstacleFeedback, AvoidObstacleResult

class ObstacleAvoidanceServer:
  def __init__(self):
    self.server = actionlib.SimpleActionServer('Avoid_Obstacle', AvoidObstacleAction, self.execute, False)
    self.server.start()
    print('action server starting')
    self.feedback = AvoidObstacleFeedback()
    self.result = AvoidObstacleResult()

  def execute(self, goal):
    rate = rospy.Rate(100)
    index = 1
    while True:
      if index % 3 == 0:
        self.feedback.obstacle_clearance = True
        self.server.publish_feedback(self.feedback)
        print('obstacle detected: {}'.format(index))
        rospy.sleep(3)
        print('obstacle avoided: {}'.format(index))
        self.server.set_succeeded()
      
      else:
        print('no obstacle detected: {}'.format(index))
        self.feedback.obstacle_clearance = False
        self.server.publish_feedback(self.feedback)

      rospy.sleep(2)
      index +=1
      rate.sleep()

    self.server.set_succeeded()


if __name__ == '__main__':
  rospy.init_node('Obstacle_Server_test')
  server = ObstacleAvoidanceServer()
  rospy.spin()
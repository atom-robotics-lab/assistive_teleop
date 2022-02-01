#! /usr/bin/env python3

import roslib
import rospy
import actionlib

from assistive_teleop.msg import AvoidObstacleAction, AvoidObstacleGoal, AvoidObstacleFeedback

class drive_client:

  def __init__(self):

    self.ob_status = False
    self.client = actionlib.SimpleActionClient('Avoid_Obstacle', AvoidObstacleAction)
    print('waiting for server')
    self.client.wait_for_server()
    print('server started')
    goal = AvoidObstacleGoal()
    goal.angle = 5
    self.client.send_goal(goal, feedback_cb = self.object_cb)

  
  def object_cb(self, feedback):
    self.ob_status = feedback.obstacle_clearance

  def drive_robot(self):
    rate = rospy.Rate(100)
    drive_task = 0

    while drive_task != 10:
      if self.ob_status:
        print('stoped Object Avoidance in Progress')
        self.client.wait_for_result()

      else:
        drive_task += 1
        print('task completed {}/10'.format(drive_task)) 
      rospy.sleep(2)

      rate.sleep()


  

if __name__ == '__main__':
  rospy.init_node('Obstacle_Client_test')
  robot_driver = drive_client()
  robot_driver.drive_robot()

  
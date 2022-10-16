#! /usr/bin/env python3
import rospy
from nav_msgs.msg import Path
import numpy as np


class WaypointManager:
    def __init__(self):
        rospy.init_node('waypoints_sub')
        self.w_pose = []
        self.waypoints_x = 0
        self.waypoints_y = 0
        rospy.Subscriber('/move_base/TrajectoryPlannerROS/global_plan', Path, self.callback)

    def callback(self, data):
        self.waypoints_x = data.poses[0].pose.position.x
        self.waypoints_y = data.poses[0].pose.position.y
        
    def way_point(self):
        x = np.array([])
        y = np.array([])
        rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            x = np.append(x, [self.waypoints_x])     
            y = np.append(y, [self.waypoints_y])
            rospy.loginfo("x-axis:"+str(x))
            rospy.loginfo("y-axis:"+str(y))
            rospy.loginfo("")
            rate.sleep()


if __name__ == "__main__":
    obj = WaypointManager()
    obj.way_point()


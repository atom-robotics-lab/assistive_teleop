
import rospy 
from geometry_msgs.msg import Twist 
from sensor_msgs.msg import LaserScan 

regions = {
    'right': 0, 
    'fright': 0, 
    'front': 0, 
    'fleft': 0, 
    'left': 0, 
}

class obstacle_avoider:

    def laser_callback(msg): 
        global regions 
        regions = { 
            'right':  min(min(msg.ranges[0:143]), 10), 
            'fright': min(min(msg.ranges[144:287]), 10), 
            'front':  min(min(msg.ranges[288:431]), 10), 
            'fleft':  min(min(msg.ranges[432:575]), 10), 
            'left':   min(min(msg.ranges[576:713]), 10), 
    } 

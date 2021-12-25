from genpy import message
import rospy 
from geometry_msgs.msg import Twist 
from sensor_msgs.msg import LaserScan 
import math

dist = 0.5
avoid = False

class obstacle_avoider:
    def __init__(self):

        self.regions = {
            'right': 0, 
            'fright': 0, 
            'front': 0, 
            'fleft': 0, 
            'left': 0, 
            }

    pub = None
    message = Twist()

    def turn(self, z):
        message
        message.linear.x = 0
        message.angular.z = z
        self.pub.publish(message)

    def laser_callback(self, msg): 
        self.regions = { 
            'right':  min(min(msg.ranges[0:143]), 10), 
            'fright': min(min(msg.ranges[144:287]), 10), 
            'front':  min(min(msg.ranges[288:431]), 10), 
            'fleft':  min(min(msg.ranges[432:575]), 10), 
            'left':   min(min(msg.ranges[576:713]), 10), 
    } 

    rospy.Subscriber('/ebot/laser/scan', LaserScan, laser_callback())
    rate = rospy.Rate(15)

    def obstacle_avoid(self , lin_vel , ang_vel):

        self.vel.linear.x = lin_vel
        self.vel.angular.z = ang_vel

        while avoid != True:
       
            if self.regions['fright'] < dist and self.regions['front'] < dist and self.regions['fleft'] < dist: #no wall detected
                print("Wall Avoided")
                avoid = True
            elif self.regions['fright'] > dist and self.regions['front'] < dist and self.regions['fleft'] < dist: #wall on front-right
                print("Turning Left")
                self.turn(math.pi/4)
                avoid = False
            elif self.regions['fright'] < dist and self.regions['front'] > dist and self.regions['fleft'] < dist: #wall in front
                print("Turning left")
                self.turn(math.pi/2)
                avoid = False
            elif self.regions['fright'] < dist and self.regions['front'] < dist and self.regions['fleft'] > dist: #wall on front-left
                print("Turning right")
                self.turn(-math.pi/4)
                avoid = False
            elif self.regions['fright'] < dist or self.regions['front'] < dist or self.regions['fleft'] > dist: #wall either in front and front-right
                print("Turning left")
                self.turn(math.pi/4)
                avoid = False
            elif self.regions['fright'] > dist or self.regions['front'] < dist or self.regions['fleft'] < dist: #wall either in front and front-left
                print("Turning right")
                self.turn(-math.pi/4)
                avoid = False

        if avoid == True:
            #call the goto function as the wall is avoided
            print("calling the goto function now")


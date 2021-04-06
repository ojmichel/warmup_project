#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import Twist, Vector3

class DriveSquare(object):

    def __init__(self):
        rospy.init_node('drive_square')
        self.move_pub = rospy.Publisher('/cmd_vel',Twist,queue_size=10)

    def run(self):
        rospy.sleep(2)
        r = rospy.Rate(4.0)
        msg = Twist()
        ang_speed = np.pi / 10.0 #turning for 5 seconds, want total angle of pi/2

        for _ in range(4): #go in square
            msg.linear.x = 0.25
            msg.angular.z = 0.0
            r.sleep()
            for _ in range(16): #dive for 4 seconds
                self.move_pub.publish(msg)
                r.sleep()
            msg.linear.x = 0.0
            msg.angular.z = ang_speed
            for _ in range(20): #turn for 5 seconds
                self.move_pub.publish(msg)
                r.sleep()
        self.move_pub.publish(Twist()) #stop robot


if __name__ == '__main__':
    node = DriveSquare()
    node.run()




            

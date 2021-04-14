#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan

class WallFollower(object):

    def __init__(self):

        rospy.init_node('wall_follower')
        rospy.on_shutdown(self.shutdown)
        self.move_pub = rospy.Publisher('/cmd_vel',Twist,queue_size=10)
        self.scan_sub = rospy.Subscriber('/scan',LaserScan,callback=self.scanner)

    
    def shutdown(self):
        self.move_pub.publish(Twist())

    def run(self):
        rospy.spin()

    def scanner(self,data):

        K_ang = 0.75
        K_lin = 1.25
        N = len(data.ranges)
        epsilon = 0.25
        stop_dist = 0.5    
        front_dist = data.ranges[0]
        side_dist = data.ranges[3*N//4]
        msg = Twist()

        if abs(front_dist - side_dist) < epsilon: 
            angle = 0 #this is to prevent getting stuck when front_dist = side_dist, choose front bc turning
        else:
            angle = self.find_min_angle(data) #angle of least distance is deviation from parallel w/ wall

        if side_dist < 0.50: #dont want touching the wall, otherwise cant turn
            angle += np.pi / 6.0
            
        #min part is in case of "inf" distance, max is proportional controll
        msg.linear.x = min(0.5,max(0.0,K_lin*(front_dist - stop_dist))) 
        msg.angular.z = K_ang*(np.pi / 2.0 + angle) #want parallel to side wall
        
        self.move_pub.publish(msg)
              
    def find_min_angle(self,data):
        N = len(data.ranges)
        min_dist = float('inf')
        angle = 0.0
    
        for i in range(N):
            if data.ranges[i] < min_dist:
                min_dist = data.ranges[i]
                angle = i
        
        if angle > len(data.ranges)//2: #for example, want -90 deg instead of 270 deg
            angle = angle - len(data.ranges)
        
        angle = angle * (2.0*np.pi / N) #convert to radians (N is 360 in this case)
    
        return angle
    

if __name__ == '__main__':
    node = WallFollower()
    node.run()
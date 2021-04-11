#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan

class PersonFollower(object):

    def __init__(self):
        rospy.init_node('person_follower')
        rospy.on_shutdown(self.shutdown)
        self.move_pub = rospy.Publisher('/cmd_vel',Twist,queue_size=10)
        self.scan_sub = rospy.Subscriber('/scan',LaserScan,callback=self.scanner)

    def run(self):
        rospy.spin()

    def shutdown(self):
        self.move_pub.publish(Twist())
        
    def scanner(self,data):
        
        min_dist = float('inf')
        angle = 0
        N = len(data.ranges)

        for i in range(N):
            if data.ranges[i] < min_dist:
                min_dist = data.ranges[i]
                angle = i

        self._update_vel(angle,min_dist,N)
        
    def _update_vel(self,angle,min_dist,N):

        K_ang = 0.50
        K_lin = 0.25
        stop_dist = 0.5
        msg = Twist()
        
        if angle > N//2: #for example, want -90 deg instead of 270 deg
            angle = angle - N

        if min_dist < float('inf'):
            angle = angle * (2.0*np.pi / N) #convert to radians (N is 360 in this case)
            msg.angular.z = K_ang * angle
            behind = (np.pi / 4.0 <= angle <= np.pi) or (-np.pi <= angle <= -np.pi / 4.0) #behind := outside of +- pi/4
            if not behind: #don't want linear vel when person behind
                msg.linear.x = K_lin * max(0, min_dist - stop_dist) #stop when stop_dist meters away
            #print(msg.angular.z,msg.linear.x)

        self.move_pub.publish(msg) 


        


if __name__ == "__main__":
    node = PersonFollower()
    node.run()

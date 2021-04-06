# warmup_project

## Driving in Square

To drive in a sqaure, I made the robot drive straight for 4 seconds at 0.25 m/s, so 1 meter in total. Then I turned the robot 90 degrees. This was done by setting the angular velocity to pi/10 rad/s for a total of 5 seconds. The code for this task is contained in a single run() method within the DriveSquare class. The code utilizes a rospy rate object, which I found to be very helpful for timing the robot movements. The outer loop is executed 4 times, to drive in a single square. Two inner loops control the turning and forward movement of the robot. 

![](robot_square.gif)






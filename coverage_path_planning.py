#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time
import math
from math import pi
import numpy

DEG2RAD = (pi / 180.0)
RAD2DEG = (180.0 / pi)

CENTER = 0
LEFT   = 1
RIGHT  = 2

LINEAR_VELOCITY  = 0.2
ANGULAR_VELOCITY = 1.5

GET_TB3_DIRECTION = 0
TB3_DRIVE_FORWARD = 1
TB3_RIGHT_TURN    = 2
TB3_LEFT_TURN     = 3

scan_data = [0 , 0 , 0 ]
check_forward_dist = 1.14
check_side_dist  = 1.13;
escape_range   = 30.0 * DEG2RAD;
prev_tb3_pose = 0.0
tb3_pose = 0.0;
def odomMsgCallBack(data):
    siny = 2.0 * (data.pose.pose.orientation.w * data.pose.pose.orientation.z + data.pose.pose.orientation.x * data.pose.pose.orientation.y)
    cosy = 1.0 - 2.0 * (data.pose.pose.orientation.y * data.pose.pose.orientation.y + data.pose.pose.orientation.z * data.pose.pose.orientation.z);
    tb3_pose = math.atan2(siny, cosy);
    

def laserScanMsgCallBack(msg):
    #rospy.loginfo("laser callback")
    scan_angle = [0, 30, 330]
    #rospy.loginfo(msg.ranges[0])

    for x in scan_angle:
         if (numpy.isnan(msg.ranges[x])):
             scan_data[scan_angle.index(x)] = msg.range_max
             
         else:
             scan_data[scan_angle.index(x)] = msg.ranges[x]
             

def updatecommandVelocity(linear,angular):
    cmd_vel = Twist()
    cmd_vel.linear.x = linear
    cmd_vel.angular.z = angular
    return cmd_vel
    #cmd_vel_pub.publish(cmd_vel)	

def listener():
    rospy.init_node('coverage_plan_node', anonymous=True)
    cmd_vel_pub = rospy.Publisher("cmd_vel",Twist, queue_size=50)
    rospy.Subscriber("scan", LaserScan, laserScanMsgCallBack)
    rospy.Subscriber("odom", Odometry, odomMsgCallBack)
    turtlebot3_state_num = 0
    
    while not rospy.is_shutdown():
            print("scan_data[CENTER]")
            print(scan_data[CENTER])
	    if (turtlebot3_state_num == GET_TB3_DIRECTION):
                print("loop1")
                
		if (scan_data[CENTER] > check_forward_dist):#if no obs front
     		    cmd_vel = updatecommandVelocity(LINEAR_VELOCITY, 0);
		    cmd_vel_pub.publish(cmd_vel)#move forward
		    print("straight")
	        elif (scan_data[CENTER] < check_forward_dist):#if obs front
		    if (scan_data[LEFT] > check_forward_dist):#if no obs left
			    cmd_vel = updatecommandVelocity(0.0, ANGULAR_VELOCITY);#left
		            cmd_vel_pub.publish(cmd_vel)
		            #time.sleep(2)
		            print("left")
		    elif (scan_data[RIGHT] > check_forward_dist):#if no obs right
			    cmd_vel = updatecommandVelocity(0.0, -1*ANGULAR_VELOCITY);#right
		            cmd_vel_pub.publish(cmd_vel)
		            #time.sleep(2)
		            print("right")
		    else:
			    cmd_vel = updatecommandVelocity(0.0, 3.14);#right
		            cmd_vel_pub.publish(cmd_vel)
		            #time.sleep(2)
		            print("back")
                else:
        	    cmd_vel = updatecommandVelocity(0.0, 0.0);
                    cmd_vel_pub.publish(cmd_vel)
                    time.sleep(2)
                    print("no movement")

	   
if __name__ == '__main__':
    try:
        listener()
        updatecommandVelocity(0.0, 0.0);
    except KeyboardInterrupt:
        updatecommandVelocity(0.0, 0.0);
        raise KeyboardInterrupt

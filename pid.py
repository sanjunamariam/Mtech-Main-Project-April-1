#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import time
import tf
import math
from math import sin, cos, pi
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
#pid params
kp = 400
ki = 100
kd = 0

def callback(data):
    ##rospy.loginfo("orientation_w %s" % orientation_w )
def listener():
    rospy.init_node('pid_node', anonymous=True)
    pid_command_pub_r = rospy.Publisher("pid_vel_r",Odometry, queue_size=50)
    pid_command_pub_l = rospy.Publisher("pid_vel_l",Odometry, queue_size=50)
    ##rospy.Subscriber("imu", Imu, callback)
    r = rospy.Rate(10.0)
    while not rospy.is_shutdown():
	    last_time = rospy.Time.now()
	    #error
	    error_r = wr - feedback_wr
	    error_l = wl - feedback_wl

	    current_time = rospy.Time.now()
	    time = (current_time-last_time).to_sec()
	    #integral of error
	    error_r_integral += error_r * time
	    error_l_integral += error_l * time
	    #derivative of error
	    error_r_derivative = (error_r - previous_error_r)/time
	    error_l_derivative = (error_l - previous_error_l)/time

	    previous_error_r = error_r
	    previous_error_l = error_l

            pid_r = (kp * error_r) + (ki * error_r_integral) + (kd * error_r_derivative)
            pid_l = (kp * error_l) + (ki * error_l_integral) + (kd * error_l_derivative)
            
if __name__ == '__main__':
    listener()

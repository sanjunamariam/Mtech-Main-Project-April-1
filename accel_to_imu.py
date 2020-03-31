#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import time
import tf
import math
from math import sin, cos, pi
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from std_msgs.msg import Int32

global ax 
global ay 
global az 
ax = 0
ay = 0
az = 0

def callback_x(data):
    #listen to ACCELEROMETER X topic
    ax = data.data
    ay = data2.data 
    az = data3.data
    
    rospy.loginfo("ax %s" % ax )
    rospy.loginfo("ay %s" % ay )
    rospy.loginfo("az %s" % az )
 #publish imu
    imu = Imu()        
    imu.header.stamp = rospy.Time.now()
    imu.header.frame_id = 'hokuyo_link'
####### need some processing of
    imu.linear_acceleration.x = ax
    imu.linear_acceleration.y = ay
    imu.linear_acceleration.z = az
    imu_pub.publish(imu);    
    
def callback_y(data2):
    #listen to ACCELEROMETER Y topic
    ay = data2.data  
    rospy.loginfo("ay %s" % ay )
 
def callback_z(data3):
    #listen to ACCELEROMETER Z topic
    az = data3.data
    rospy.loginfo("az %s" % az )
def listener():
    rospy.init_node('imu_to_odom_node', anonymous=True)
    current_time = rospy.Time.now()
    last_time = rospy.Time.now() 
    
    imu_pub = rospy.Publisher("imu",Imu, queue_size=50)
    rospy.Subscriber("accelerometer_x", Int32, callback_x)
    rospy.Subscriber("accelerometer_y", Int32, callback_x)
    rospy.Subscriber("accelerometer_z", Int32, callback_x)   
    r = rospy.Rate(10.0)
    rospy.loginfo("hi" )
    rospy.spin()
if __name__ == '__main__':
    listener()

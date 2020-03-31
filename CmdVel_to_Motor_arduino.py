#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_msgs.msg import Int32
import time
from math import sin, cos, pi

#publishes pwm for right and left.. also direction flag
def callback(data):
    #radius of wheel (rough value in m 4.5cm)
    wheel_rad = 0.045

    
    #distance between wheels 18cm
    dist_bw_wheels = 0.18
    v = data.linear.x
    theta = data.angular.z
    #unit m/s
    rospy.loginfo(rospy.get_name() + ": SPEED LINEAR %s" % v )
    #unit rad/s
    rospy.loginfo(rospy.get_name() + ": ORIENTATION %s" % theta )
    v_r = (v) + ((dist_bw_wheels * theta)/(2))
    v_l = (v) - ((dist_bw_wheels * theta)/(2))
    rospy.loginfo("linear v right %s" % v_r )
    rospy.loginfo("linear v left %s" % v_l )
    w_r = (v/wheel_rad) + ((dist_bw_wheels * theta)/(2*wheel_rad))
    w_l = (v/wheel_rad) - ((dist_bw_wheels * theta)/(2*wheel_rad))
    #unit rad/s
    rospy.loginfo("angular v right %s" % w_r )
    rospy.loginfo("angular v left %s" % w_l ) 

    #ARDUINO
    #guess 72 rev per min .. need to change  72 rev /60 sec =  1.2 rev/sec = 7.5 rad/sec... so 7.5 rad/sec --> 255
    # so eqn (angular vel / 7.5 ) * 255 is the pwm signal for each motor
    pub_right = rospy.Publisher("pwm_right", Int32, queue_size=50 )
    pub_left = rospy.Publisher("pwm_left", Int32, queue_size=50) 
    direc_flag = rospy.Publisher("direction", Int32, queue_size=50)
    pwmRight = abs((w_r/7.5)*255)
    pwmLeft = abs((w_l/7.5)*255)
    if (pwmRight > 255):
	pwmRight = 255
    elif (pwmRight < 0):
	pwmRight = 0
    if (pwmLeft > 255):
	pwmLeft = 255
    elif (pwmLeft < 0):
	pwmLeft = 0
    rospy.loginfo("PWM right %s" % pwmRight )
    rospy.loginfo("PWM left %s" % pwmLeft )
    if (v > 0):
          direction_flag = 1 #cw
    if (v < 0):
          direction_flag = 2 #ccw
    if (v == 0):
          direction_flag = 0
    pub_right.publish(pwmRight)
    pub_left.publish(pwmLeft)
    direc_flag.publish(direction_flag)
    rospy.loginfo("direction flag %s" % direction_flag )
def listener():
    
    rospy.init_node('vel_listener', anonymous=True)
    
    rospy.Subscriber("cmd_vel", Twist, callback)
    rospy.spin()
   
if __name__ == '__main__':
    listener()

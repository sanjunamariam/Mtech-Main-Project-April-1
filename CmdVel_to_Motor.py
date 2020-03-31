#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import RPi.GPIO as GPIO
import time
from math import sin, cos, pi

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

GPIO.setup(12, GPIO.OUT)
GPIO.setup(16, GPIO.OUT)
GPIO.setup(20, GPIO.OUT)
GPIO.setup(21, GPIO.OUT)

GPIO.setup(4, GPIO.IN, pull_up_down = GPIO.PUD_DOWN)
GPIO.setup(17, GPIO.IN, pull_up_down = GPIO.PUD_DOWN)
GPIO.setup(27, GPIO.IN, pull_up_down = GPIO.PUD_DOWN)
GPIO.setup(22, GPIO.IN, pull_up_down = GPIO.PUD_DOWN)

GPIO.setup(13,GPIO.OUT)
GPIO.setup(19,GPIO.OUT)
pwm1 = GPIO.PWM(13,1000) #configuring enable pin 1 right
pwm2 = GPIO.PWM(19,1000) #configuring enable pin 2 left

counter = 0

def brake():

    GPIO.output(12,GPIO.HIGH)
    GPIO.output(16,GPIO.HIGH)
    GPIO.output(20,GPIO.HIGH)
    GPIO.output(21,GPIO.HIGH)
   
    readEncoderValue()
    run = False
    

def moveccw(pwmRight,pwmLeft): 
    pwm1.ChangeDutyCycle(pwmRight)
    pwm2.ChangeDutyCycle(pwmLeft)
    GPIO.output(12,GPIO.HIGH)
    GPIO.output(16,GPIO.LOW)
    GPIO.output(20,GPIO.HIGH)
    GPIO.output(21,GPIO.LOW)
    readEncoderValue()
    run = True
    
def movecw(pwmRight,pwmLeft):
    pwm1.ChangeDutyCycle(pwmRight)
    pwm2.ChangeDutyCycle(pwmLeft)
    GPIO.output(12,GPIO.LOW)
    GPIO.output(16,GPIO.HIGH)
    GPIO.output(20,GPIO.LOW)
    GPIO.output(21,GPIO.HIGH)

    readEncoderValue()
    run = True

def readEncoderValue():
    
    encA1 = GPIO.input(4)
    encB1 = GPIO.input(17)
    encA2 = GPIO.input(27)
    encB2 = GPIO.input(22)
    print(encA1)
    print(encB1)
    print(encA2)
    print(encB2)
   
    global counter
    counter += 1
    print('count:')
    print(counter)
    
    if (encA1 == 0) and (encB1 ==  1):
        print('motor 1 right clockwise')
    elif (encA1 == 1) and (encB1 ==  0):
        print('motor 1 anticlockwise')
    else :
        print('motor 1 brake')
    if (encA2 == 0) and (encB2 ==  1):
        print('motor 2 left clockwise')
    elif (encA2 == 1) and (encB2 ==  0):
        print('motor 2 anticlockwise')
    else :
        print('motor 2 brake')


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

    ##pwmRight = (w_r/7.5)*255
    ##pwmLeft = (w_l/7.5)*255

    ##rospy.loginfo("PWM right %s" % pwmRight )
    ##rospy.loginfo("PWM left %s" % pwmLeft )

    #RASPBERRY
    pwmRight = abs((w_r/7.5)*100)
    pwmLeft = abs((w_l/7.5)*100)
    rospy.loginfo("PWM right %s" % pwmRight )
    rospy.loginfo("PWM left %s" % pwmLeft )
    #if angular vel is negative move ccw
    # if positive move cw
    # take mod / abs inside loop for the angular vel  .. if abs(angular velocity) == 7.5 --> 255 pwm signal... 

    if (w_r > 0):
          movecw(pwmRight,pwmLeft)
    if (w_r < 0):
          moveccw(pwmRight,pwmLeft)
    if (w_r == 0 and w_l == 0):
          brake()
     
def listener():
    
    rospy.init_node('vel_listener', anonymous=True)
    rospy.Subscriber("cmd_vel", Twist, callback)
    rospy.spin()
 
if __name__ == '__main__':
    listener()

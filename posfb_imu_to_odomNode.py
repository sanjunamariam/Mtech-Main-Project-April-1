#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import time
import tf
import math
from math import sin, cos, pi
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

def callback(data):
    #listen to imu topic
    vx = data.angular_velocity.x
    vy = data.angular_velocity.y
    vz = data.angular_velocity.z

    ax = data.linear_acceleration.x
    ay = data.linear_acceleration.y
    az = data.linear_acceleration.z

    orientation_x = data.orientation.x
    orientation_y = data.orientation.y
    orientation_z = data.orientation.z
    orientation_w = data.orientation.w

    rospy.loginfo("orientation_w %s" % orientation_w )
def listener():
    rospy.init_node('imu_to_odom_node', anonymous=True)
    current_time = rospy.Time.now()
    last_time = rospy.Time.now()
    odometry_pub = rospy.Publisher("modified_odom",Odometry, queue_size=50)
    rospy.Subscriber("imu", Imu, callback)
    
    #radius of wheel (rough value in m 4.5cm)
    wheel_rad = 0.045
    #distance between wheels 18cm
    dist_bw_wheels = 0.18

    previous_left_tick_encoder = 0
    previous_right_tick_encoder = 0
    actual_x = 0;
    actual_y = 0;
    actual_theta = 0;
############################need to modify #############################
    ticks_for_full_wheel_rotation = 810 #from robu.in spec
    tick_left_encoder = 200
    tick_right_encoder = 50
    
#############################
    r = rospy.Rate(10.0)

    while not rospy.is_shutdown():
            current_time = rospy.Time.now()

	    # distance travelled in one tick 
	    #perimeter of wheel / number of tick counts for one rotation
	    distancePerCount = (2 * pi * wheel_rad)/ticks_for_full_wheel_rotation;
	    diff_tick_left = tick_left_encoder - previous_left_tick_encoder;
	    diff_tick_right = tick_right_encoder - previous_right_tick_encoder;

	    #diff denotes the no of additional ticks from last one.. 
	    #so diff * distance per count will give you distance from last..
	    # bt we need velocity.. so distance / time difference

	    diff_distance_left = diff_tick_left * distancePerCount;
	    diff_distance_right = diff_tick_right  * distancePerCount;

	    time_diff = (current_time-last_time).to_sec();
	    #need to verify whether linear or angular
	    feedback_wl = diff_distance_left/time_diff;
	    feedback_wr = diff_distance_right/time_diff;
	    #calculation of actual pose and linear angular velocities 
	    #from position encoded data #based on cpcr notes
	    actual_vehicle_linear_vel_x = (feedback_wr+feedback_wl)*wheel_rad/2;
	    actual_vehicle_linear_vel_y = 0;
	    actual_vehicle_angular_vel = (feedback_wr-feedback_wl)*wheel_rad/dist_bw_wheels;
	    
	    dx = actual_vehicle_linear_vel_x * cos(actual_theta);
	    dy = actual_vehicle_linear_vel_x * sin(actual_theta);
	    dtheta = actual_vehicle_angular_vel;

	    actual_x += dx;
	    actual_y += dy;
	    actual_theta += dtheta;
            
            odom_quat = tf.transformations.quaternion_from_euler(0, 0, actual_theta)
	    #publish odom
	    odom = Odometry()
	    odom.header.stamp = current_time
            odom.header.frame_id = "odom"

            odom.pose.pose = Pose(Point(actual_x, actual_y, 0.0), Quaternion(*odom_quat))
            odom.child_frame_id = "base_footprint"
            odom.twist.twist = Twist(Vector3(actual_vehicle_linear_vel_x, actual_vehicle_linear_vel_y, 0), Vector3(0, 0, actual_vehicle_angular_vel))
	    #publishing odometry data
	    odometry_pub.publish(odom);
	    #update for next loop
	
 
if __name__ == '__main__':
    listener()

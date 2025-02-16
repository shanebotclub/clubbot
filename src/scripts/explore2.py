#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

def move_forward():
    velocity = Twist()
    velocity.linear.x = 0.5
    velocity.angular.z = 0.0
    return velocity

def move_backward():
    velocity = Twist()
    velocity.linear.x = -0.5
    velocity.angular.z = 0.0
    return velocity    

def turn_left():
    velocity = Twist()
    velocity.linear.x = 0.0
    velocity.angular.z = 1.0
    return velocity

def turn_right():
    velocity = Twist()
    velocity.linear.x = 0.0
    velocity.angular.z = -1.0
    return velocity

def stop():
    velocity = Twist()
    velocity.linear.x = 0.0
    velocity.angular.z = 0.0
    return velocity


if __name__ == '__main__':
    rospy.init_node('explore2')
    rate = rospy.Rate(10)
    drive = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    
    while not rospy.is_shutdown():
        # rotate left 
        velocity = turn_left()
        drive.publish(velocity)
        rate.sleep()
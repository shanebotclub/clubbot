#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Range
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

def sonarL_callback(msg):
    sonarL = (int(msg.range))


def sonarLM_callback(msg):
    sonarLM = (int(msg.range))
    

def sonarRM_callback(msg):    
    sonarRM = (int(msg.range))
    

def sonarR_callback(msg):
    sonarR = (int(msg.range))

sonarL = rospy.Subscriber('snr_1', Range, sonarL_callback)
sonarLM = rospy.Subscriber('snr_2', Range, sonarLM_callback)
sonarRM = rospy.Subscriber('snr_3', Range, sonarRM_callback)
sonarR = rospy.Subscriber('snr_4', Range, sonarR_callback)
    


if __name__ == '__main__':
    rospy.init_node('explore4')
    rate = rospy.Rate(10)
    
    
    
    while not rospy.is_shutdown():
        
        
        print("sonarL: ", sonarL)
        print("sonarLM: ", sonarLM)
        print("sonarRM: ", sonarRM)
        print("sonarR: ", sonarR)
        
        rate.sleep()
    
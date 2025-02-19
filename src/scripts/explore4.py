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

sonarL = 0.0
sonarLM = 0.0
sonarRM = 0.0
sonarR = 0.0


def sonarL_callback(data):
    sonarL = (data.range)
    return sonarL


def sonarLM_callback(data):
    sonarLM = data.range
    return sonarLM
    

def sonarRM_callback(data):    
    sonarRM = data.range
    return sonarRM
    

def sonarR_callback(data):
    sonarR = data.range
    return sonarR


    


def main():
    rospy.init_node('explore4')
    rate = rospy.Rate(5)

    rospy.Subscriber('snr_1', Range, sonarL_callback)
    rospy.Subscriber('snr_2', Range, sonarLM_callback)
    rospy.Subscriber('snr_3', Range, sonarRM_callback)
    rospy.Subscriber('snr_4', Range, sonarR_callback)
    
    
    
    while not rospy.is_shutdown():
        

        print("sonarL: ", sonarL)
        print("sonarLM: ", sonarLM)
        print("sonarRM: ", sonarRM)
        print("sonarR: ", sonarR)
        
        rate.sleep()


if __name__ == '__main__':

    main()

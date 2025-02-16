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
    rospy.init_node('explore3')
    rate = rospy.Rate(10)
    drive = rospy.Publisher('cmd_vel', Twist, queue_size=10)
       
    while not rospy.is_shutdown():
    
        # Drive in a square, move forward for 1 seconds, rotate left for half a second ish, and then repeat the process.
        for i in range(4):
            for i in range(10):
                velocity = move_forward()
                drive.publish(velocity)
                rate.sleep()
            for i in range(4):
                velocity = turn_left()
                drive.publish(velocity)
                rate.sleep()
        velocity = stop()
        drive.publish(velocity)
        rate.sleep()
        rospy.signal_shutdown('Done')
    rospy.spin()


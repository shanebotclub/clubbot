#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

# define velocity functions for the robot
# These functions will be used to control the robot's movement
# by publishing velocity commands to the 'cmd_vel' topic
# The robot will move forward, backward, turn left, turn right, or stop
# depending on the function called
def move_forward(speed):
    velocity = Twist()
    velocity.linear.x = speed
    velocity.angular.z = 0.0
    return velocity

def move_backward(speed):
    velocity = Twist()
    velocity.linear.x = -speed
    velocity.angular.z = 0.0
    return velocity    

def turn_left(speed):
    velocity = Twist()
    velocity.linear.x = 0.0
    velocity.angular.z = speed
    return velocity

def turn_right(speed):
    velocity = Twist()
    velocity.linear.x = 0.0
    velocity.angular.z = -speed
    return velocity

def stop():
    velocity = Twist()
    velocity.linear.x = 0.0
    velocity.angular.z = 0.0
    return velocity


def main():
    # Initialize the ROS node
    rospy.init_node('just_motors')
    # Set the rate at which to publish messages
    rate = rospy.Rate(10)
    # Create a publisher to send velocity commands to the robot
    drive = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    
    while not rospy.is_shutdown():
        # set the velocity to move forward at a speed of 0.5
        # and publish it to the 'cmd_vel' topic
        velocity = move_forward(0.5)
        drive.publish(velocity)
        rate.sleep()

if __name__ == '__main__':
    main()
    rospy.spin()
    
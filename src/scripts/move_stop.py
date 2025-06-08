#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

linear_speed = 0.3  # Set the linear speed for the robot
angular_speed = 0.0  # Set the angular speed for the robot

def drive_robot(linear_speed, angular_speed):
    velocity = Twist
    velocity.linear.x = linear_speed
    velocity.angular.z = angular_speed
    return velocity

def main():
    global linear_speed, angular_speed
    # Initialize the ROS node
    rospy.init_node('move_stop')
    
    # Create a publisher to send velocity commands to the robot
    drive = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    
    # Set the rate at which to publish messages
    rate = rospy.Rate(1)
    
    while not rospy.is_shutdown():
        # Get the velocity command for driving the robot
       
        velocity = drive_robot(linear_speed, angular_speed)
        print("moveing forward")
        
        # Publish the velocity command
        drive.publish(velocity)
        
        # Sleep to maintain the loop rate
        rate.sleep()

        linear_speed = 0.0  # Stop the robot
        angular_speed = 0.0  # Stop the robot
        velocity = drive_robot(linear_speed, angular_speed)
        drive.publish(velocity)  # Publish the stop command
        print("robot stopped")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        print("ROS node interrupted. Exiting...")

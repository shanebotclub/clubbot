#!/usr/bin/env python3

# Robot will start stopped and move when back bumper is pressed
# Robot will stop when front bumper is pressed
# testing commit to github

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

move = False

LF_bumper = False
MF_bumper = False
RF_bumper = False

LB_bumper = False
MB_bumper = False
RB_bumper = False



def drive_robot(linear_speed, angular_speed):
    velocity = Twist()
    velocity.linear.x = linear_speed
    velocity.angular.z = angular_speed
    return velocity

# define callback functions for each bumper
def LF_bumper_callback(data):
    global move
    global LF_bumper
    LF_bumper = data.data
    move = False

def MF_bumper_callback(data):
    global move
    global MF_bumper
    MF_bumper = data.data
    move = False

def RF_bumper_callback(data):
    global move
    global RF_bumper
    RF_bumper = data.data
    move = False

def LB_bumper_callback(data):
    global move
    global LB_bumper
    LB_bumper = data.data
    move = True

def MB_bumper_callback(data):
    global move
    global MB_bumper
    MB_bumper = data.data
    move = True

def RB_bumper_callback(data):
    global move
    global RB_bumper
    RB_bumper = data.data
    move = True


# define functions to subscribe to each bumper sensor
def left_front_bumper():
    rospy.Subscriber("bpr_lf", Bool, LF_bumper_callback)

def middle_front_bumper():
    rospy.Subscriber("bpr_mf", Bool, MF_bumper_callback)

def right_front_bumper():
    rospy.Subscriber("bpr_rf", Bool, RF_bumper_callback)

def left_back_bumper():
    rospy.Subscriber("bpr_lb", Bool, LB_bumper_callback)

def middle_back_bumper():
    rospy.Subscriber("bpr_mb", Bool, MB_bumper_callback)

def right_back_bumper():
    rospy.Subscriber("bpr_rb", Bool, RB_bumper_callback)



def main():
    global linear_speed, angular_speed, move
    
    # Initialize the ROS node
    rospy.init_node('move_stop')
    
    # Create a publisher to send velocity commands to the robot
    drive = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    left_front_bumper()
    middle_front_bumper()
    right_front_bumper()
    left_back_bumper()
    middle_back_bumper()
    right_back_bumper()
    
    # Set the rate at which to publish messages
    rate = rospy.Rate(20)
    
    while not rospy.is_shutdown():



        if move == False:
            print("robot stopped")
            velocity = drive_robot(0.0, 0.0)
            drive.publish(velocity)
            #rate.sleep()
            
        # Get the velocity command for driving the robot
       
        if move:
            # If no bumpers are pressed, move forward
            velocity = drive_robot(0.3, 0.0)
            print("moveing forward")
            
            # Publish the velocity command
            drive.publish(velocity)
            #rate.sleep()
        
        rate.sleep()
        print("move:", move)
       

        
        
    

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        print("ROS node interrupted. Exiting...")

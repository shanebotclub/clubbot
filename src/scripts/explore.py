#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from std_msgs.msg import Bool

# set up variables for bumpers
LF_bumper = False
MF_bumper = False
RF_bumper = False

LB_bumper = False
MB_bumper = False
RB_bumper = False

# set up variables for sonar
sonarL = 0.0
sonarLM = 0.0
sonarR = 0.0
sonarRM = 0.0

turn_velocity = 0.1



def main():
    # Initialize the ROS node
    rospy.init_node('explore')

    # Set the rate at which to publish messages
    rate = rospy.Rate(10)

    # Create a publisher to send velocity commands to the robot
    drive = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    # Subscribe to sonar and bumper sensors
    lSonar()
    rSonar()
    lMiddleSonar()
    rMiddleSonar()

    left_front_bumper()
    middle_front_bumper()
    right_front_bumper()
    left_back_bumper()
    middle_back_bumper()
    right_back_bumper()

    while not rospy.is_shutdown():
        
        # make dictionary of sonar values and sort them from lowest to highest
        def func(input):
            return input['value'] 
        sonarsDict = [{'sonar': 'sonarL', 'value': sonarL}, {'sonar': 'sonarLM', 'value': sonarLM}, {'sonar': 'sonarR', 'value': sonarR}, {'sonar': 'sonarRM', 'value': sonarRM}]
        print("raw sonar: ", sonarsDict)
        
        # replace 0 values with 9999
        for sonar in range(len(sonarsDict)):
            if sonarsDict[sonar]['value'] == 0:
                sonarsDict[sonar]['value'] = 9999
        print("sonar no zeros: ", sonarsDict)
        
        sonarsSorted = sonarsDict
        sonarsSorted.sort(key=func)
        print("sonar sorted: ", sonarsSorted)
        #print('\n', sonarsSorted)
        #print(sonarsSorted[0]['value'])
        #print(sonarsSorted[0]['sonar'])

        if sonarsSorted[0]['value'] < 40:
            if sonarsSorted[3]['sonar'] == 'sonarL':
                velocity = turn_left(turn_velocity)
                
            elif sonarsSorted[3]['sonar'] == 'sonarR':
                velocity = turn_right(turn_velocity)
                
            elif sonarsSorted[3]['sonar'] == 'sonarLM':
                velocity = turn_left(turn_velocity)
                
            elif sonarsSorted[3]['sonar'] == 'sonarRM':
                velocity = turn_right(turn_velocity)
                
        
        else:
            velocity = move_forward(0.3)

        drive.publish(velocity)

    

       


# define velocity functions for the robot
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


# define callback functions for each sonar sensor
def sonarL_callback(data):
    global sonarL
    sonarL = data.range

def sonarLM_callback(data):
    global sonarLM 
    sonarLM = data.range

def sonarR_callback(data):
    global sonarR
    sonarR = data.range

def sonarRM_callback(data):
    global sonarRM
    sonarRM = data.range


# define functions to subscribe to each sonar sensor
def lSonar():
    rospy.Subscriber("snr_1", Range, sonarLM_callback)
    
def rSonar():
   rospy.Subscriber("snr_4", Range, sonarRM_callback)
   
def lMiddleSonar():
    rospy.Subscriber("snr_2", Range, sonarL_callback)
   
def rMiddleSonar():
    rospy.Subscriber("snr_3", Range, sonarR_callback)


# define callback functions for each bumper sensor
def LF_bumper_callback(data):
    global LF_bumper
    LF_bumper = data.data

def MF_bumper_callback(data):
    global MF_bumper
    MF_bumper = data.data

def RF_bumper_callback(data):
    global RF_bumper
    RF_bumper = data.data

def LB_bumper_callback(data):
    global LB_bumper
    LB_bumper = data.data

def MB_bumper_callback(data):
    global MB_bumper
    MB_bumper = data.data

def RB_bumper_callback(data):
    global RB_bumper
    RB_bumper = data.data


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


if __name__ == '__main__':
    main()
    
    rospy.spin()
   
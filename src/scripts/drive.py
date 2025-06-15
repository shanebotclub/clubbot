#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from sensor_msgs.msg import Range


def main():
    global linear_speed, angular_speed, move
    
    # Initialize the ROS node
    rospy.init_node('drive')
    
    # Create a publisher to send velocity commands to the robot
    drive = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    rate = rospy.Rate(20)

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

    
    left_front_bumper()
    middle_front_bumper()
    right_front_bumper()
    left_back_bumper()
    middle_back_bumper()
    right_back_bumper()
    
    # Set the rate at which to publish messages
    rate = rospy.Rate(20)
    
    while not rospy.is_shutdown():

        # make dictionary of sonar values and sort them from lowest to highest
        def func(input):
            return input['value'] 
        sonarsDict = [{'sonar': 'sonarL', 'value': sonarL}, {'sonar': 'sonarLM', 'value': sonarLM}, {'sonar': 'sonarR', 'value': sonarR}, {'sonar': 'sonarRM', 'value': sonarRM}]
        #print("raw sonar: ", sonarsDict)
        
        # replace 0 values with 9999
        for sonar in range(len(sonarsDict)):
            if sonarsDict[sonar]['value'] == 0:
                sonarsDict[sonar]['value'] = 9999
        #print("sonar no zeros: ", sonarsDict)
        
        sonarsSorted = sonarsDict
        sonarsSorted.sort(key=func)

        #print("sonar sorted: ", sonarsSorted)

        Fspeed = sonarsSorted[0]['value'] / 100.0  # convert to meters
        if Fspeed >= 0.5:
            Fspeed = 0.3
        if Fspeed <= 0.15:
            Fspeed = 0.0



        if move == False:
            print("robot stopped")
            velocity = drive_robot(0.0, 0.0)
            drive.publish(velocity)
            break
            
            
        # Get the velocity command for driving the robot
       
        if move:
            # If no bumpers are pressed, move forward speed determined by sonar
            velocity = drive_robot(Fspeed, 0.0)
            print("moveing forward")
            
            # Publish the velocity command
            drive.publish(velocity)
           
        
        rate.sleep() # nessesery to controll motors correctly?
        
        print("forward speed: ", Fspeed)
       

move = True

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
    move = False

def MB_bumper_callback(data):
    global move
    global MB_bumper
    MB_bumper = data.data
    move = False

def RB_bumper_callback(data):
    global move
    global RB_bumper
    RB_bumper = data.data
    move = False


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
        
        
    

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        print("ROS node interrupted. Exiting...")

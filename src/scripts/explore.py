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

rb_activated = 0
lb_activated = 0

# set up variables for sonar
sonarL = 0.0
sonarLM = 0.0
sonarR = 0.0
sonarRM = 0.0
# set up variables for velocities
forward_velocity = 0.15
turn_velocity = 0.03
forward_turn_velocity = 0.03
velocity = 0
move = True
lb_activated = 0
rb_activated = 0



def main():
    global move, velocity, lb_activated, rb_activated
    
    # Initialize the ROS node
    rospy.init_node('explore')

    # Set the rate at which to publish messages
    rate = rospy.Rate(20)

    # Create a publisher to send velocity commands to the robot
    drive = rospy.Publisher('cmd_vel', Twist, latch=True, queue_size=1)

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

        while move:
            # record the last velocity to check it has changed before publishing (it is best to only publish when the velocity changes)
            last_velocity = velocity
        
            # make dictionary of sonar values and sort them from lowest to highest
            def func(input):
                return input['value'] 
            sonarsDict = [{'sonar': 'sonarL', 'value': sonarL}, {'sonar': 'sonarLM', 'value': sonarLM}, {'sonar': 'sonarRM', 'value': sonarRM}, {'sonar': 'sonarR', 'value': sonarR}]
            #print("raw sonar: ", sonarsDict)
            
            # replace 0 values with 9999
            for sonar in range(len(sonarsDict)):
                if sonarsDict[sonar]['value'] == 0:
                    sonarsDict[sonar]['value'] = 9999
            #print("sonar no zeros: ", sonarsDict)
            
            sonarsSorted = sonarsDict
            sonarsSorted.sort(key=func)
            print("sonar sorted: ", sonarsSorted)
            #print('\n', sonarsSorted)
            #print(sonarsSorted[0]['value'])
            #print(sonarsSorted[0]['sonar'])

            if sonarsSorted[0]['value'] < 40:
                if sonarsSorted[0]['sonar'] == 'sonarL':
                    velocity = drive_robot(forward_turn_velocity, -turn_velocity)
                    print("Left Sonar = ", sonarsSorted[0]['value'], "cm, rotate right")
                    
                    
                elif sonarsSorted[0]['sonar'] == 'sonarR':
                    velocity = drive_robot(forward_turn_velocity, turn_velocity)
                    print("Right Sonar = ", sonarsSorted[3]['value'], "cm, rotate left")
                    
                    
                elif sonarsSorted[0]['sonar'] == 'sonarLM':
                    velocity = drive_robot(forward_turn_velocity, -turn_velocity,)
                    print("Left Middle Sonar = ", sonarsSorted[0]['value'], "cm, rotate right")
                    
                    
                elif sonarsSorted[0]['sonar'] == 'sonarRM':
                    velocity = drive_robot(forward_turn_velocity, turn_velocity)
                    print("Right Middle Sonar = ", sonarsSorted[0]['value'], "cm, rotate left")
                   
                    
            
            # stop if any bumper is pressed
            elif LF_bumper or MF_bumper:  
                lb_activated = 8
                
                
            elif RF_bumper:
                rb_activated = 8
                
                
            # if any back bumper is pressed, stop the robot
            elif LB_bumper or MB_bumper or RB_bumper:
                velocity = drive_robot(0.0, 0.0)
                rospy.loginfo("Back bumper pressed, stopping robot")
                move = False
                
            elif lb_activated > 0 or rb_activated > 0:
                reverse_and_turn()
                
                
                
                


            # if no bumpers are pressed and no sonar is too close, move forward 
            else:
                velocity = drive_robot(forward_velocity, 0.0)
                

            if last_velocity != velocity:
                drive.publish(velocity)
                

            rate.sleep()

        # if move is False, stop the robot
        if not move:
            print("robot stopped")
            velocity = drive_robot(0.0, 0.0)
            drive.publish(velocity)
            rate.sleep()
            break
       
    
    

       





 # define velocity functions for the robot
def drive_robot(linear_speed, angular_speed):
    velocity = Twist()
    velocity.linear.x = linear_speed
    velocity.angular.z = angular_speed
    print("drive")
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
    rospy.Subscriber("snr_1", Range, sonarL_callback)
    
def rSonar():
   rospy.Subscriber("snr_4", Range, sonarR_callback)
   
def lMiddleSonar():
    rospy.Subscriber("snr_2", Range, sonarLM_callback)
   
def rMiddleSonar():
    rospy.Subscriber("snr_3", Range, sonarRM_callback)


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
    
# functions to reverse and turn when a bumper is pressed
def reverse_and_turn():
    global lb_activated, rb_activated, velocity
    if lb_activated > 1:
        velocity = drive_robot(-forward_velocity, 0.0)
        lb_activated -= 1
    elif rb_activated > 1:
        velocity = drive_robot(-forward_velocity, 0.0)
        rb_activated -= 1
    
    elif lb_activated == 1:
        velocity = drive_robot(0.0, turn_velocity)
        lb_activated = 0
    elif rb_activated == 1:
        velocity = drive_robot(0.0, -turn_velocity)
        rb_activated = 0
        



if __name__ == '__main__':
    main()
    # Keep the node running until it is shut down
    
    rospy.spin()
   
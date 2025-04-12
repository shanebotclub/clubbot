#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Range

# Initialize sonar variables
# These variables will store the distance readings from the sonar sensors
sonarL = 0.0
sonarLM = 0.0
sonarR = 0.0
sonarRM = 0.0

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
    



def main():
    # Initialize the ROS node
    rospy.init_node('just_sonar')

    while not rospy.is_shutdown():
        # get sonar readings
        lSonar()
        rSonar()
        lMiddleSonar()
        rMiddleSonar()

        # Print the sonar readings
        print("Left Sonar: ", sonarL)
        print("Left Middle Sonar: ", sonarLM)
        print("Right Middle Sonar: ", sonarRM)
        print("Right Sonar: ", sonarR)

        # Sleep for a short duration to avoid spamming the console
        rospy.sleep(1)

    rospy.spin()



if __name__ == '__main__':

    main()


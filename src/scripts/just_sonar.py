#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Range

#sonarLM = 0.0



def sonar_callback(data):
    
    print("Sonar distance:", data.range)
    #sonarLM = data.range
    #return sonarLM



def main():

    rospy.init_node('just_sonar')

    rospy.Subscriber("snr_2", Range, sonar_callback)
    #sonarLM = rospy.Subscriber("snr_2", Range, sonar_callback)
    #print("Sonar distance: ", sonarLM)

    rospy.spin()



if __name__ == '__main__':

    main()


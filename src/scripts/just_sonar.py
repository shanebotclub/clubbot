#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Range

sonarLM = 0.0



def sonar_callback(data):
    global sonarLM 
    sonarLM = data.range
  



def main():

    rospy.init_node('just_sonar')

    rospy.Subscriber("snr_2", Range, sonar_callback)
    while not rospy.is_shutdown():
        print("Sonar distance: ", sonarLM)

    rospy.spin()



if __name__ == '__main__':

    main()


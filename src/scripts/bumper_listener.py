#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool

#bumper = False

def bumper_callback(data):
    
    print("Bumper status:", data.data)
    #bumper = data.data


    

def main():

    rospy.init_node('bumper_listener')

    rospy.Subscriber("bpr_lf", Bool, bumper_callback)

    #print("Bumper status: ", bumper)

    rospy.spin()

if __name__ == '__main__':
    main()

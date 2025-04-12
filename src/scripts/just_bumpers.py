#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool

LF_bumper = False
MF_bumper = False
RF_bumper = False

LB_bumper = False
MB_bumper = False
RB_bumper = False


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

    rospy.init_node('just_bumpers')

    while not rospy.is_shutdown():
        left_front_bumper()
        middle_front_bumper()
        right_front_bumper()
        left_back_bumper()
        middle_back_bumper()
        right_back_bumper()
        
        # Print the bumper readings
        print("Left front bumper status: ", LF_bumper)
        print("Middle front bumper status: ", MF_bumper)
        print("Right front bumper status: ", RF_bumper)
        print("Left back bumper status: ", LB_bumper)
        print("Middle back bumper status: ", MB_bumper)
        print("Right back bumper status: ", RB_bumper)

        # Sleep for a short duration to slow down the printout
        rospy.sleep(1)

    rospy.spin()


if __name__ == '__main__':
    main()

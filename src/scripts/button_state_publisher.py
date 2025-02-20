#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool
import RPi.GPIO as GPIO

GREEN_BUTTON_GPIO = 6
BLUE_BUTTON_GPIO = 19
YELLOW_BUTTON_GPIO = 26
RED_BUTTON_GPIO = 4

if __name__ == '__main__':
    rospy.init_node('button_state_publisher')

    g_pub = rospy.Publisher('g_button_state',Bool, queue_size = 10)
    b_pub = rospy.Publisher('b_button_state',Bool, queue_size = 10)
    y_pub = rospy.Publisher('y_button_state',Bool, queue_size = 10)
    r_pub = rospy.Publisher('r_button_state',Bool, queue_size = 10)
    
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(GREEN_BUTTON_GPIO, GPIO.IN, pull_up_down = GPIO.PUD_UP)
    GPIO.setup(BLUE_BUTTON_GPIO, GPIO.IN, pull_up_down = GPIO.PUD_DOWN)
    GPIO.setup(YELLOW_BUTTON_GPIO, GPIO.IN, pull_up_down = GPIO.PUD_DOWN)
    GPIO.setup(RED_BUTTON_GPIO, GPIO.IN, pull_up_down = GPIO.PUD_UP)  

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        green_state = not GPIO.input(GREEN_BUTTON_GPIO)
        blue_state = not GPIO.input(BLUE_BUTTON_GPIO)
        yellow_state = not GPIO.input(YELLOW_BUTTON_GPIO)
        red_state = not GPIO.input(RED_BUTTON_GPIO)
        
        
        g_pub.publish(green_state)
        b_pub.publish(blue_state)
        y_pub.publish(yellow_state)
        r_pub.publish(red_state)
        
        if green_state == 1:
            print("green button true")

        if blue_state == 1:
            print("blue button true")

        if yellow_state == 1:
            print("yellow button true")

        if red_state == 1:
            print("red button true")
        
        rate.sleep()

    GPIO.cleanup()



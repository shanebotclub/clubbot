#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool
import RPi.GPIO as GPIO

GREEN_LED_GPIO = 12
BLUE_LED_GPIO = 16
YELLOW_LED_GPIO = 20
#RED_LED_GPIO = 14

def g_button_state_callback(msg):
    GPIO.output(GREEN_LED_GPIO, msg.data)

def b_button_state_callback(msg):
    GPIO.output(BLUE_LED_GPIO, msg.data)

def y_button_state_callback(msg):
    GPIO.output(YELLOW_LED_GPIO, msg.data)

#def r_button_state_callback(msg):
    #GPIO.output(RED_LED_GPIO, msg.data)
    

if __name__ == '__main__':
    rospy.init_node('led_actuator')

    GPIO.setmode(GPIO.BCM)
    GPIO.setup(GREEN_LED_GPIO, GPIO.OUT)
    GPIO.setup(BLUE_LED_GPIO, GPIO.OUT)
    GPIO.setup(YELLOW_LED_GPIO, GPIO.OUT)
    #GPIO.setup(RED_LED_GPIO, GPIO.OUT)

    rospy.Subscriber('g_button_state', Bool, g_button_state_callback)
    rospy.Subscriber('b_button_state', Bool, b_button_state_callback)
    rospy.Subscriber('y_button_state', Bool, y_button_state_callback)
    #rospy.Subscriber('r_button_state', Bool, r_button_state_callback)
    
    

    rospy.spin()

    GPIO.cleanup()

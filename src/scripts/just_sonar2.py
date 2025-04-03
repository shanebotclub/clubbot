#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Range

class Sonar:
    def __init__(self):
        self.sonarLM = 0.0
        self.sonarRM = 0.0
        self.sonarL = 0.0
        self.sonarR = 0.0

    def sonarL_callback(self, data):
        self.sonarL = data.range
        return self.sonarL

    def sonarLM_callback(self, data):
        self.sonarLM = data.range
        return self.sonarLM

    def sonarRM_callback(self, data):
        self.sonarRM = data.range
        return self.sonarRM

    def sonarR_callback(self, data):
        self.sonarR = data.range
        return self.sonarR
    
    def main(self):
        rospy.init_node('just_sonar')
        rospy.Subscriber("snr_2", Range, self.sonarLM_callback)
        rospy.Subscriber("snr_3", Range, self.sonarRM_callback)
        rospy.Subscriber("snr_1", Range, self.sonarL_callback)
        rospy.Subscriber("snr_4", Range, self.sonarR_callback)
        print("Sonar distance: ", self.sonarLM)
        print("Sonar distance: ", self.sonarRM)
        print("Sonar distance: ", self.sonarL)
        print("Sonar distance: ", self.sonarR)
        rospy.spin()

if __name__ == '__main__':
    sonar = Sonar()
    sonar.main()
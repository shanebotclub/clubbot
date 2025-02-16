#! /usr/bin/env python3
# Base Controller based on Diff Drive motor node without
# monitoring the encoders which will be passed to an
# odometry/robot state controller/publisher.

import rospy
import math
from DCMotor import DCMotor as DCM
from std_msgs.msg import Float64
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist


class baseController:
    ''' Differential drive node for use with DCMotor script

        Subscribe to encoder readings from Arduino:
        enc_lf, enc_lb, enc_rf, enc_rb for state readings.
        Subscribe to control_effort for PID control of motors.
        Subscribe to /cmd_vel to accept twist messages.

        Publish state readings to PID package.
        Publish pwm to motors.
    '''

    def __init__(self):

        self._leftPWM = 0.0
        self._rightPWM = 0.0

        self.speed = 0.0
        self.spin = 0.0

        self._lmotorSub = rospy.Subscriber('lcontrol_effort', Float64, self.lmotorCB)
        self._rmotorSub = rospy.Subscriber('rcontrol_effort', Float64, self.rmotorCB)
        self._cmd_velSub = rospy.Subscriber('cmd_vel', Twist, self._cmd_vel_CB)

        self._lsetpointPub = rospy.Publisher('lsetpoint', Float64, queue_size=10)
        self._rsetpointPub = rospy.Publisher('rsetpoint', Float64, queue_size=10)
        self._lPIDenablePub = rospy.Publisher('lpid_enable', Bool, queue_size=10)
        self._rPIDenablePub = rospy.Publisher('rpid_enable', Bool, queue_size=10)

        rospy.loginfo("checking for Robot Parameters")
        robotParams = rospy.search_param('/Robot name')
        if robotParams:
            name = rospy.get_param('/Robot name')
            rospy.loginfo("Robot Parameters loaded for : %s", name)
            # Retrieve Parameters for physical properties of the robot.
            self._leftMaxRPM = rospy.get_param('/leftMaxRPM')
            self._rightMaxRPM = rospy.get_param('/rightMaxRPM')
            self._wheel_diameter = rospy.get_param('/wheel_diameter')
            self._wheel_base = rospy.get_param('/wheel_base')
            self._leftTPR = rospy.get_param('/leftTicksPerRotation')
            self._rightTPR = rospy.get_param('/rightTicksPerRotation')
            self._lfPin = rospy.get_param('/leftForwardPin')
            self._lbPin = rospy.get_param('/leftBackwardPin')
            self._rfPin = rospy.get_param('/rightForwardPin')
            self._rbPin = rospy.get_param('/rightBackwardPin')
        else:
            rospy.logerror("Robot Parameters not loaded")

        #  Set pin numbers of the differential motor pair:
        self._leftWheel = DCM(self._lfPin, self._lbPin)
        self._rightWheel = DCM(self._rfPin, self._rbPin)

        rospy.loginfo("Base Controller running")
        # Publish initial setpoints as zero
        self._lsetpointPub.publish(0.0)
        self._rsetpointPub.publish(0.0)

    def _cmd_vel_CB(self, msg):
        self.speed = msg.linear.x
        self.spin = msg.angular.z
        self._set_motor_speeds()

    def lmotorCB(self, lpwm):
        self._leftPWM += lpwm.data
        # print("Left PWM", self._leftPWM)
        self._leftPWM = max(min(self._leftPWM, 100), -100)
        self._leftWheel.run(self._leftPWM)

    def rmotorCB(self, rpwm):
        self._rightPWM += rpwm.data
        # print("Right PWM", self._rightPWM)
        self._rightPWM = max(min(self._rightPWM, 100), -100)
        self._rightWheel.run(self._rightPWM)

    def max_speed(self):
        '''Speed in meters per second at maximum RPM'''
        rpm = (self._leftMaxRPM + self._rightMaxRPM) / 2.0
        mps = rpm * math.pi * self._wheel_diameter / 60.0
        return mps

    def max_twist(self):
        '''Rotation in radians per second at maximum RPM'''
        return self.max_speed() / self._wheel_diameter

    def _set_motor_speeds(self):
        if self.speed == 0 and self.spin == 0:
            leftSetpoint = 0
            rightSetpoint = 0
            self._lPIDenablePub.publish(False)
            self._rPIDenablePub.publish(False)
            self._leftWheel.stop()
            self._rightWheel.stop()
        else:
            # Enable PID on both wheels
            self._lPIDenablePub.publish(True)
            self._rPIDenablePub.publish(True)
            #
            # Figure out the speed of each wheel based on spin: each wheel covers
            # self._wheel_base meters in one radian, so the target speed for each wheel
            # in meters per sec is spin (radians/sec) times wheel_base divided by
            # wheel_diameter
            #
            right_twist_mps = self.spin * self._wheel_base / self._wheel_diameter
            left_twist_mps = -1.0 * self.spin * self._wheel_base / self._wheel_diameter
            #
            # Now add in forward motion.
            #
            left_mps = self.speed + left_twist_mps
            right_mps = self.speed + right_twist_mps
            #
            # Convert meters/sec into RPM: for each revolution, a wheel travels pi * diameter
            # meters, and each minute has 60 seconds.
            #
            left_target_rpm = (left_mps * 60.0) / (math.pi * self._wheel_diameter)
            right_target_rpm = (right_mps * 60.0) / (math.pi * self._wheel_diameter)
            #
            # convert rpm to ticks per interval: interval is .1 seconds
            # 1 rotation is 990ticks
            #
            leftSetpoint = (left_target_rpm/600) * self._leftTPR
            rightSetpoint = (right_target_rpm/600) * self._rightTPR
        #
        # Publish setpoints
        #
        rospy.loginfo("Left  setpoint = %s", leftSetpoint)
        rospy.loginfo("Right setpoint = %s", rightSetpoint)
        self._lsetpointPub.publish(leftSetpoint)
        self._rsetpointPub.publish(rightSetpoint)


def main():
    rospy.init_node('base controller')
    rospy.loginfo("Base Controller starting")
    baseController()
    rospy.spin()


if __name__ == '__main__':
    main()


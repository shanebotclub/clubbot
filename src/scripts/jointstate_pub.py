#! /usr/bin/env python3
# Animate rviz/simulation robot using JointState.


import rospy
from math import pi
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState


def main():
    rospy.init_node('jointstate_publisher')
    rospy.loginfo("Joint State Publisher Node Started")
    Wheels = JointState()
    Wheels.name = ["left_wheel_to_motors", "right_wheel_to_motors"]
    Wheels.position = [0.0, 0.0]
    # Create wheel joint state publisher
    wheels = rospy.Publisher('Wheels', JointState, queue_size=10)
    # Look for robot Parameters
    robotParams = rospy.search_param('/Robot name')
    if robotParams:
        name = rospy.get_param('/Robot name')
        rospy.loginfo("Robot Parameters loaded for : %s", name)
        # Retrieve Parameters for physical properties of the robot.
        _leftTPR = rospy.get_param('/leftTicksPerRotation')
        _rightTPR = rospy.get_param('/rightTicksPerRotation')
    else:
        rospy.logwarn("Robot Parameters not loaded")

    _lstate = 0
    _rstate = 0

    def lstateCB(lstat):
        global _lstate
        _lstate = lstat.data

    def rstateCB(rstat):
        global _rstate
        _rstate = rstat.data

    lstateSub = rospy.Subscriber('lstate', Float64, lstateCB)
    rstateSub = rospy.Subscriber('rstate', Float64, rstateCB)

    def moveWheels():
        left_wheel_pos = (2 * pi/_leftTPR) * _lstate
        right_wheel_pos = (2 * pi/_rightTPR) * _rstate

        if left_wheel_pos > pi:
            left_wheel_pos = -pi
        if left_wheel_pos < -pi:
            left_wheel_pos = pi
        if right_wheel_pos > pi:
            right_wheel_pos = -pi
        if right_wheel_pos < -pi:
            right_wheel_pos = pi

        Wheels.position[0] = left_wheel_pos
        Wheels.position[1] = right_wheel_pos
        wheels.publish(Wheels)


if __name__ == '__main__':
    main()


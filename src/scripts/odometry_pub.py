#! /usr/bin/env python3
# odometry node


import rospy
import tf
from math import sin, cos, pi
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3


class odometry:
    ''' Node to oversee odometry information.
        Publishes odometry information for the navigation stack.
    '''

    def __init__(self):
        self._lstate = 0.0
        self._rstate = 0.0

        self.lstateSub = rospy.Subscriber('lstate', Float64, self.lstateCB)
        self.rstateSub = rospy.Subscriber('rstate', Float64, self.rstateCB)
        self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=10)
        self.odom_broadcaster = tf.TransformBroadcaster()

        rospy.loginfo("checking for Robot Parameters")
        robotParams = rospy.search_param('/Robot name')
        if robotParams:
            name = rospy.get_param('/Robot name')
            rospy.loginfo("Robot Parameters loaded for : %s", name)
            # Retrieve Parameters for physical properties of the robot.
            self._wheel_diameter = rospy.get_param('/wheel_diameter')
            self._wheel_base = rospy.get_param('/wheel_base')
            self._leftTPR = rospy.get_param('/leftTicksPerRotation')
            self._rightTPR = rospy.get_param('/rightTicksPerRotation')
        else:
            rospy.logerror("Robot Parameters not loaded")
        rospy.loginfo("Odometry Publisher Node running")

    def lstateCB(self, lstat):
        self._lstate = lstat.data

    def rstateCB(self, rstat):
        self._rstate = rstat.data

    def move_robot(self):
        x = 0.0
        y = 0.0
        th = 0.0

        vx = 0.0
        vy = 0.0
        vth = 0.0

        current_time = rospy.Time.now()
        last_time = rospy.Time.now()

        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            current_time = rospy.Time.now()
            lvx = (pi/self._leftTPR) * self._lstate  # (2 * pi/self._leftTPR) * self._lstate
            rvx = (pi/self._rightTPR) * self._rstate  # (2 * pi/self._rightTPR) * self._rstate
            vx = (lvx + rvx) / 2
            vth = -(lvx - rvx) / self._wheel_base
            # compute odometry in a typical way given the velocities of the robot
            dt = (current_time - last_time).to_sec()
            delta_x = (vx * cos(th)) * dt
            delta_y = (vx * sin(th)) * dt
            delta_th = vth * dt

            x += delta_x
            y += delta_y
            th += delta_th

            # since all odometry is 6DOF we'll need a quaternion created from yaw
            odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)

            # first, we'll publish the transform over tf
            self.odom_broadcaster.sendTransform(
                (x, y, 0.),
                odom_quat,
                current_time,
                "base_link",
                "odom"
            )

            # next, we'll publish the odometry message over ROS
            odom = Odometry()
            odom.header.stamp = current_time
            odom.header.frame_id = "odom"

            # set the position
            odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))

            # set the velocity
            odom.child_frame_id = "base_link"
            odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))

            # publish the message
            self.odom_pub.publish(odom)

            last_time = current_time
            rate.sleep()


def main():
    rospy.init_node('odometry_publisher')
    rospy.loginfo("Odometry Publisher Node Starting")
    odom = odometry()
    odom.move_robot()


if __name__ == '__main__':
    main()


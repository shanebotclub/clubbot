#!/usr/bin/env python3
import rospy
import smbus
import math
from sensor_msgs.msg import Imu

# MPU6050 Registers
MPU6050_ADDR = 0x68
PWR_MGMT_1   = 0x6B
ACCEL_XOUT_H = 0x3B

bus = smbus.SMBus(1)

def read_word(reg):
    high = bus.read_byte_data(MPU6050_ADDR, reg)
    low  = bus.read_byte_data(MPU6050_ADDR, reg + 1)
    value = (high << 8) + low
    if value >= 0x8000:
        value = -((65535 - value) + 1)
    return value

def init_mpu():
    bus.write_byte_data(MPU6050_ADDR, PWR_MGMT_1, 0)  # wake up sensor

def mpu6050_node():
    rospy.init_node("mpu6050_imu")
    pub = rospy.Publisher("/imu/data_raw", Imu, queue_size=10)
    rate = rospy.Rate(50)  # 50 Hz

    imu_msg = Imu()
    imu_msg.header.frame_id = "imu_link"

    accel_scale = 16384.0  # LSB/g for ±2g

    while not rospy.is_shutdown():
        ax = read_word(ACCEL_XOUT_H) / accel_scale
        ay = read_word(ACCEL_XOUT_H + 2) / accel_scale
        az = read_word(ACCEL_XOUT_H + 4) / accel_scale

        imu_msg.header.stamp = rospy.Time.now()

        imu_msg.linear_acceleration.x = ax * 9.81
        imu_msg.linear_acceleration.y = ay * 9.81
        imu_msg.linear_acceleration.z = az * 9.81

        # Gyro not used here, set to zero
        imu_msg.angular_velocity.x = 0.0
        imu_msg.angular_velocity.y = 0.0
        imu_msg.angular_velocity.z = 0.0

        pub.publish(imu_msg)
        rate.sleep()

if __name__ == "__main__":
    init_mpu()
    try:
        mpu6050_node()
    except rospy.ROSInterruptException:
        pass

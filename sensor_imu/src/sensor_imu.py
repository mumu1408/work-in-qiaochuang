#!/usr/bin/env python
#-*- coding: utf-8 -*-

import rospy
import serial
import numpy as np
import math
from sensor_msgs.msg import Imu

HEADER = 0x55
ACC_FLAG = 0x51
ANGULAR_VELOCITY_FLAG = 0X52
ANGLE_FLAG = 0X53
RANGE_ACC = 16.0
RANGE_ANGULAR_VELOCITY = 2000
RANGE_ANGLE = 180
RANGE_DICT = {ACC_FLAG: RANGE_ACC, ANGULAR_VELOCITY_FLAG: RANGE_ANGULAR_VELOCITY, ANGLE_FLAG: RANGE_ANGLE}

def hexShow(argv): 
    result = '' 
    hLen = len(argv) 
    for i in xrange(hLen): 
        hvol = ord(argv[i]) 
        hhex = '%02x'%hvol 
        result += hhex+' '
    print 'hexShow:',result 

def frame_data_process(data_frame):
    sum = HEADER
    frame_len = len(data_frame)
    data_temp = [0, 0, 0, 0]
    for i in range(frame_len - 1):
        sum = sum + ord(data_frame[i])
    if np.uint8(sum) == ord(data_frame[9]):
        print "数据帧正确"
        data_temp[0] = (np.short(ord(data_frame[2])<<8)|ord(data_frame[1]))/32768.0*RANGE_DICT[ord(data_frame[0])]
        data_temp[1] = (np.short(ord(data_frame[4])<<8)|ord(data_frame[3]))/32768.0*RANGE_DICT[ord(data_frame[0])]
        data_temp[2] = (np.short(ord(data_frame[6])<<8)|ord(data_frame[5]))/32768.0*RANGE_DICT[ord(data_frame[0])]
    else:
        print "数据帧错误"
        data_temp[3] = 1
    print data_temp
    return data_temp

def eular_to_quaternion(angle_data):
    quaternion_temp = [0, 0, 0, 0]
    roll = angle_data[0] * math.pi / 180.0
    pitch = angle_data[1] * math.pi / 180.0
    yaw = angle_data[2] * math.pi / 180.0
    quaternion_temp[0] = math.cos(0.5*roll)*math.cos(0.5*pitch)*math.cos(0.5*yaw) + math.sin(0.5*roll)*math.sin(0.5*pitch)*math.sin(0.5*yaw)
    quaternion_temp[1] = math.sin(0.5*roll)*math.cos(0.5*pitch)*math.cos(0.5*yaw) - math.cos(0.5*roll)*math.sin(0.5*pitch)*math.sin(0.5*yaw)
    quaternion_temp[2] = math.cos(0.5*roll)*math.sin(0.5*pitch)*math.cos(0.5*yaw) + math.sin(0.5*roll)*math.cos(0.5*pitch)*math.sin(0.5*yaw)
    quaternion_temp[3] = math.cos(0.5*roll)*math.cos(0.5*pitch)*math.sin(0.5*yaw) - math.sin(0.5*roll)*math.sin(0.5*pitch)*math.cos(0.5*yaw)
    print roll, pitch, yaw
    print quaternion_temp
    return quaternion_temp

def sensor_imu_publisher():
    pub = rospy.Publisher('imu_data', Imu, queue_size=10)
    rospy.init_node('sensor_imu', anonymous=True)

    ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
    imu_data = Imu()
    imu_data.header.frame_id = 'imu'
    rate = rospy.Rate(100) #100hz
    while not rospy.is_shutdown():   
        header = ser.read(1)
        hexShow(header)
        if ord(header) == HEADER:
            print "新的数据帧"
            data_frame = ser.read(10)
            hexShow(data_frame)
            if ord(data_frame[0]) == ACC_FLAG:
                print "加速度数据帧"
                acc_data = frame_data_process(data_frame)#单位g,即9.8m/s^2
                if acc_data[3] == 1:
                    continue
                imu_data.linear_acceleration.x = acc_data[0]
                imu_data.linear_acceleration.y = acc_data[1]
                imu_data.linear_acceleration.z = acc_data[2]
            elif ord(data_frame[0]) == ANGULAR_VELOCITY_FLAG:
                print "角速度数据帧"
                angular_velocity_data = frame_data_process(data_frame)#单位:度/s
                if angular_velocity_data[3] == 1:
                    continue
                imu_data.angular_velocity.x = angular_velocity_data[0]
                imu_data.angular_velocity.y = angular_velocity_data[1]
                imu_data.angular_velocity.z = angular_velocity_data[2]
            elif ord(data_frame[0]) == ANGLE_FLAG:
                print "姿态角数据帧"
                angle_data = frame_data_process(data_frame)#单位：度
                if angle_data[3] == 1:
                    continue
                quaternion_data = eular_to_quaternion(angle_data[0:3])#欧拉角转换为四元数
                imu_data.orientation.w = quaternion_data[0]
                imu_data.orientation.x = quaternion_data[1]
                imu_data.orientation.y = quaternion_data[2]
                imu_data.orientation.z = quaternion_data[3]
            else:
                continue
        else:
            print "不是包头"
            continue
        pub.publish(imu_data)
        rate.sleep()
    ser.flushInput()
    ser.close()

if __name__ == '__main__':
    try:
        sensor_imu_publisher()
    except rospy.ROSInterruptException:
        pass

#!/usr/bin/env python

# Copyright (c) 2012, Tang Tiong Yew
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the Willow Garage, Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import rospy
import serial
import string
import math
import sys

#from time import time
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
from tf.transformations import quaternion_from_euler

gps_heading = 0.0
pre_yaw = 0.0
combined_yaw = 0.0
degrees2rad = math.pi/180.0
imu_yaw_calibration = 0.0

first=1

def gps_heading_callback(msg):
    global gps_heading
    gps_heading = msg.data

def normalizing_angle(yaw_deg):
    if yaw_deg > 180.0:
        yaw_deg = yaw_deg - 360.0
    if yaw_deg < -180.0:
        yaw_deg = yaw_deg + 360.0
    return yaw_deg

rospy.init_node("razor_node")
#We only care about the most recent measurement, i.e. queue_size=1
pub = rospy.Publisher('imu', Imu, queue_size=1)
yaw_pub = rospy.Publisher('/current_yaw', Float64, queue_size=1)

imuMsg = Imu()

imuMsg.orientation_covariance = [
0.025, 0 , 0,
0, 0.025, 0,
0, 0, 0.025
]

imuMsg.angular_velocity_covariance = [
0.02, 0 , 0,
0 , 0.02, 0,
0 , 0 , 0.02
]

imuMsg.linear_acceleration_covariance = [
0.04 , 0 , 0,
0 , 0.04, 0,
0 , 0 , 0.04
]

default_port='/dev/ttyUSB2'
# Check your COM port and baud rate
rospy.loginfo("Opening %s...", default_port)
try:
    ser = serial.Serial(port=default_port, baudrate=921600, timeout=1)
except serial.serialutil.SerialException:
    rospy.logerr("IMU not found at port "+default_port + ". Check the order of port")
    #exit
    sys.exit(0)

roll=0
pitch=0
yaw=0
seq=0
accel_factor = 9.806    # Convert to m/s^2.
rospy.loginfo("Giving the EBIMU 5 seconds to boot...")
rospy.sleep(5) # Sleep for 5 seconds to wait for the board to boot

rospy.loginfo("Set yaw to 0 ")
ser.write('<00cmo>')
rospy.sleep(1)
rospy.loginfo("Angle velocity on")
rospy.sleep(1)
ser.write('<sog1>')
rospy.loginfo(ser.readline())
rospy.sleep(1)


rospy.loginfo("Publishing IMU data...")
#f = open("raw_imu_data.log", 'w')


while not rospy.is_shutdown():
    line = ser.readline()
    if len(line) == 0:
        rospy.loginfo("Waiting for IMU device to turn on...")
    else:
        rospy.loginfo(line)
        
            
    #f.write(line)                     # Write to the output log file
    words = string.split(line,",")    # Fields split
    if len(words) > 2:
        #in AHRS firmware z axis points down, in ROS z axis points up (see REP 103)
        if first ==1:
            yaw_deg = -float(words[3]) + (-45)
            combined_yaw = normalizing_angle(yaw_deg)
            combined_yaw = combined_yaw*degrees2rad
            
            first += 1
        else:
            yaw_deg = -float(words[3])
            normalizing_angle(yaw_deg) 
            yaw = yaw_deg*degrees2rad
        ## -----Solution 1 ----- ##
        combined_yaw += normalizing_angle((float(words[6])*0.04*(math.pi/180))+gps_heading)
        print("combined yaw : {}".format(combined_yaw))
        if abs(pre_yaw - combined_yaw)>1.06:
            combined_yaw = pre_yaw
        pre_yaw = combined_yaw
        #publish radian
        yaw_pub.publish(combined_yaw)
        #rospy.loginfo("successfully publishing yaw data")
        
        #in AHRS firmware y axis points right, in ROS y axis points left (see REP 103)
        pitch = -float(words[2])*degrees2rad
        roll = float(words[1])*degrees2rad

        # Publish message
        # AHRS firmware accelerations are negated
        # This means y and z are correct for ROS, but x needs reversing
        imuMsg.linear_acceleration.x = -float(words[7]) * accel_factor
        imuMsg.linear_acceleration.y = float(words[8]) * accel_factor
        imuMsg.linear_acceleration.z = float(words[9]) * accel_factor

        
        imuMsg.angular_velocity.x = float(words[4]) #* degrees2rad
        #in AHRS firmware y axis points right, in ROS y axis points left (see REP 103)
        imuMsg.angular_velocity.y = -float(words[5])# * degrees2rad
        #in AHRS firmware z axis points down, in ROS z axis points up (see REP 103) 
        imuMsg.angular_velocity.z = -float(words[6])# * degrees2rad

    q = quaternion_from_euler(roll,pitch,yaw)
    imuMsg.orientation.x = q[0]
    imuMsg.orientation.y = q[1]
    imuMsg.orientation.z = q[2]
    imuMsg.orientation.w = q[3]
    imuMsg.header.stamp= rospy.Time.now()
    imuMsg.header.frame_id = 'base_imu_link'
    imuMsg.header.seq = seq
    seq = seq + 1
    pub.publish(imuMsg)
    

ser.write('<00cmco>')        
ser.close

#f.close

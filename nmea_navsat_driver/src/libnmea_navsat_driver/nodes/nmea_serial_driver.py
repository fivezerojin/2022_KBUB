# Software License Agreement (BSD License)
#
# Copyright (c) 2013, Eric Perko
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the names of the authors nor the names of their
#    affiliated organizations may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Defines the main method for the nmea_serial_driver executable."""

import serial
import sys,os

import rospy

import datetime
from libnmea_navsat_driver.driver import RosNMEADriver


def main():
    """Create and run the nmea_serial_driver ROS node.

    Creates a ROS NMEA Driver and feeds it NMEA sentence strings from a serial device.

    :ROS Parameters:
        - ~port (str)
            Path of the serial device to open.
        - ~baud (int)
            Baud rate to configure the serial device.
    """
    rospy.init_node('nmea_serial_driver')

    serial_port = rospy.get_param('~port', '/dev/ttyUSB1')
    serial_baud = rospy.get_param('~baud', 115200)
    frame_id = RosNMEADriver.get_frame_id()
    
    file_path = "/home/"+os.getlogin()+"/GPS_LOG/"
    ask = raw_input("Do you want to log the GPS MESSAGES?(Y/N)")
    
    try:
        GPS = serial.Serial(port=serial_port, baudrate=serial_baud, timeout=2)
        nowDate = datetime.datetime.now()
        try:
            if ask == 'Y' or ask == 'y':
                f = open(file_path+"gps"+ nowDate.strftime("%Y-%m-%d_%H%M")+".ubx", "w")
                s = open(file_path+"gps_GGA"+ nowDate.strftime("%Y-%m-%d_%H%M")+".txt", "w")
                driver = RosNMEADriver()
                while not rospy.is_shutdown():
                    raw = GPS.readline()
                    if raw[3:6] == 'GGA':
                        rospy.loginfo(raw)
                        s.write(raw)
                    f.write(raw)
                    data = raw.strip()
                    try:
                        driver.add_sentence(data, frame_id)
                    except ValueError as e:
                        rospy.logwarn(  "Value error, likely due to missing fields in the NMEA message. "
                        "Error was: %s. Please report this issue at "
                        "github.com/ros-drivers/nmea_navsat_driver, including a bag file with the NMEA "
                        "sentences that caused it." %
                        e)
            elif ask == 'N' or ask == 'n':
                rospy.logwarn("GPS_LOG will not be saved")
                driver = RosNMEADriver()
                while not rospy.is_shutdown():
                    raw = GPS.readline()
                    if raw[3:6] == 'GGA':
                        rospy.loginfo(raw)
                    data = raw.strip()
                    try:
                        driver.add_sentence(data, frame_id)
                    except ValueError as e:
                        rospy.logwarn(
                        "Value error, likely due to missing fields in the NMEA message. "
                        "Error was: %s. Please report this issue at "
                        "github.com/ros-drivers/nmea_navsat_driver, including a bag file with the NMEA "
                        "sentences that caused it." %
                        e)
            else:
                rospy.logerr("INVALID ANSWER PLEASE INPUT 'Y','y' or 'N','n'")

        except (rospy.ROSInterruptException, serial.serialutil.SerialException):
            GPS.close()  # Close GPS serial port
            if f.is_open() and s.is_open():
                f.close()
                s.close()
                rospy.loginfo("gps log  create success")
            else:
                pass
    except serial.SerialException as ex:
        rospy.logfatal(
            "Could not open serial port: I/O error({0}): {1}".format(ex.errno, ex.strerror))

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

"""Defines the main method for the nmea_topic_serial_reader executable."""
import rospy
import serial
import string
import math


from geometry_msgs.msg import QuaternionStamped
from geometry_msgs.msg import TwistStamped
from nmea_msgs.msg import Sentence

from tf.transformations import quaternion_from_euler

from libnmea_navsat_driver.driver import RosNMEADriver

real_words = list()
real_words = [0.0 for i in range(15)]
VTG_data = list()
VTG_data = [0.0 for i in range(8)] # Initialize list to 0
VTG = list()
VTG = [0.0 for i in range(9)] # Initialize list to 0
VTG_saved = list()
VTG_saved = [0.0 for i in range(2)]# Initialize list to 0
deg2rad = 3.141592 / 180.0
rad2deg = 180 / 3.141592

ang = 0.0
mean = 0.0
first_ang = 0.0
first_run = 1
def main():
    """Create and run the nmea_topic_serial_reader ROS node.

    Opens a serial device and publishes data from the device as nmea_msgs.msg.Sentence messages.

    ROS parameters:
        ~port (str): Path of the serial device to open.
        ~baud (int): Baud rate to configure the serial device.

    ROS publishers:
        nmea_sentence (nmea_msgs.msg.Sentence): Publishes each line from the open serial device as a new
            message. The header's stamp is set to the rostime when the data is read from the serial device.
    """
    global real_words
    global first_run
    global ang
    rospy.init_node('nmea_topic_serial_reader')

    nmea_pub = rospy.Publisher("nmea_sentence", Sentence, queue_size = 1)
    heading_pub = rospy.Publisher("gps_heading", QuaternionStamped, queue_size = 1)
    vel_pub = rospy.Publisher("gps_vel", TwistStamped, queue_size = 1)
    
    serial_port = rospy.get_param('~port', '/dev/ttyUSB0')
    serial_baud = rospy.get_param('~baud', 115200)

    # Get the frame_id
    frame_id = RosNMEADriver.get_frame_id()

    try:
        GPS = serial.Serial(port=serial_port, baudrate=serial_baud, timeout=2)
        while not rospy.is_shutdown():
            data = GPS.readline().strip()
            #print(data)
            print('--------------------------------------------------------------------')
            
            #Confirm messages
            sentence = Sentence()
            heading_msg = QuaternionStamped()
            vel_msg = TwistStamped()

            sentence.header.stamp = rospy.get_rostime()
            sentence.header.frame_id = frame_id
            sentence.sentence = data
            #rate = rospy.Rate(8) # 8hz
            words = string.split(data,",")    # Fields split
            VTG_saved[0] = VTG_saved[1]
            if  words[0] == "$GNVTG":
                
                real_words = string.split(data,",")
                if not len(real_words[1]):
                    real_words[1] = 0.0
                if not len(real_words[7]):
                    real_words[7] = 0.0
            for i in range(0, 7, 1):
            
                VTG_data[i] = VTG_data[i + 1]
            
            VTG_data[7] = real_words[1]
            
            mean = 0.0
            
            for x in range(0, 8, 1):
                print("1111111111", VTG_data[x])
                mean += float(VTG_data[x])
            
            mean = -mean / 8.0;  # reverse clockwise
            
            if first_run == 1 and float(VTG[0]) != 0.0 and float(VTG_data[0]) != 0.0 :
            
                first_ang = mean
                first_run = 0
            
            if float(VTG[0]) == 0.0:
                print("Please move !\n")
                VTG_saved[1] = VTG_saved[0]
            
            else:
                VTG_saved[1] = mean
                ang = (float(VTG_saved[1]) - first_ang) * deg2rad  # degree to radian , (first angle - second angle)
                print("Your Heading = %.5f" % ang * rad2deg)
        
            #gps_heading
            q = quaternion_from_euler(0, 0, ang)
            heading_msg.quaternion.x = q[0]
            heading_msg.quaternion.y = q[1]
            heading_msg.quaternion.z = q[2]
            heading_msg.quaternion.w = q[3]
            heading_msg.header.frame_id = 'gps'
          
            #They must be published at the same time for Kalman filter!!!!!!!!
            
            heading_msg.header.stamp = rospy.Time.now()
            vel_msg.header.stamp = rospy.Time.now()



            #gps_vel
            vel_msg.header.frame_id = "gps_velocity"   
            # if not len(real_words[7]):
            #     real_words[7] = 0.0
            print("22222222222222222", real_words[7])
            gps_velocity = float(real_words[7]) / 3.6
        
            if gps_velocity < 0.06:     #Ignore the velocity when it is under 0.06m/s
        
                print("stop!!\n")
                vel_msg.twist.linear.x = 0
                vel_msg.twist.linear.y = 0
        
            else:
        
                vel_msg.twist.linear.x = gps_velocity * math.cos(ang * 180 / 3.141592)  #km/h  to  m/s
                vel_msg.twist.linear.y = gps_velocity * math.sin(ang * 180 / 3.141592)  #km/h  to  m/s
        
            print ("vel: %.5f m/s\n" % gps_velocity)
            print ("vel x: %.5f m/s\n" % vel_msg.twist.linear.x) 
            print ("vel y: %.5f m/s\n" % vel_msg.twist.linear.y)
            
            if (first_run == 1):
        
                print("You are not moving!!\n")
        
            else:
        
                print("first_ang : %lf\n" % first_ang)
        

            print("Your Heading = %.5f\n" % (ang * 180 / 3.141592))
            
            #publish messages
            nmea_pub.publish(sentence)
            heading_pub.publish(heading_msg)
            vel_pub.publish(vel_msg)

    except rospy.ROSInterruptException:
        GPS.close()  # Close GPS serial port

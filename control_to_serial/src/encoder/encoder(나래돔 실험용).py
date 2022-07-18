#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float32, Float64
from std_msgs.msg import Bool
import time
import sys

gear1=0.0
encoder=0.0
speed1 =0.0
control_mode = 0.0
steering =0
brake1=0
encoder_mode = 3
sleep = True

pub1 = rospy.Publisher("speed1", Float32, queue_size=10)
pub2 = rospy.Publisher("gear1", Float32, queue_size=10)   
pub3 = rospy.Publisher("steer1", Float32, queue_size=10) 
pub4 = rospy.Publisher("brake1", Float32, queue_size=10)

start_time, end_time = 0,0
first =1 

#메뉴얼 = 0, 오토 = 1 
enc = []
time_list = []

def callbackencoder_mode(data):
    global encoder_mode
    encoder_mode = data.data

def callbackcontrol_mode(data): #콜백잘됨
    global control_mode
    control_mode = data.data
    # print("control_mode :{}".format(control_mode))

def callbackencoder(data): #콜백잘됨
    global encoder    
    global sleep
    encoder = data.data
    if encoder_mode == 2:
        enc.append(encoder)
    elif encoder_mode == 1:
        if sleep == True:
            time.sleep(3)
            sleep = False
        if encoder >= enc[-2]-30:
            speed1 = 10 * 10
            delta = 0
            gear1 = 2
            brake1 = 0
            # pub1.publish(speed1)
            # pub2.publish(gear1)
            # pub3.publish(delta)
            # pub4.publish(brake1) 
        elif encoder < enc[-2]-30 and encoder >= enc[-2]-240:
            speed1 = 10 * 10
            delta = 28 * 71
            gear1 = 2
            brake1 = 0
            # pub1.publish(speed1)
            # pub2.publish(gear1)
            # pub3.publish(delta)
            # pub4.publish(brake1)     
        else:
            speed1 = 0
            delta = 0
            gear1 = 0
            brake1 = 200
            # pub1.publish(speed1)
            # pub2.publish(gear1)
            # pub3.publish(delta)
            # pub4.publish(brake1) 
        pub1.publish(speed1)
        pub2.publish(gear1)
        pub3.publish(delta)
        pub4.publish(brake1)

def main():
    global start_time,end_time, first
    rospy.init_node("encoder", anonymous=True)
    rospy.loginfo("HI")
        
    rospy.Subscriber("encoder", Float32, callbackencoder)  
    rospy.Subscriber("control_mode", Float32, callbackcontrol_mode)
    rospy.Subscriber("encoder_mode", Float64, callbackencoder_mode)
    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        rate.sleep()
    rospy.spin()
    
if __name__ == '__main__':
    try:
        main()
    except   rospy.ROSInterruptException:
        pass
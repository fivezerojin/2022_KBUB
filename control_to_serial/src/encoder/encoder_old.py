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
encoder_finish = False
enc_fin = False

pub1 = rospy.Publisher("speed1", Float32, queue_size=10)
pub2 = rospy.Publisher("gear1", Float32, queue_size=10)   
pub3 = rospy.Publisher("steer1", Float32, queue_size=10) 
pub4 = rospy.Publisher("brake1", Float32, queue_size=10)
pub5 = rospy.Publisher("encoder_finish", Bool, queue_size=10)

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
    global enc_fin
    global encoder_finish
    encoder = data.data
    if encoder_mode == 2:
        enc.append(encoder)
    elif encoder_mode == 1:
        print("1234123412ggpgpgp")
        if sleep == True:
            sleep = False
            time.sleep(2)
        if encoder >= enc[-2]-15:
            speed1 = 10 * 10
            steer = 0
            gear1 = 2
            brake1 = 0
            encoder_finish = False
          
        elif encoder < enc[-2]-15 and encoder >= enc[-2]-200:
            speed1 = 10 * 10
            steer = 28 * 71
            gear1 = 2
            brake1 = 0
            encoder_finish = False

        else:
            speed1 = 0
            steer = 0
            gear1 = 0
            brake1 = 200
            pub1.publish(speed1)
            pub2.publish(gear1)
            pub3.publish(steer)
            pub4.publish(brake1)
            rospy.sleep(2)
            pub5.publish(encoder_finish)
            encoder_finish = False
            enc_fin = True

        if enc_fin == True:
            encoder_finish = True
              
        pub1.publish(speed1)
        pub2.publish(gear1)
        pub3.publish(steer)
        pub4.publish(brake1)
        pub5.publish(encoder_finish)
    elif encoder_finish==True and encoder_mode==1:
        encoder_finish = True
        pub1.publish(speed1)
        pub2.publish(gear1)
        pub3.publish(steer)
        pub4.publish(brake1)
        pub5.publish(encoder_finish)
        

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
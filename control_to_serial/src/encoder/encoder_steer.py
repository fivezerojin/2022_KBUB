#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float32, Float64
from std_msgs.msg import Bool
import time
import sys

gear1=0.0
steer1=0.0
speed1 =0.0
control_mode = 0.0
steering =0
brake1=0
encoder_mode = 3 #비전에서 받아오는 값

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

def callbacksteer(data): #콜백잘됨
    global steer1    
    steer1 = data.data

def encoderrecord():
    global steer1
    enc.append(steer1)


def encoderread():
    global pub1,pub2,pub3,pub4
    print(444444444444444444444444)
    # print("time_List :{} ".format(time_list))

    speed1= 50.0
    #print(speed1)
    gear1 = 2.0
    #print(gear1)
    brake1 = 0.0

    for i in range(0, len(enc)):
        steering = -enc[i]

        dt = abs(round(end_time-start_time,2))
        print("Publishing datas")
        print(dt)
        if i==len(enc)-1:
            pub1.publish(0)
            pub2.publish(gear1)
            pub3.publish(steering)
            pub4.publish(200) 
            time.sleep(dt/float(len(enc)))
            continue            
        else:
            pub1.publish(speed1)
            pub2.publish(gear1)
            pub3.publish(steering)
            pub4.publish(brake1) 
            time.sleep(dt/float(len(enc)))
            print("time : {}", format(dt/float(len(enc))))
            continue

        # print(dt/float(len(enc)))
        


def main():
    global start_time,end_time, first
    rospy.init_node("encoder", anonymous=True)
    rospy.loginfo("HI")
    
    #rospy.Subscriber("gear", Float32, callbackgear)    
    rospy.Subscriber("steer", Float32, callbacksteer)  
    rospy.Subscriber("control_mode", Float32, callbackcontrol_mode)
    rospy.Subscriber("encoder_mode", Float64, callbackencoder)
    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        if control_mode == 0.0: #상황에 맞게 변경
            if first ==1:
                start_time = time.time()
                first +=1
            print("start record!!!")
            encoderrecord()
        elif control_mode ==1.0: #상황에 맞게 변경
            end_time = time.time()
            print("dt : {}".format(end_time-start_time))
            print(type(start_time))
            time.sleep(2)
            print("Recorded encoder : {}".format(enc))
            for i in range(0,9):
                {
                    enc.append(0)
                }
            enc.reverse()
            print("Reversed encoder : ",enc)
            # encoderread()
            # time.sleep(50)
            break
        else:
            #rospy.logerr("Wrong control mode received!")
            continue
        rate.sleep()
    encoderread()
    rospy.spin()
    
    
    


if __name__ == '__main__':
    try:
        main()
    except   rospy.ROSInterruptException:
        pass




#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import math
import random
# import matplotlib
# matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import os
import string
import time

from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
rospy.init_node('plotter', anonymous=True)

cx_s = []
cy_s = []
target_yaw, current_yaw = [], []

path = "/home/"+os.getlogin()+"/catkin_ws/src/kbub_pkg/src/map/"
file_list = os.listdir(path)
print(len(file_list))

txts = [file for file in file_list if file.endswith(".txt")]
for txt in txts:
    try:
        cx, cy = [], []
        with open("/home/usera/catkin_ws/src/kbub_pkg/src/map/"+txt, 'r') as f:
            while True:
                line = f.readline()
                if not line:
                    break
                if line.find('\n'):
                    line = line[:line.find('\n')]
                cxy = (line.split(','))
		    #create random color
                cx.append(float(cxy[0]))
                cy.append(float(cxy[1]))

            random.seed(time.time())
            r = random.random()
            g = random.random()
            b = random.random()
            random_rgb=(r,g,b)
            plt.plot(cx,cy,c=random_rgb,marker='.',markersize=5)
            plt.text(cx[int(len(cx)/2)]+0.2, cy[int(len(cy))/2]+0.2, txt)
    except:
        rospy.logerr("error detected at {}".format(txt))
            
    


def callback(msg):
    global x, y, cx_s, cy_s
    
    # plt.cla()
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    cx_s.append(x)
    cy_s.append(y)
    # plt.plot(cx,cy,'.r')
    # plt.plot(cx_s,cy_s,'-b')
    # plt.xlabel('x')
    # plt.ylabel('y')
    # plt.axis("equal")
    # plt.grid(True)
    # plt.draw()
    # plt.pause(0.000000001)
    # plt.show(block=True)

def plotter():

    # plt.figure(1)
    # plt.subplot(2,1,1)
    plt.plot(cx_s,cy_s, "-r")
    plt.xlabel('x')
    plt.ylabel('y')
    plt.axis("equal")
    plt.grid(True)

    # # plt.figure(2)
    # plt.subplot(2,1,2)
    # plt.plot(target_yaw,"-r",label='Target yaw')
    # plt.plot(current_yaw,"-b",label='Current yaw')
    # plt.legend()
    # plt.axis("equal")
    # plt.grid(True)

    plt.show()



def target_yaw_callback(msg):
    target_yaw.append(msg.data)

def current_yaw_callback(msg):
    current_yaw.append(msg.data)


def main():

    
    rospy.Subscriber("odom", Odometry, callback)
    rospy.Subscriber("current_yaw", Float64, current_yaw_callback)
    rospy.Subscriber("target_yaw", Float64, target_yaw_callback)

    # if rospy.is_shutdown():
    #     print('shutdown')
    #     plotter()

    rospy.spin()


if __name__ == '__main__':
    try:
        main()
        plotter()
    except KeyboardInterrupt:
    # Ctrl+C 입력시 예외 발생
        sys.exit()  # 종료


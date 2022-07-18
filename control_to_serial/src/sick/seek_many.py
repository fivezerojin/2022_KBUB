#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import math
# import matplotlib
# matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import os
import string
import time

from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
cx = []
cy = []
cx_s = []
cy_s = []
first = 1 


path = "/home/"+os.getlogin()+"/catkin_ws/src/txt_saver/map"
file_list = os.listdir(path)
print(len(file_list))


for i in range(len(file_list)-1):
	with open("/home/"+os.getlogin()+"/catkin_ws/src/kbub_pkg/src/map/"+str(i+1)+".txt", 'r') as f:
	    while True:
		line = f.readline()
		if not line:
		    break
		#print(line)
		if line.find('\n'):
		    line = line[:line.find('\n')]
		cxy = (line.split(','))
	      #  a.append([float(cxy[0]),float(cxy[1])])

		cx.append(float(cxy[0]))
		cy.append(float(cxy[1]))
#arrlen = len(cxy)
print(len(cx))


def callback(msg):
    global x, y, first, cx_s, cy_s
    print("callback respond!")

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    cx_s.append(x)
    cy_s.append(y)
    plt.cla()
            # for stopping simulation with the esc key.
    plt.gcf().canvas.mpl_connect('key_release_event',
                    lambda event: [exit(0) if event.key == 'escape' else None])
    plt.plot(cx, cy, ".r")
    plt.plot(x, y, "bo", markersize=3)
    plt.xlabel('x')
    plt.ylabel('y')
    plt.axis("equal")
    plt.grid(True)
    plt.draw()
    plt.pause(0.00000000001)


def plotter():
    plt.plot(cx, cy, ".r")
    plt.plot(cx_s,cy_s, "-b")
    plt.xlabel('x')
    plt.ylabel('y')
    plt.axis("equal")
    plt.grid(True)
    plt.show()







def main():

    rospy.init_node('plotter', anonymous=True)
    #pub = rospy.Publisher('delta',Float32,queue_size=10)

    #rospy.Subscriber("tartgetyaw", Float64, callback2)
    rospy.Subscriber("odom", Odometry, callback)
    plotter()

    # if rospy.is_shutdown():
    #     print('shutdown')
    #     plotter()

    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
    # Ctrl+C 입력시 예외 발생
        sys.exit()  # 종료


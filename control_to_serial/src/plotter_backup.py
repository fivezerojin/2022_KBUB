#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import math
import matplotlib.pyplot as plt
import numpy as np
from nav_msgs.msg import Odometry
cx = []
cy = []

with open("/home/usera/catkin_ws/src/control_to_serial/map/x_y_path2.txt", 'r') as f:
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


def posecallback(msg):
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    plt.plot(x, y, '.r')
    if (first == 1):
        plt.plot(cx, cy, '.b')
        first = 2
    #plt.plot(msg.position.y, msg.position.x, '*')
    plt.axis("equal")
    plt.draw()
    plt.pause(0.00000000001)

def yawcallback(msg):
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    plt.plot(x, y, '.r')
    if (first == 1):
        plt.plot(cx, cy, '.b')
        first = 2
    #plt.plot(msg.position.y, msg.position.x, '*')
    plt.axis("equal")
    plt.draw()
    plt.pause(0.00000000001)


def main():

    rospy.init_node('plotter', anonymous=True)
    rospy.Subscriber("gps_meas2", Odometry, posecallback)

    while not rospy.is_shutdown():

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

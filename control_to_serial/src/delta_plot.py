#!/usr/bin/env python

import matplotlib.pyplot as plt
import rospy
import sys
from std_msgs.msg import Float32

delta = []
def delta_callback(msg):
    delta.append(msg.data)

def main():
    rospy.init_node("delta_plot", anonymous=True)
    rospy.Subscriber("delta", Float32, delta_callback)
    rate = rospy.Rate(10)
    rospy.loginfo("Hi")
    while not rospy.is_shutdown():
        rate.sleep()
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
        plt.plot(delta,'r')
        plt.grid(True)
        plt.show()
    except KeyboardInterrupt:
        sys.exit()
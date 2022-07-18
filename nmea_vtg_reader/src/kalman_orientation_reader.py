#!/usr/bin/env python


import rospy
import string
import math


from nav_msgs.msg import Odometry


from tf.transformations import euler_from_quaternion


def orientation_callback (Odometry):
    
    rospy.loginfo('Successfully callback')
    # for i in range(0,3,1):
    #     quaternion = [i] 
    x = Odometry.pose.pose.orientation.x 
    y = Odometry.pose.pose.orientation.y 
    z = Odometry.pose.pose.orientation.z 
    w = Odometry.pose.pose.orientation.w 
    q = euler_from_quaternion([x,y,z,w])
    # print(x,y,z,w)
    # roll = q[0]*(180/3.141592)
    # pitch = q[1]*(180/3.141592)
    yaw = q[2]*(180/3.141592)
    print('yaw=',yaw )

def listener():
    rospy.init_node('kalman_orientation_reader', anonymous=True)
    rospy.Subscriber("/odometry/filtered", Odometry , orientation_callback)
    rospy.spin()
    

    
if __name__ == '__main__':
    listener()
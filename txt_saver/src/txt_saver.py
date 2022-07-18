#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import sys
import string
import os
from nav_msgs.msg import Odometry
import sys
cnt=1

# path_name = sys.argv[1]

# if len(sys.argv) != 2:
#     print("Insufficient arguments")
#     sys.exit()

# print("Start logging File path : " + file_path)
# file_path = '/home/usera/catkin_ws/src/control_to_serial/map/'+path_name+'.txt'

fw = open('/home/usera/catkin_ws/src/txt_saver/map/bus.txt','w')

def text_callback(msg):
	global utm_x
	global utm_y
	global fw	
	global cnt
	utm_x = msg.pose.pose.position.x
	utm_y = msg.pose.pose.position.y
	#print(os.getcwd())
	fw.write(str(utm_x))
	fw.write(',')
	fw.write(str(utm_y))
	fw.write('\n')


	cnt += 1
	print(cnt)
def main():
	
	while not rospy.is_shutdown():
		
		rospy.init_node('txt_saver', anonymous=True)
		rospy.Subscriber("odom", Odometry, text_callback)
		rospy.loginfo("writing x,y for UTM")
		
		rospy.spin()
	fw.close()
	rospy.loginfo("Successfully exits")

		
if __name__ == '__main__':
	main()
	



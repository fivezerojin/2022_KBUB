#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys,os
import math
import numpy as np
import matplotlib.pyplot as plt
import rospy
from std_msgs.msg import Float64MultiArray,Float64, Float32
from nav_msgs.msg import Odometry

#-------추가(해결)해야할 사항------
#https://velog.io/@legendre13/Stanley-Method
#1 map_yaws=[] --> 해결 완료
#2 BACKTOWHEEL, WHEEL_LEN, TREAD 뭔지 알아내기

# Initializing parameters
gps_heading = 0.0
steer = 0.0
speed_r =0.0
step = 0 #for counting Hz
xs = []
ys = []
yaws = []
steers = []
ts = []

# paramters 
dt = 0.125


k =  0.22# control gain
x_s, y_s, yaw_s = [0 for i in range(20)], [0 for i in range(20)], [0 for i in range(20)]

# # GV70 PARAMETERS
# LENGTH = 4.715
# WIDTH = 1.160
# L = 1.040 #wheel base
# BACKTOWHEEL = 1.0
# WHEEL_LEN = 0.3  # [m]
# WHEEL_WIDTH = 0.2  # [m]
# TREAD = 0.8  # [m]

# ERP-42 PARAMETERS
LENGTH = 4.715  # 길이
WIDTH = 1.910   # 너비
L = 2.875       # 휠베이스(축거)
BACKTOWHEEL = 1.0
WHEEL_LEN = 0.33  # [m] #휠 인치
WHEEL_WIDTH = 0.2  # [m] #휠 폭
TREAD = 0.8  # [m] #윤거

class VehicleModel(object):
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v

        self.max_steering = np.radians(18)

    def update(self, steer, a=0):
        steer = np.clip(steer, -self.max_steering, self.max_steering)
        self.x += self.v * np.cos(self.yaw) * dt
        self.y += self.v * np.sin(self.yaw) * dt
        self.yaw += self.v / L * np.tan(steer) * dt
        self.yaw = self.yaw % (2.0 * np.pi)
        self.v += a * dt


def normalize_angle(angle):
    while angle > np.pi:
        angle -= 2.0 * np.pi

    while angle < -np.pi:
        angle += 2.0 * np.pi

    return angle


def stanley_control(x, y, yaw, v, map_xs, map_ys, map_yaws):
    # find nearest point
    min_dist = 1e9
    min_index = 0
    n_points = len(map_xs)

    front_x = x + L * np.cos(yaw)
    front_y = y + L * np.sin(yaw)

    for i in range(n_points):
        dx = front_x - map_xs[i]
        dy = front_y - map_ys[i]
        
        dist = np.sqrt(dx * dx + dy * dy)
        # print(dist)
        if dist < min_dist:
            min_dist = dist
            min_index = i

    # compute cte at front axle
    map_x = map_xs[min_index]
    map_y = map_ys[min_index]
    map_yaw = map_yaws[min_index]
    dx = map_x - front_x
    dy = map_y - front_y

    perp_vec = [np.cos(yaw + np.pi/2), np.sin(yaw + np.pi/2)]
    cte = np.dot([dx, dy], perp_vec)
    print("Calculated CTE is : {:.3f} meters".format(cte))
    # control law
    yaw_term = normalize_angle(map_yaw - yaw)
    cte_term = np.arctan2(k*cte, v)

    # steering
    steer = yaw_term + cte_term
    # print(steer)
    return steer

def yaw_callback(msg):
    global gps_heading
    gps_heading = msg.data

def array_callback(msg):
    global x_s,y_s,yaw_s
    # get points from kbub topic
    _idx = 0
    _idxx = 0
    while True:
        # print(len(msg.data))
        x_s[_idx] = msg.data[_idxx]
        y_s[_idx] = msg.data[_idxx+1]
        _idx += 1
        _idxx += 2
        if _idx==20:
            # print(_idx)
            break
    # create map_yaw
    for i in range(len(x_s)-1):
        temp_x = x_s[i+1] - x_s[i]
        temp_y = y_s[i+1] - y_s[i]
        calculated_yaw = normalize_angle(math.atan2(temp_y,temp_x))
        yaw_s[i] = calculated_yaw
        if i == len(x_s)-2:
            yaw_s[i+1] = calculated_yaw


def speed_r_callback(msg):
    global speed_r
    speed_r = (msg.data /10)/3.6 #[m/s]

def odom_callback(msg):
    global step,x,y, x_s, y_s, xs, ys, yaws, steers, ts, speed_r, gps_heading
    
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    t = step * dt
    # vehicle
    model = VehicleModel(x=x, y=y, yaw=gps_heading, v=speed_r) #v에 speed_r
    steer = stanley_control(model.x, model.y, model.yaw, model.v, x_s, y_s, yaw_s)
    steer = np.clip(steer, -model.max_steering, model.max_steering)
    print("\n")
    print(type(steer))
    print("Calculated steer is : {:.3f} degrees".format(steer))
    print("ERP-42 speed        : {:.3f} m/s".format(speed_r))
    model.update(steer)    # steer is degree or radian??? 

    # only for plotting result
    xs.append(model.x)
    ys.append(model.y)
    yaws.append(model.yaw)
    ts.append(t)
    steers.append(steer)
    step += 1 

    #----Publish <delta>----#
    delta_pub.publish(float(steer*(180/math.pi)))
       

# plot car
def plotting_cars():
    #-----------Loading Reference Path-------------#
    path_x, path_y = [], []
    path = "/home/"+os.getlogin()+"/catkin_ws/src/kbub_pkg/src/map/"
    file_list = os.listdir(path)
    file_list = [file for file in file_list if file.endswith(".txt")]
    for txt_file in file_list:
        with open(path + txt_file) as t:
            while True:
                temp_line = t.readline()
                if not temp_line:
                    break
                if temp_line.find('\n'):
                    temp_line = temp_line[:temp_line.find('\n')]
                temp_line = map(float,temp_line.split(","))
                path_x.append(temp_line[0])
                path_y.append(temp_line[1])
            
    plt.figure("Stanley Result")
    plt.plot(path_x, path_y, 'r-', label="reference")
    plt.plot(xs, ys, 'b--', alpha=0.5, label="stanley")
    for i in range(len(xs)):
    # plt.clf()
        if i % 25 == 0:
            plt.plot(path_x, path_y, 'r-')
            plt.plot(xs, ys, 'b--', alpha=0.5)
            x = xs[i]
            y = ys[i]
            yaw = yaws[i]
            steer = steers[i]

            outline = np.array([[-BACKTOWHEEL, (LENGTH - BACKTOWHEEL), (LENGTH - BACKTOWHEEL), -BACKTOWHEEL, -BACKTOWHEEL],
                            [WIDTH / 2, WIDTH / 2, - WIDTH / 2, -WIDTH / 2, WIDTH / 2]])
            fr_wheel = np.array([[WHEEL_LEN, -WHEEL_LEN, -WHEEL_LEN, WHEEL_LEN, WHEEL_LEN],
                             [-WHEEL_WIDTH, -WHEEL_WIDTH, WHEEL_WIDTH, WHEEL_WIDTH, -WHEEL_WIDTH]])

            rr_wheel = np.copy(fr_wheel)
            fl_wheel = np.copy(fr_wheel)
            rl_wheel = np.copy(rr_wheel)

            Rot1 = np.array([[np.cos(yaw), np.sin(yaw)],
                         [-np.sin(yaw), np.cos(yaw)]])
            Rot2 = np.array([[np.cos(steer+yaw), np.sin(steer+yaw)],
                         [-np.sin(steer+yaw), np.cos(steer+yaw)]])

            fr_wheel = (fr_wheel.T.dot(Rot2)).T
            fl_wheel = (fl_wheel.T.dot(Rot2)).T
            fr_wheel[0, :] += L * np.cos(yaw) - TREAD * np.sin(yaw)
            fl_wheel[0, :] += L * np.cos(yaw) + TREAD * np.sin(yaw)
            fr_wheel[1, :] += L * np.sin(yaw) + TREAD * np.cos(yaw)
            fl_wheel[1, :] += L * np.sin(yaw) - TREAD * np.cos(yaw)
            rr_wheel[1, :] += TREAD
            rl_wheel[1, :] -= TREAD

            outline = (outline.T.dot(Rot1)).T
            rr_wheel = (rr_wheel.T.dot(Rot1)).T
            rl_wheel = (rl_wheel.T.dot(Rot1)).T

            outline[0, :] += x
            outline[1, :] += y
            fr_wheel[0, :] += x
            fr_wheel[1, :] += y
            rr_wheel[0, :] += x
            rr_wheel[1, :] += y
            fl_wheel[0, :] += x
            fl_wheel[1, :] += y
            rl_wheel[0, :] += x
            rl_wheel[1, :] += y

            plt.plot(np.array(outline[0, :]).flatten(),
                 np.array(outline[1, :]).flatten(), 'k-', alpha=0.5)
            plt.plot(np.array(fr_wheel[0, :]).flatten(),
                 np.array(fr_wheel[1, :]).flatten(), 'k-')
            plt.plot(np.array(rr_wheel[0, :]).flatten(),
                 np.array(rr_wheel[1, :]).flatten(), 'k-')
            plt.plot(np.array(fl_wheel[0, :]).flatten(),
                 np.array(fl_wheel[1, :]).flatten(), 'k-')
            plt.plot(np.array(rl_wheel[0, :]).flatten(),
                 np.array(rl_wheel[1, :]).flatten(), 'k-')
            plt.plot(x, y, "bo")
            plt.axis("equal")
            # plt.pause(0.1)
    plt.xlabel("X [m]")
    plt.ylabel("Y [m]")
    plt.legend(loc="best")
    plt.tight_layout()
    plt.grid(True)
    plt.savefig("stanley_method.png", dpi=300)
    plt.show()

def main():
    global delta_pub
    rospy.init_node("stanley_python")

    # --------------Subscribers--------------#
    rospy.Subscriber("/odom_array",Float64MultiArray,array_callback)
    rospy.Subscriber("/odom",Odometry,odom_callback)
    rospy.Subscriber("/speed",Float32,speed_r_callback)
    rospy.Subscriber("/current_yaw",Float64,yaw_callback)

    # --------------Publishers--------------#
    delta_pub=rospy.Publisher("delta",Float32,queue_size=1)
    rospy.spin()


if __name__=="__main__":
    main()
    rospy.loginfo("----------PLOTTING RESULT----------")
    plotting_cars()

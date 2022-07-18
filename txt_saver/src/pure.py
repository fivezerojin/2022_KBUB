#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import os
import numpy as np
import math
import matplotlib.pyplot as plt
from std_msgs.msg import Float32
from std_msgs.msg import Float64, Float64MultiArray
from nav_msgs.msg import Odometry

K = 0.1 #前视距离系数
Lfc = 0.8 #前视距离
Kp = 1.0 #速度P控制器系数
dt = 0.1 #时间间隔，单件 s
L = 2.9 #车辆轴距，单位：m

delta_max = 18
cx = [0 for i in range(20)]
cy = [0 for i in range(20)]
yaw_data = 0.0
pu_delta=0.0



class VehicleState:

    def __init__(self,x=0.0,y=0.0,yaw=0.0,v=0.0):
        self.x = x
        self.y = y # the position of car
        self.yaw = yaw # the yaw of car, 0-360 degree
        self.v = v  # the speed of car, constant;
    
def update(state,a,delta): # update the state of car

    if state.v < 0:
        delta = -delta
    state.x = state.x +state.v*math.cos(state.yaw)*dt
    state.y = state.y + state.v*math.sin(state.yaw)*dt
    state.yaw = state.yaw +  math.tan(delta)*state.v / L * dt # the rate of yaw: dot(yaw) = v_r*tan(delta_f)/L;  the delta is chaing, 
                                                              # so the position is chaging.
    state.v = state.v + a * dt # PID controller ，要用到越往后写。
    return state

def calc_target_index(state,cx,cy):
    dx = [state.x - icx for icx in cx] #对象state的位置(x,y)在不断地变化
    dy = [state.y - icy for icy in cy]
    d = [math.sqrt(idx**2+idy**2) for (idx,idy) in zip(dx,dy)]
    ind = d.index(min(d)) # the nearsest point, which is nearest to the position of car.
    L = 0.0

    Lf = K*abs(state.v) + Lfc

    while L < Lf and (ind+1) < len(cx):#from the nearest to the target point
        dx_t = cx[ind+1] - cx[ind]
        dy_t = cy[ind+1] - cx[ind]
        L += math.sqrt(dx_t**2 + dy_t ** 2)
        ind += 1

    return ind # the index of target point 

def PContorl(target,current):
    a = Kp * (target - current)
    return a

def pure_pursuit_control(state, cx, cy, pind):
    ind = calc_target_index(state, cx, cy)

    #if pind >= ind: # what is pind?
        #ind = pind
    
    if ind < len(cx):
        tx = cx[ind]
        ty = cy[ind]
    else:
        tx = cx[-1]
        ty = cy[-1] # endif to get the tx and ty

    alpha = math.atan2(ty-state.y,tx-state.x) - state.yaw

    #if state.v < 0:
        #alpha = math.pi - alpha
    
    Lf = K * abs(state.v) + Lfc

    delta = math.atan2(2.0*L*math.sin(alpha)/Lf, 1.0) # the output: delta_f, the angle of front wheels.
    if state.v < 0:
        delta = -delta

    return delta, ind

# def odoam_array_callback(msg):
#     global cx, cy
#     _idx = 0
#     while _idx == 19:
#         cx[_idx] = msg.data[_idx]
#         cy[_idx] = msg.data[_idx+1]
#         _idx +=1

def array_callback(msg):
    global cx,cy,yaw_s
    # get points from kbub topic
    _idx = 0
    _idxx = 0
    while True:
        # print(len(msg.data))
        cx[_idx] = msg.data[_idxx]
        cy[_idx] = msg.data[_idxx+1]
        _idx += 1
        _idxx += 2
        if _idx==20:
            # print(_idx)
            break
    # create map_yaw
    # for i in range(len(x_s)-1):
    #     temp_x = cx[i+1] - cx[i]
    #     temp_y = cy[i+1] - cy[i]
    #     calculated_yaw = normalize_angle(math.atan2(temp_y,temp_x))
    #     yaw_s[i] = calculated_yaw
    #     if i == len(x_s)-2:
    #         yaw_s[i+1] = calculated_yaw    

def callback1(msg):

    global yaw_data

    yaw_data = msg.data

def callback2(msg):

    global pu_delta

    state = VehicleState(x = msg.pose.pose.position.x  , y = msg.pose.pose.position.y ,yaw = yaw_data, v = 12.0/3.6)

    target_ind =  calc_target_index(state,cx,cy)
    ai = PContorl(12/3.6,state.v)
    di, target_ind = pure_pursuit_control(state,cx,cy,target_ind)
    
    state = update(state,ai,di)

    pu_delta = di * (180/math.pi)
    



def main():
    global pu_delta
    rospy.init_node('stanley', anonymous=True)
    #----------PUBLISHER-------------#
    pub = rospy.Publisher('delta',Float32,queue_size=10)
    #----------SUBSCRIBER-------------#
    rospy.Subscriber("/current_yaw", Float64, callback1)
    rospy.Subscriber("/odom", Odometry, callback2)
    rospy.Subscriber("/odom_array",Float64MultiArray,array_callback)



    rate = rospy.Rate(150)

    while not rospy.is_shutdown():


        delta_float = pu_delta 
        # delta_float = np.clip(delta_float,-delta_max, delta_max)# % rospy.get_time()      tttttttttttttttttttttttttttttttttttttttttttttttttttttttttttttttttttttttttttttttttttt
        rospy.loginfo(delta_float)
        
        pub.publish(delta_float)
        rate.sleep()
    # cx = np.arange(0,50,1)
    # cy =[math.sin(ix / 5.0) * ix / 2.0  for ix in cx]

    # target_speed = -20.0 / 3.6 

    # T = 100.0

    # state = VehicleState(x = -0.0, y = -3.0, yaw = 0.0, v = 0.0)

    # lastIndex = len(cx) - 1 # the index is from 0;
    # time = 0.0 
    # x = [state.x]
    # y = [state.y]
    # yaw = [state.yaw]
    # v = [state.v]
    # t = [0.0]
    # target_ind = 0
    # out = []

    # while T >= time and lastIndex > target_ind:
    #     target_ind =  calc_target_index(state,cx,cy)
    #     ai = PContorl(target_speed,state.v)
    #     di, target_ind = pure_pursuit_control(state,cx,cy,target_ind)
    #     print(di)
    #     out.append(di)
    #     state = update(state,ai,di)

    #     time = time + dt

    #     x.append(state.x)
    #     y.append(state.y)
    #     yaw.append(state.yaw)
    #     v.append(state.v)
    #     t.append(time)

    #     plt.cla()
    #     plt.plot(cx,cy,".r",label="course")
    #     plt.plot(x,y,"-b",label="trajectory")
    #     plt.plot(cx[target_ind],cy[target_ind],"go",label="target")
    #     plt.axis("equal")
    #     plt.grid(True)
    #     plt.title("speed[Km/h]:"+ str(state.v*3.6)[:4])
    #     plt.pause(0.001)

if __name__ == '__main__':
    main()


    




#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
import matplotlib.pyplot as plt
import numpy as np
import os
import string
import time
from control_to_serial.msg import control2serial
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from std_msgs.msg import Int16
from nav_msgs.msg import Odometry

# yaw_pub = rospy.Publisher('targetyaw',Float64 ,queue_size=10)

k = 1.0 #6 #k는 반드시 전륜차 속도의 변화에 따라 변화해야 한다. k= a*v+b, 계수는 스스로 속도를 조절해야 한다
kp = 0 #20
ki = 0 #0.0001
dt = 0.02
L = 1.8
Lambda = 0.8 #stanley는 각도 출력의 무게를 계산해 크면 클수록 코너링 효과는 좋지만 직선으로 걸었을 때 효과가 떨어진다
delta = 0
targetV = 13.0
pre_del = 0
pre_del1 = 0
pre_del2 = 0

first = 1
pre_brake = 0
Flag = 4
global cx
global cy
print(os.getcwd())
a = []
point_x = []
point_y = []
cx = []
cy = []
yaw_data = 0.0
px = 0
py = 0
e = 0
# ran = 50
# once = 0
targetIndex = 0


with open("/home/usera/catkin_ws/src/control_to_serial/map/산학앞.txt", 'r') as f:
    while True:
        line = f.readline()
        if not line:
            break
        #print(line)
        if line.find('\n'):
            
            line = line[:line.find('\n')]
            # print(line)
        cxy= (line.split(','))
        # print(line)
      #  a.append([float(cxy[0]),float(cxy[1])])

        cx.append(float(cxy[0]))
        cy.append(float(cxy[1]))
arrlen = len(cxy)
# print(len(cx))

# plt.plot(cx, cy, '.b')

# plt.axis("equal")
# plt.xlabel('x')
# plt.ylabel('y')
# plt.pause(0.00000000001)
# plt.show()
	


            
      




class VehicleState:
    def __init__(self,x,y,yaw,v):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.a = 0.0
        #self.e = []
	#print(type(x))


    def Update(self):
        self.yaw = self.NormalizeAngle(self.yaw)

    def NormalizeAngle(self,angle):
        if (angle > math.pi):
            angle = angle - 2*math.pi
        elif (angle < -math.pi):
            angle = 2*math.pi + angle
        else:
            angle = angle

        return angle

    def AxisTrans(self,x,y):
        tx = (x-self.x) * math.cos(self.yaw) + (y - self.y) * math.sin(self.yaw)
        ty = -(x -self.x) * math.sin(self.yaw) + (y - self.y) * math.cos(self.yaw)
        return tx,ty

    def PID(self,targetV):
        self.a = kp * (targetV - self.v) + ki*(targetV - self.v)*dt


    def Calculate(self):
        global cx
        global cy
        global pre_brake
        global ran
        # global once
        global e
        global targetIndex
        
        preViewDelta = 0

        # if ran > (len(cx)-160):
        #     ran = len(cx) - 151

        # if once < 10:
        #     dx = [self.x - ix for ix in cx]
        #     dy = [self.y - fy for fy in cy]
        #     d = [math.sqrt(tx**2 + ty**2) for (tx,ty) in zip(dx,dy)]
        #     e = min(d)
        #     targetIndex = d.index(e)
        #     ran = targetIndex
        #     once = once + 1
        # else:
        #     aa = ran - 50
        #     ad = ran + 150
            
        #     dx = [self.x - cx[ix] for ix in range(aa,ad)]
        #     dy = [self.y - cy[fy] for fy in range(aa,ad)]
        #     d = [math.sqrt(tx**2 + ty**2) for (tx,ty) in zip(dx,dy)]
        #     e = min(d)
        #     targetIndex = d.index(e) - 50 + ran  
        #     ran = targetIndex


        
        dx = [self.x - ix for ix in cx]
        dy = [self.y - fy for fy in cy]
        d = [math.sqrt(tx**2 + ty**2) for (tx,ty) in zip(dx,dy)]
        e = min(d)
        targetIndex = d.index(e)

        
        # print("ran ",ran)

        preViewInd = targetIndex + 7
    

        if (preViewInd >= len(cx)):
            preViewInd = len(cx) - 1

        preX,preY = self.AxisTrans(cx[preViewInd],cy[preViewInd])
        preViewDelta = self.NormalizeAngle(math.atan2(preY,preX))

        tmp_y = -(cx[targetIndex] -self.x)* math.sin(self.yaw) + (cy[targetIndex]-self.y)*math.cos(self.yaw)
        if (tmp_y < 0):
            e = -e
        elif (tmp_y == 0):
            e = 0
        else:
            e = e

        index = []
        ind = 0

        while ind < (len(cx)-5):
            ind_tmp = math.atan2(cy[ind+5]-cy[ind+4],cx[ind+5]-cx[ind+4])
            ind_tmp = self.NormalizeAngle(ind_tmp)
            index.append(ind_tmp)
            ind += 1

        if targetIndex < len(cx)-1:
            targetYaw = index[targetIndex]
        else:
            targetYaw = index[len(cx)-2]

        if targetIndex < len(cx)-10:
            cur_tan = math.atan2(cy[targetIndex+1]-cy[targetIndex],cx[targetIndex+1]-cx[targetIndex])
            pre_tan = math.atan2(cy[targetIndex+10]-cy[targetIndex+9],cx[targetIndex+10]-cx[targetIndex+9])
            brake_triger = pre_tan - cur_tan
            if abs(brake_triger) > 15:
                pre_brake = 1
            else:
                pre_brake = 0

        print("Tar",targetYaw*180/math.pi)
        # yaw_pub.publish(targetYaw)
        print("self",self.yaw*180/math.pi)
        print("distance", e)

       
        return e,targetYaw,targetIndex,preViewDelta

    def StanleyControl(self,e,targetYaw,preViewDelta):
        phi = 0.0
        deltaStanley = 0.0
        deltaPreview = 0.0

        deltaMax = 28 * math.pi /180



        if (targetYaw - self.yaw) < math.pi:
            phi = targetYaw- self.yaw
        else:
            if (targetYaw > 0):
                phi = -2*math.pi + targetYaw - self.yaw
            else:
                phi = 2*math.pi - targetYaw + self.yaw

# ---------------------------------------------------------------------

        phi = self.NormalizeAngle(phi)
        back = False

        if (abs(phi) > math.pi / 2 ):
            back = True
        if (back == True and phi > 0):                  # 후진일 경우
            phi = math.pi - phi
        else:

            if (back == True and phi < 0):
                phi = -math.pi - phi
            else:
                phi = phi

# ----------------------------------------------------------------------
        print("phi",phi*180/math.pi)
        print("atan",math.atan2(k * e , self.v )*180/math.pi)

        deltaStanley = phi + math.atan2(k * e , self.v )
        delta = Lambda * deltaStanley + (1-Lambda)* preViewDelta


        if (delta > deltaMax):
            delta = deltaMax
        elif (delta < -deltaMax):
            delta = -deltaMax
        else:
            delta = delta


        return delta



# def callback1(msg):

#     global yaw_data
#     global Flag
#     if Flag == 4:
#         yaw_data = msg.data-(0*3.141592/180)			
#         #yaw_data = msg.data
 
def callback2(msg):
    global Flag
    global pre_del
    global pre_del1
    global pre_del2

    global delta
    global yaw_data
    global pxist 
    global first
    global k
    global pre_brake
    global px, py

    if Flag == 4:
        #print("callback respond!")
        if(pre_del > 15):
            k = 1.2
        else:
            k = 0.8

        x = msg.pose.pose.position.x   
        y = msg.pose.pose.position.y
        # plt.subplot(2,1,1)
        plt.plot(x,y,'.r')
        if (first == 1) :
            plt.plot(cx, cy, '.b')
            first =2
        #plt.plot(msg.position.y, msg.position.x, '*')
        plt.axis("equal")
        plt.draw()
        plt.pause(0.00000000001)


        cal_yaw = math.atan2(y-py,x-px)

        # yaw_data = cal_yaw + pre_del * (math.pi / 180) * 0.3#################################################################################
        yaw_data = cal_yaw + pre_del * (math.pi / 180) *0.2
        state = VehicleState(x = msg.pose.pose.position.x  , y = msg.pose.pose.position.y ,yaw = yaw_data, v = 10.0/3.6)
        state.Update() # gps present pose1
        e, targetYaw, targetIndex,preViewDelta = state.Calculate()

        delta = state.StanleyControl(e,targetYaw,preViewDelta) *180 /math.pi
        delta = delta

    #     delta = delta  # * 0.5 + pre_del * 0.25 + pre_del1 * 0.15 + pre_del2 * 0.1

        if pre_brake == 1:
            delta = delta + 100


        px = x
        py = y
        pre_del = delta
        pre_del1 = pre_del
        pre_del2 = pre_del1

        # plt.subplot(2,1,2)
        # plt.plot(time.time(),targetYaw,'.r')
        # plt.plot(time.time(),cal_yaw,'.b')
        # plt.axis("equal")
        # plt.draw()
        # plt.pause(0.00000000001)



def SetFlag(msg):
    global Flag

    Flag = msg.data

def main():

    global delta
   # cx = [0.2222,2,3,4,5,6]
   # cy = [20,15,3,5,4,9]


    #state = VehicleState(x = 346170.457985 , y = 4069631.59678 ,yaw = math.pi/2, v = 3.0)
   # state.Update() # gps present pose
   # e, targetYaw, targetIndex,preViewDelta = state.Calculate()

   # delta = state.StanleyControl(e,targetYaw,preViewDelta) *180 /math.pi





    rospy.init_node('stanley', anonymous=True)
    pub = rospy.Publisher('delta',Float32, queue_size=10)
    
    # rospy.Subscriber("heading", Float64, callback1)
    rospy.Subscriber("gps_meas2", Odometry, callback2)
    rospy.Subscriber("Plan_Flag", Int16, SetFlag)

    rate = rospy.Rate(150)


    while not rospy.is_shutdown():
        if Flag == 4:
            
            delta_float = delta # % rospy.get_time()
            rospy.loginfo(delta_float)
            pub.publish(delta_float)

        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

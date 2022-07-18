#include <ros/ros.h>
#include <string.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>
#include <iostream>
#include <math.h>
#include <cmath>
#include <nav_msgs/Odometry.h>
#include <fstream>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float64MultiArray.h"

using namespace std;

#define NUM_OF_C 20 //7976 //txt파일 라인개수 + 1 //점 개수에 따라 수정

const double PI = 3.1415926;
double k = 1.0;
double kp = 0;
double ki = 0;  //미사용
double kd = 0;  //실험 예정
double dt = 0.2; //미정
double L = 1.8;
double Lambda = 0.8;
double delta = 0.f;
double targetV = 15.0 / 3.6;
double pre_del = 0.f;
double pre_del1 = 0.f;
double pre_del2 = 0.f;
double a = 0.f;
double brake_t =0;

double pre_brake_trigger = 0.f;

int first = 1;
bool pre_brake = false;
bool Flag = 1;


double yaw_data = 0.f;
double px = 0.f;
double py = 0.f;
double pyaw = 0.f;
double yaw1 = 0.f;
double yaw = 0.f;

double e = 0.f;
int targetIndex = 0;
double targetYaw = 0.f;
double preViewDelta = 0.f;
double speed;

int num_of_cx = NUM_OF_C;
double cx[NUM_OF_C];
double cy[NUM_OF_C]; 
double Arr[10000][2];

int num_of_coord = 0;
int gps_count =0;

bool GoStraight = true;
//좌표 배열로 받아오기
void arrayCallback(const std_msgs::Float64MultiArray::ConstPtr& array);
void arr_to_cxcy();
void arrayCallback(const std_msgs::Float64MultiArray::ConstPtr& array)
{
	num_of_coord = 0;
	int x = 0;
	int y = 0;
	while(true)
	{
		// Arr[x][y] = array->data[num_of_coord++];
		// if(y == 0)	y++;
		// else{
		// 	x++;
		// 	y--;
		// }
		cx[x] = array -> data[num_of_coord++];
		cy[x++] = array -> data[num_of_coord++];

		if(array->data[num_of_coord] < 1.2)	break;
	}
    // arr_to_cxcy();
	return;
}

// void arr_to_cxcy()
// {
//     for (int i=0; i<20; i++)
//     {
//         printf("cx[%d] : %lf\n", i, cx[i]);
//         printf("cy[%d] : %lf\n", i, cy[i]);
//     }
// }




class VehicleState
{
private:
    double x;
    double y;
    double yaw;
    double v;
    double error;
    //double error_sum;
    //double prev_error = 0;

public:
    VehicleState(double x_, double y_, double yaw_, double v_,double e_)
    {
        x = x_;
        y = y_;
        yaw = yaw_;
        v = v_;
        e = e_;
        
    }

    double NormalizeAngle(double angle)
    {
        if(angle > PI)
            angle = angle - 2 * PI;
        else if ( angle < -PI)
            angle = 2*PI + angle;
        else
            angle = angle;
        return angle;        
    }
    
    void Update()
    {
        yaw = NormalizeAngle(yaw);
    }
    
    double AxisTrans_x(double x_, double y_)
    {
        double tx = (x_ - x) * cos(yaw) + (y_ - y) * sin(yaw);
        return tx;
    }

    double AxisTrans_y(double x_, double y_)
    {
        double ty= -(x_ -x) * sin(yaw) + (y_ - y) * cos(yaw);
        return ty;
    }

    double distance(double x_, double y_)
    {
        return sqrt((x-x_)*(x-x_) + (y-y_)*(y-y_));
    }
    
    void Calculate()
    {
        preViewDelta = 0;
        e = 2100000000;
        for(int i = 0; i < num_of_cx; i++)
        {
            double temp = distance(cx[i], cy[i]);
            if(temp < e)
            {
                e = temp;
                targetIndex = i;
            }
        }

        int preViewInd = targetIndex + 3;

        if(preViewInd >= num_of_cx)
            preViewInd = num_of_cx - 1;

        double preX = AxisTrans_x(cx[preViewInd], cy[preViewInd]);
        double preY = AxisTrans_y(cx[preViewInd], cy[preViewInd]);
        preViewDelta = NormalizeAngle(atan2(preY,preX));

        double tmp_y = -(cx[targetIndex] - x) * sin(yaw) + (cy[targetIndex] - y) * cos(yaw);

        if(tmp_y < 0)
        {
            e = -e;
        }
        else if (tmp_y == 0)
        {
            e = 0;
        }
        
        double index[num_of_cx] = {0,};
        int ind = 0;


        while( ind < (num_of_cx - 4))
        {
            double ind_tmp1 = atan2(cy[ind + 4] - cy[ind+2], cx[ind+4] - cx[ind+2]);
            double ind_tmp = atan2(cy[ind + 3] - cy[ind+1], cx[ind+3] - cx[ind+1]);
            // printf("cy[ind] :%lf, cy[%d+3] : %lf\n", cy[ind], ind, cy[ind + 3]);
            ind_tmp = NormalizeAngle((ind_tmp1+ind_tmp)/2);
            index[ind] = ind_tmp;
            ind += 1;
        }

        if(targetIndex < num_of_cx)
            targetYaw = index[targetIndex];
        else
        {
            targetYaw = index[num_of_cx - 2];
        }

        if(targetIndex < num_of_cx)
        {
            double cur_tan = atan2(cy[targetIndex + 3] - cy[targetIndex], cx[targetIndex + 3] - cx[targetIndex]);
            double pre_tan = atan2(cy[targetIndex +15] - cy[targetIndex + 6], cx[targetIndex + 15]-cx[targetIndex + 6]);
            double brake_triger = pre_tan - cur_tan;
            pre_brake_trigger = brake_triger;
            if(abs(brake_triger) > 13*(PI/180))
                {
                    pre_brake = true;
                }
            else            
                {
                    pre_brake = false;
                }
        }

        printf("Tar %lf\n", targetYaw*180/PI);
        printf("self %lf\n", yaw*180/PI);
        printf("distance %lf\n", e);     
    }
    
    void PID() //PID제어 
	{
        
		//double tmp_y = -(cx[targetIndex] - x) * sin(yaw) + (cy[targetIndex] - y) * cos(yaw)
		if (atan2(cy[targetIndex + 3] - y, cx[targetIndex + 3] - x)- yaw_data > 0) //오른쪽
           { error = e;

		    a = kp * (error); //+ kd * (error-prev_error)/dt; // ki * (error_sum)* dt;
            

		    // // printf("a: %lf\n", a);
            // printf("error: %1f\n", error);
            // printf("error sum: %1f\n", error_sum);
           }
		
		else
        {
            error = -e;
		    a = kp * (error); //+ kd * (error-prev_error)/dt; // ki * (error_sum)* dt;

		    // printf("a: %lf\n", a);
            // printf("error: %1f\n", error);
            // printf("error sum: %1f\n", error_sum);
            
        }
        // prev_error= error;
        // printf("prev_error: %1f\n", prev_error);
    }

    void StanleyControl()
    {
        double phi = 0.f;
        double deltaStanley = 0.f;
        double deltaPreview = 0.f;
        double deltaMax = 18 * PI /180;

        if((targetYaw - yaw) < PI)
            // phi = targetYaw - yaw;

            if(targetYaw - yaw < -PI)
            {
                phi = 2 * PI + targetYaw - yaw;
            }   
            else
            {
                phi = targetYaw - yaw;
            }

        else
        {
            if(targetYaw> 0)
            {
                phi = -2 * PI + targetYaw - yaw;
            }   
            else
            {
                phi = 2 * PI - targetYaw + yaw;
            }
        }
        
       // phi = NormalizeAngle(phi);
        
        printf("phi %lf\n", phi*180/PI);
        printf("atan %lf\n", atan2(k*e, targetV)*180/PI);

        deltaStanley = phi + atan2(k*e, targetV);
        delta = Lambda * deltaStanley + (1 - Lambda)*preViewDelta + a;
        
        if(delta > deltaMax)
        {
            delta = deltaMax;
            // printf("delta: %f",delta);

        }
        else if(delta < -deltaMax)
        {
            delta = -deltaMax;
            // printf("delta: %f",delta);
        }
        printf("-------------------------\n");
        printf("delta: %f \n",delta * 180/PI);
    }
    
    

};
void callback_heading(const std_msgs::Float64::ConstPtr &msg)
{
    yaw = msg->data;
    // printf("yaw : %lf \n", yaw * (180 / PI));

    // if (GoStraight)
	// {   
        
        gps_count += 1;
        
        if(gps_count < 24)
	    {
	    	delta = 0;
            std::cout<<gps_count<<endl;
	    }	

}
void callback1(const nav_msgs::Odometry::ConstPtr& odom)
{
        
    if(Flag)
    {
        // if(abs(pre_del) > 15)
        // {
        //     k = 1.2;
        // }
       
        // else
        // {
        //     k = 0.8;
        // }
        if(abs(pre_brake_trigger) > 13*(PI/180) && abs(delta) > 15)
        {
            k = 2.0;
        }
        else if((abs(pre_brake_trigger) <= 13*(PI/180) && abs(pre_brake_trigger) > 10*(PI/180)) && (abs(delta) <= 15 && abs(delta) > 10))
        { 
            k = 1.6;
        }
        else
        {
            k = 0.3;
        }
        
            
        double x = odom->pose.pose.position.x;
        double y = odom->pose.pose.position.y;

        double cal_yaw = atan2(y-py,x-px);

        yaw_data = yaw + pre_del * (PI/180) * 0.4 + pre_del1 * (PI/180) * 0.1;
        printf("yaw_local : %lf \n", yaw*180/PI);
        printf("cal_yaw : %lf \n", cal_yaw*180/PI);
        printf("yaw_data :  %lf \n", yaw_data*180/PI);

        // if(pyaw==0.f){
        //     yaw1 = yaw_data;
        // }

        // else{
        //     yaw1 = (pyaw + yaw_data)/2;
        // }

        double e_data = e;
        VehicleState state(x,y,yaw_data,targetV,e_data);       
        state.Update();
        state.Calculate();
        state.StanleyControl();
        state.PID();

        delta = delta * 180 / PI;

        if(pre_brake == true)
            delta = delta + 100;
           
        px = x;
        py = y;
        pyaw = yaw_data;
        pre_del = delta;
        pre_del1 = pre_del;
        // pre_del2 = pre_del1;

    }
}

void SetFlag(const std_msgs::Bool::ConstPtr &msg)
{
    Flag = msg->data;
}

void callback2(const std_msgs::Float32::ConstPtr& msg)
{
    speed = msg->data;
}

int main(int argc, char **argv)
{
    
    ros::init(argc, argv, "stanley");
	ros::NodeHandle n;
    ros::Publisher delta_pub = n.advertise<std_msgs::Float32>("delta",10);
    ros::Subscriber gps_meas2_sub = n.subscribe("odom", 10, callback1);
    ros::Subscriber Plan_Flag_sub = n.subscribe("Flag", 10, SetFlag);
    ros::Subscriber speed_sub = n.subscribe("speed", 100, callback2);
    ros::Subscriber sub3 = n.subscribe("odom_array", 10, arrayCallback);
    ros::Subscriber sub_heading = n.subscribe("/current_yaw", 10, callback_heading);



     std_msgs::Float32 delta_;

    ros::Rate loop_rate(50);
    //read_to_txt();
    while (ros::ok())
    {   
        
        ros::spinOnce();
        if (Flag)
        {
            
            // printf("-------------------------\n");
            // printf("delta: %f\n",delta);
            
            delta_.data = delta;
            delta_pub.publish(delta_);
        }
        loop_rate.sleep();
    }
    ros::spin();
	return 0;
}

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <nmea_vtg_reader/gps_heading.h> // using custom message for publish gps_heading
#include <stdio.h>
#include <math.h>

#include <sstream>
#include <time.h>
#include <string.h>

#include <cmath>
#define M_PI 3.14159265358979323846
// #include <tf2/LinearMath/Quaternion.h>
using namespace std;

double vecx_saved[9] = {0};
double vecy_saved[9] = {0};
double result_save[2] = {0};
double velocity = 0;
double imu_yaw = 0;
double result_for = 0;
static int first = 1;
static int second = 1;

ros::Time t1, t2;

ros::Publisher heading_pub, odom_pub, park_pub;
//nmea_vtg_reader::gps_heading gps_head;
nav_msgs::Odometry odom2;
std_msgs::Float64 yaw, heading;
std_msgs::Int16 park_flag;

typedef struct Vector
{
    double x;
    double y;
} Vector;

Vector v1, v2, v3, p1, p2, p3, cent; //v1~3 == vector , p1~3 == points for triangle, cent == triangle centroid

Vector VectorSubtractVector(Vector dest, Vector start)
{
    Vector ret;
    ret.x = dest.x - start.x;
    ret.y = dest.y - start.y;
    return ret;
}

double GetHeadingAngle(Vector v1, Vector v2)
{
    double ret;
    ret = acos((v1.x * v2.x + v1.y * v2.y) / (sqrt(v1.x * v1.x + v1.y * v1.y) * sqrt(v2.x * v2.x + v2.y * v2.y))) * 180 / M_PI;
    return ret;
}
// void imu_callback(const sensor_msgs::Imu::ConstPtr &imu) 가속도 받기 
// {
//     //imu_yaw = imu->;
//       result_for = result * (M_PI / 180)
         //printf("diff between yaw = %lf\n",fabs(imu_yaw-result_for));
// }
void heading_callback(const nav_msgs::Odometry::ConstPtr &odom)
{
    t2 = ros::Time::now();
    if (first == 1)
    {
        ROS_INFO("heading_callback function is successfully responding");
    }

    //input points for UTM

    // Reference vector to EAST

    v1.x = 1;
    v1.y = 0;

    //Make queue
    for (int cnt = 0; cnt < 8; cnt++)
    {
        vecx_saved[cnt] = vecx_saved[cnt + 1];
        vecy_saved[cnt] = vecy_saved[cnt + 1];
    }
    //Input new one
    vecx_saved[8] = odom->pose.pose.position.x;
    vecy_saved[8] = odom->pose.pose.position.y;
    
    //Calculate distance for parking mission
    double dist; 
    dist = (sqrt(pow(346724.53333 - vecx_saved[8], 2) + pow(4070416.5015000 - vecy_saved[8], 2)));

    if (dist < 0.5)
    {
        park_flag.data = 1;
    } 
    else
    {
        park_flag.data = 0;
    }
    
    if (first > 9 && vecx_saved[0] != 0 && vecy_saved[0] != 0)
    {
        v3.x = vecx_saved[8];
        v3.y = vecy_saved[8];
        v2.x = vecx_saved[1];
        v2.y = vecy_saved[1];

        p1.x = vecx_saved[8];
        p1.y = vecy_saved[8];
        p2.x = vecx_saved[7];
        p2.y = vecy_saved[7];
        p3.x = vecx_saved[6];
        p3.y = vecy_saved[6];

        velocity = (sqrt(pow(p1.x - vecx_saved[0], 2) + pow(p1.y - vecy_saved[0], 2))) / (t2 - t1).toSec();

        printf("---------------------------------\n");
        printf("velocity=%lf\n ", velocity);

        //make triangle centroid
        cent.x = (p1.x + p2.x + p3.x) / 3;
        cent.y = (p1.y + p2.y + p3.y) / 3;

        if (GetHeadingAngle(v1, VectorSubtractVector(v3, v2)))
        {
            
            double result = GetHeadingAngle(v1, VectorSubtractVector(v3, v2)); // v1 벡터를 기준으로 v1과 (v3 - v2) 사이의 헤딩각을 구한다.

            
            double obse = vecy_saved[8]-vecy_saved[7];
            if (obse <= 0.0)
            {
                heading.data = -(result * (M_PI / 180));
                //printf("%lf\n", -result);
                result_save[1]= -result;
            }
            else
            {
                heading.data = result * (M_PI / 180);
                //printf("%lf\n", result);
                result_save[1]= result;
            }
            if (fabs(result_save[1]-result_save[0]) > 20)
            {
                heading.data = result_save[0] * (M_PI / 180);
            }
            result_save[0] = result_save[1];
            printf("%lf\n",heading.data*180/M_PI);


            odom2.pose.pose.position.x = cent.x;
            odom2.pose.pose.position.y = cent.y;
            heading_pub.publish(heading);
            odom_pub.publish(odom2);
            park_pub.publish(park_flag);

            //velocity 값 신뢰할 수 있는지 판명될 때 주석 해제
            // if(velocity < 0.2)
            // {
            //     gps_head.heading = imu_yaw;
            // }
            // else
            // {
            //     gps_head.heading = result * (M_PI/180);
            // }
        }
    }
    first++;
    t1 = ros::Time::now();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "heading_using_vector");
    ros::NodeHandle nh;

    ros::Subscriber coor_sub = nh.subscribe("/gps_meas2", 1, heading_callback);
    //ros::Subscriber imu_sub = nh.subscribe("/yaw", 1, imu_callback);
   // heading_pub = nh.advertise<nmea_vtg_reader::gps_heading>("/heading", 1);
    heading_pub = nh.advertise<std_msgs::Float64>("/heading", 1);
    odom_pub = nh.advertise<nav_msgs::Odometry>("/gps_meas1", 1);
    park_pub = nh.advertise<std_msgs::Int16>("/mission_flag",1);
    ros::Rate loop_rate(10);
    loop_rate.sleep();
    ros::spin();
}
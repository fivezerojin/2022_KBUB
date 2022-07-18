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
int str1 = 1,str2 = 1,str3 =1,str4=1, str5 = 1, str6 = 1, str7 = 1, str8 = 1, str9 = 1, str10 = 1;
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
    
    //Calculate distance for mission zone
    double park_d,statica_d,staticb_d,stra_d,strb_d,strc_d,strd_d,moving_d,lefta_d,leftb_d,leftc_d; 
    park_d = (sqrt(pow(302478.5944 - vecx_saved[8], 2) + pow(4123743.9455 - vecy_saved[8], 2)));
    statica_d = (sqrt(pow(302490.5498 - vecx_saved[8], 2) + pow(4123800.3899 - vecy_saved[8], 2)));
    staticb_d = (sqrt(pow(302563.1888 - vecx_saved[8], 2) + pow(4123895.5999 - vecy_saved[8], 2)));
    stra_d = (sqrt(pow(302495.23555 - vecx_saved[8], 2) + pow(4123851.1855 - vecy_saved[8], 2)));
    strb_d = (sqrt(pow(302590.03222 - vecx_saved[8], 2) + pow(4123954.59999 - vecy_saved[8], 2)));
    strc_d = (sqrt(pow(302567.3433 - vecx_saved[8], 2) + pow(4123920.00777 - vecy_saved[8], 2)));
    strd_d = (sqrt(pow(302542.56333 - vecx_saved[8], 2) + pow(4123869.88222 - vecy_saved[8], 2)));
    moving_d = (sqrt(pow(302577.8455 - vecx_saved[8], 2) + pow(4123821.3733 - vecy_saved[8], 2)));
    lefta_d = (sqrt(pow(302543.8755 - vecx_saved[8], 2) + pow(4123824.45555 - vecy_saved[8], 2)));
    leftb_d = (sqrt(pow(302597.51555 - vecx_saved[8], 2) + pow(4124066.44333 - vecy_saved[8], 2)));
    leftc_d = (sqrt(pow(302571.5655 - vecx_saved[8], 2) + pow(4124127.9833 - vecy_saved[8], 2)));
    //주차
    if (park_d < 0.6)
    {
        park_flag.data = 1;
    } 
    //좌회전
    else if (str8 == 1 && lefta_d < 1)
    {
        park_flag.data = 3;
        str8 = 2;
    } 
    else if (str8 == 2 && lefta_d < 15)
    {
        park_flag.data = 3;
    }
    else if (str9 == 1 && leftb_d < 5)
    {
        park_flag.data = 3;
        str9 = 2;
    } 
    else if (str9 == 2 && leftb_d < 30)
    {
        park_flag.data = 3;
    }
    else if (str10 == 1 && leftc_d < 5)
    {
        park_flag.data = 3;
        str10 = 2;
    } 
    else if (str10 == 2 && leftc_d < 30)
    {
        park_flag.data = 3;
    }
    //직진
    else if (str1 == 1 && stra_d < 1)
    {
        park_flag.data = 2;
        str1= 2;
    } 
    else if (str1 == 2 && stra_d < 15)
    {
        park_flag.data = 2;
    } 
    
    else if (str2 == 1 &&strb_d < 1)
    {
        park_flag.data = 2;
        str2 = 2;
    } 
    else if (str2 == 2 && strb_d < 35)
    {
        park_flag.data = 2;
    } 

    else if (str3 == 1 && strc_d < 1)
    {
        park_flag.data = 2;
        str3 = 2;
    } 
    else if (str3 == 2 && strc_d < 35)
    {
        park_flag.data = 2;
    } 

    else if (str4 ==1 && strd_d < 1)
    {
        park_flag.data = 2;
        str4 = 2;
    }
    else if (str4 == 2 && strd_d < 15)
    {
        park_flag.data = 2;
    }  
    //정적
    else if (statica_d < 1 && str5 == 1)
    {
        park_flag.data = 4;
        str5 = 2;
    }
    else if (str5 == 2 && statica_d < 15)
    {
        park_flag.data = 4;
    } 

    else if (staticb_d < 1 && str6 == 1)
    {
        park_flag.data = 4;
        str6 =2;
    } 
    else if (str6 == 2 && staticb_d < 15)
    {
        park_flag.data = 4;
    }
    //동적
    else if (str7 ==1 &&moving_d < 1)
    {
        park_flag.data = 5;
        str7 = 2;
    }
    else if (str7 == 2 && moving_d < 15)
    {
        park_flag.data = 5;
    } 
    else
            // cout << num_of_cx << endl;
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
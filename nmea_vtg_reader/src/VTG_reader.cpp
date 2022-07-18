#include <ros/ros.h>
#include <serial/serial.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <nmea_msgs/Sentence.h>
// #include "nmea_vtg_reader/vel_msg.h" // using custom message
#include <sstream>
#include <time.h>
#include <string.h>
#include "tf/transform_datatypes.h"

#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Dense>

#include <cmath>
// #include <tf2/LinearMath/Quaternion.h>
using namespace std;
using namespace Eigen;
std::string data_n;
string data_f;
serial::Serial ser;
int start_num = 0;
int end_num = 0;
float deg2rad = 3.141592 / 180.0;
float rad2deg = 180 / 3.141592;
double VTG_data[8] = {0};
double VTG[9] = {0};
double VTG_saved[2] = {0};
double ang = 0;
int saved_count = 1;
double first_ang = 0;
int first_run = 1;

typedef long long ll;

void nmea_callback(const nmea_msgs::Sentence::ConstPtr &sentence)
{
    data_n.clear();
    data_n = sentence->sentence;
    int f = data_n.find("$GNVTG");
    if (f == 0)
    {
        VTG_saved[0] = VTG_saved[1]; // make queue  ,will not execute until we move!

        if (data_n.length() != 0)
        {
            //std::cout << data_n << endl;
            //printf("callback 함수 실행 중!!\n");
            start_num = data_n.find("$GNVTG");
            //std::cout<<start_num<<endl;; //VTG messages read
            end_num = data_n.find("\n");

            data_f = data_n.substr(start_num + 7, end_num - start_num);
            //std::cout << data_f << endl;

            for (int i = 0; i < 9; i++)
            {

                VTG[i] = atof(data_f.substr(0, data_f.find(",")).c_str());
                data_f = data_f.erase(0, data_f.find(",") + 1);
                // VTG[8] = atof(data_f.c_str());
                //std::cout << VTG[i] << endl;
                //printf("%.5f ", VTG[i]);
                // printf("test\n");
                //printf("\n");
            }
            for (int cnt = 0; cnt < 7; cnt++)
            {
                VTG_data[cnt] = VTG_data[cnt + 1];
            }
            VTG_data[7] = VTG[0];
            double mean = 0;

            for (int cnt = 0; cnt < 7; cnt++)
            {
                mean += VTG_data[cnt];
            }
            mean = -mean / 8; // reverse clockwise
            
            if (first_run == 1 && VTG[0] != 0 && VTG_data[0] != 0)
            {
                first_ang = mean;
                first_run = 0;
            }
            if (VTG[0] == 0)
            {
                printf("Please move !\n");
                VTG_saved[1] = VTG_saved[0];
            }
            else
            {
                VTG_saved[1] = mean;
                ang = (VTG_saved[1] - first_ang) * deg2rad; // degree to radian , (first angle - second angle)
                //printf("Your Heading = %.5f", ang * rad2deg);
            }
            // if(ros::Publisher heading_pub;){

            // }
        }
    }
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "nmea_vtg_reader");
    ros::NodeHandle nh;
    ros::Publisher heading_pub = nh.advertise<geometry_msgs::QuaternionStamped>("/gps_heading", 1);
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::TwistStamped>("/gps_vel", 1);
    ros::Subscriber nmea_sub = nh.subscribe("nmea_sentence", 1, nmea_callback);
    std::cout << "start main func" << endl;
    geometry_msgs::QuaternionStamped heading_msg;
    // nmea_vtg_reader::vel_msg vel_msg;
    geometry_msgs::TwistStamped vel_msg;
    ros::Rate r(24);
    while (ros::ok())
    {
        system("clear");

        std::cout << "\n-----------------------------------------------------------\n";
        // static unsigned long long qwer = 1;
        // printf("count = %lli\n", qwer++);
        //RPY to Quarternion
        float roll = 0, pitch = 0, yaw = ang;
        // Quaternionf q;
        // q = AngleAxisf(roll, Vector3f::UnitX()) * AngleAxisf(pitch, Vector3f::UnitY()) * AngleAxisf(yaw, Vector3f::UnitZ());

        // std::cout<<"mids-----------------------------------------------------------\n";
        // heading_msg.quaternion.z = q.coeffs().z();
        // heading_msg.quaternion.y = q.coeffs().y();
        // heading_msg.quaternion.x = q.coeffs().x();
        // heading_msg.quaternion.w = q.coeffs().w();
        tf::Matrix3x3 obs_mat;
        obs_mat.setEulerYPR(ang, 0, 0);
        if (first_run == 1)
        {
            printf("아직 출발안함!!\n");
        }
        else
        {
            printf("first_ang : %lf\n", first_ang);
        }

        printf("Your Heading = %.5f\n", ang * 180 / 3.141592);

        tf::Quaternion q_tf;
        obs_mat.getRotation(q_tf);
        heading_msg.quaternion.x = q_tf.getX();
        heading_msg.quaternion.y = q_tf.getY();
        heading_msg.quaternion.z = q_tf.getZ();
        heading_msg.quaternion.w = q_tf.getW();
        double gps_velocity = VTG[6] / 3.6;
        if (gps_velocity < 0.06) //Ignore the velocity when it is under 0.06m/s
        {
            printf("정지!!\n");
            vel_msg.twist.linear.x = 0;
            vel_msg.twist.linear.y = 0;
        }
        else
        {
            vel_msg.twist.linear.x = gps_velocity * cos(ang * 180 / 3.141592); // km/h  to  m/s
            vel_msg.twist.linear.y = gps_velocity * sin(ang * 180 / 3.141592); // km/h  to  m/s
        }
        std::cout << "속도: " << gps_velocity << "m/s" << endl;
        std::cout << "속도 x: " << vel_msg.twist.linear.x << "m/s" << endl;
        std::cout << "속도 y: " << vel_msg.twist.linear.y << "m/s" << endl;

        //메세지 stamp 추가
        heading_msg.header.frame_id = "gps_heading";
        heading_msg.header.stamp = ros::Time::now();
        vel_msg.header.frame_id = "gps_velocity";
        vel_msg.header.stamp = ros::Time::now();
        heading_pub.publish(heading_msg); //heading publish
        vel_pub.publish(vel_msg);         //velocity publish

        ros::spinOnce();
        // r.sleep();

        // std::cout << "end-----------------------------------------------------------\n";
        // ros::spin();
    }
}

#include <ros/ros.h>
#include <string.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Point.h>
#include <iostream>
#include <math.h>
#include <cmath>
#include <nav_msgs/Odometry.h> //gps msgs -> lidar msgs로 수정
#include <fstream>
#include <vector>
using namespace std;

const double PI = 3.1415926;
double yaw = 90 * (PI / 180);
double delta = 0.f;


class VehicleState
{
private:
	double cx;
	double cy;

public:
	VehicleState(double cx_, double cy_)
	{
		cx = cx_;
		cy = cy_;
	}

	void Calculate()
	{
		double phi = 0.f;
		double deltaMax = 28 * (PI / 180);

		if (cy < 0)  //lidar 좌표가 erp 기준 오른쪽
		{
			phi = atan2(cx, -cy);
			printf("phi: %f", phi*(180 / PI));
		}

		else if (cy > 0) //lidar 좌표가 erp 기준 왼쪽
		{
			phi = PI - atan2(cx, cy);
			printf("phi: %f", phi*(180 / PI));
		}

		delta = phi - yaw;
        delta = 0.3 * delta;

		if (delta > deltaMax) //deltaMax : 28도(erp최대 조향각), delta값의 상하한선 지정
		{
			delta = deltaMax;

		}
		else if (delta < -deltaMax)
		{
			delta = -deltaMax;

		}

	}

};


void callback1(const geometry_msgs::Point::ConstPtr& odom) //lidar 메시지 형태 부분 -> 수정
{

	double cxx = odom->x; //lidar x좌표값 sub
	double cyy = odom->y; //lidar y좌표값 sub
    printf("cxx: %f\n", cxx);
	printf("cyy: %f\n", cyy);
	VehicleState state(cxx, cyy); 
	state.Calculate();

	delta = delta * (180 / PI); //delta값 degree화 -> erp controller, Serial에는 degree로 넣어야함!

	// if (abs(delta) < 2) //직선 구간, 최대속도
	// {
	// 	delta = delta + 500;
	// }
	// else if (abs(delta) < 7) {
	// 	delta = delta + 400;
	// }
	// else if (abs(delta) < 15) {
	// 	delta = delta + 300;
	// }
	// else if (abs(delta) < 22) {
	// 	delta = delta + 200;
	// }
	// else {
	// 	delta = delta + 100;
	// }
}

/*
delta를 구간에 나눠서 speed를 제어하기 위함, Serial에는 더해진 값이 들어오므로,
구간을 나눠서 speed 제어. 빠른 주행과 관성으로 smooth한 주행을 위해
brake를 사용하지 않음. 후에 구간별 각도값과 Serial에서 속도값을 조정하며 제어.

void steer_callback(const std_msgs::Float32::ConstPtr& float_msgs)

{
		del_speed = (float_msgs->data);
		if (del_speed > 50)
		{
			delta = del_speed - 100;
			speed = 7;
		}
		else if (del_speed > 150)
		{
			delta = del_speed - 200;
			speed = 10;
		}
		else if (del_speed > 250)
		{
			delta = del_speed - 300;
			speed = 13;
		}
		else if (del_speed > 350)
		{
			delta = del_speed - 400;
			speed = 15;
		}
		else if (del_speed > 450)
		{
			delta = del_speed - 500;
			speed = 20;
		}

		->serial에서 이부분을 추가.

*/



int main(int argc, char **argv)
{

	ros::init(argc, argv, "stanley"); //따로 수정하지 않음, 수정할 필요가 있으면 수정.
	ros::NodeHandle n;
	ros::Publisher delta_pub = n.advertise<std_msgs::Float32>("delta", 10); //delta라는 topic으로 ppublish (Serial에)
	ros::Subscriber lidar_sub = n.subscribe("/narrow_coord", 100, callback1); //라이다에서 좌표 publish하는 것 subscribe 부분
	std_msgs::Float32 delta_;
	ros::Rate loop_rate(50);
	while (ros::ok())
	{
		ros::spinOnce();
		printf("-------------------------\n");
		printf("delta: %f\n", delta);

		delta_.data = delta;
		delta_pub.publish(delta_);

		loop_rate.sleep();
	}
}
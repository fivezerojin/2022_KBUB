#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int16.h>
#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <nav_msgs/Odometry.h>

using namespace std;
bool f = false;
double speed = 0.0;
double brake = 0.0;
double stopLine = 0.0;
int Mission = 0;
double x=0.0, y=0.0;
void Flag_cb(const std_msgs::Bool::ConstPtr& msg)
{
	f = msg->data;
}
void speed_cb(const std_msgs::Float64::ConstPtr& msg)
{
	printf("speed cb : %lf\n", msg->data);
	speed = msg->data;
}
void brake_cb(const std_msgs::Float64::ConstPtr& msg)
{
	printf("brake cb : %lf\n", msg->data);
	brake = msg->data;
}
void steer_cb(const std_msgs::Float64::ConstPtr& msg)
{
	printf("steer cb : %lf\n", msg->data);
}
void stopline_cb(const std_msgs::Float64::ConstPtr& msg)
{
	stopLine = msg->data;
	
}
void mission_cb(const std_msgs::Int16::ConstPtr & msg)
{
	Mission = msg->data;
}
void Odometry_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
    
    x = msg->pose.pose.position.x;
    y = msg->pose.pose.position.y;
}
int main(int argc, char** argv)
{
	ros::init(argc, argv, "plan");
	ros::NodeHandle n;
	ros::Subscriber FlagSub = n.subscribe("/Flag", 10, Flag_cb);

	ros::Subscriber speedSub = n.subscribe("/speed", 10, speed_cb);
	ros::Subscriber brakeSub = n.subscribe("/brake", 10, brake_cb);
	ros::Subscriber steerSub = n.subscribe("/steer", 10, steer_cb);
	ros::Subscriber stoplineSub = n.subscribe("/stopline", 10, stopline_cb);
	ros::Subscriber missionSub = n.subscribe("/mission", 10, mission_cb);
	ros::Subscriber odometry = n.subscribe("/odom", 10, Odometry_cb); //현재 gps
	ros::Rate loop_rate(10);
	
	while(ros::ok())
	{
		printf("flag : ");
		if(f)	printf("True\n");
		else	printf("False\n");
		printf("speed : %lf, steer : %lf\n", speed, brake);
		printf("stopline : %lf\n", stopLine);
		printf("Mission : %d\n", Mission);
		printf("GPS : [%lf, %lf]\n", x, y);
		ros::spinOnce();
		loop_rate.sleep();
		system("clear");
	}
	ros::spin();
	return 0;
}
// #include <iostream>
// #include <fstream>
// #include "CarController.cpp"
// #include "GlobalPathManager.cpp"
// #include "InputManager.cpp"
// #include "CarController.cpp"

// #include <ros/ros.h>
// #include <geometry_msgs/Point.h>
// #include <sensor_msgs/PointCloud2.h>
// #include <std_msgs/Bool.h>
// #include <std_msgs/Float64.h>
// #include <std_msgs/Int16.h>
// #include "kbub_pkg/PointArray.h"
// #include "kbub_pkg/ParkPoint.h"


// #include <sensor_msgs/PointCloud.h>
// #include <sensor_msgs/PointCloud2.h>
// #include <sensor_msgs/point_cloud_conversion.h>

// #include <stdlib.h>



// using namespace std;
// int cnt = 0;
// bool isConer = true;
// bool encoder_stop = false;
// bool isFirst_park = true;
// bool isFisrt_canpark = true;
// bool isStart_ = false;
// int Flag = 4; //-1 : test, 0 : x, 1 : Lane, 2 : Parking, 3 : encoder, 4 : Local
// int MissionFlag ; // 0 : x, 1 : Parking, 2 : TrafficSign, 4 : StaticObstacle, 5 : UnexpectedObstacle
// int VisionClassCount[12] = {0,0,0,0,0,0,0,0,0,0,0,0};
// enum Sign
// {
// 	RedLight = 0,
// 	GreenLight = 1,
// 	GreenLeft,
// 	LeftSign,
// 	RightSign,
// 	StaticObstacle,
// 	UnexpectedObstacle,
// 	CrossWalkSign,
// 	ParkingSign,
// 	BusSign,
// 	SchoolSign,
// 	SpeedBump
// };

// InputManager* input_data = InputManager::GetInstance();
// CarController* control = CarController::Get();


// bool can_park , existob;
// bool Stop=false;
// bool Stop_TrafficLight(bool StopLane)
// {

// 	int TrafficThresh = 7;
// 	if(StopLane)
// 	{
// 		if(MissionFlag == 2)
// 		{
// 			if(VisionClassCount[GreenLight] > TrafficThresh || VisionClassCount[GreenLeft] > TrafficThresh)
// 				return false;
// 			if(VisionClassCount[RedLight] > TrafficThresh)
// 			{
// 				printf("Stop\n");
// 				return true;
// 			}
			
			
			
// 		}
	
// 		if(MissionFlag == 3)
// 		{
// 			if(VisionClassCount[GreenLeft] > TrafficThresh)
// 			{
// 				printf("go left");
// 				return false;
// 			}
// 			if(VisionClassCount[RedLight] > TrafficThresh || VisionClassCount[GreenLight] > TrafficThresh)
// 				{
// 					printf("stop_left");
// 					return true;
// 				}
			
// 		}
// 	}
// 	return false;

// }
// // ===============  Vision Callback  ===================
// void setVisionAngle(const std_msgs::Float64::ConstPtr &msg)
// {
// 	if(Flag == 1)
//     	input_data->setVisionAngle(msg->data);
// }
// void setParkingAngle(const std_msgs::Float64::ConstPtr &msg)
// {
// 	if(Flag == 2)
//    	 input_data->setVisionAngle(msg->data);
// }
// void setParkingStop(const std_msgs::Bool::ConstPtr &msg)
// {
// 	if(Flag == 2)
// 	{
// 		Stop = msg->data;
// 		control->SetBrake(Stop);
// 	}
// }
// void setVisionClass(const std_msgs::Int16::ConstPtr &msg)
// {
//     cout << msg->data << endl;
//     input_data->setVisionClass(msg->data);
// }

// // =============== 3d lidar 주차 미션 ===================
// void park_cb(const std_msgs::Bool::ConstPtr& input){
// 	//input_data->setPark(*input);
// }

// void velodyne_cb(const sensor_msgs::PointCloud2ConstPtr& input){
// 	if( isFisrt_canpark){
//     sensor_msgs::PointCloud total;
//     sensor_msgs::convertPointCloud2ToPointCloud(*input, total);
	
// 	//input_data->setCluster(total);

// 	}
	
// }

// // ======================================================

// void sick_cb(const kbub_pkg::PointArray::ConstPtr& input){
// 	for(int i = 0 ; i < input->cluster_center.size(); i++){
// 		// input_data->setClusterCenter(input->cluster_center[i]);
// 		// input_data->setClusterMax(input->cluster_max[i]);
// 		// input_data->setClusterMin(input->cluster_min[i]);
// 	}
// }

// void isStop_cb(const std_msgs::Bool::ConstPtr& input){
// 	input_data->setDynamicObject(input->data);

// 	if(input_data->getDynamicObject())
// 		existob = true;
// 	else
// 		existob = false;
	
// }

// void speed_cb(const std_msgs::Float64::ConstPtr& msg)
// {
// 	control->SetWheelSpeed(msg->data);
// }
// void brake_cb(const std_msgs::Float64::ConstPtr& msg)
// {
// 	control->SetBrake(msg->data);
// }
// void Fin_Encoder(const std_msgs::Bool::ConstPtr & msg)
// {
// 	if(msg->data && Flag == 3)
// 	{
// 		encoder_stop = true;
// 	}
// }
// void SetMissionFlag(const std_msgs::Int16::ConstPtr& msg)
// {
// 	// if(msg->data > 0)
// 	MissionFlag = msg->data;
// }
// int main(int argc, char** argv)
// {
// 	// cout << "--- Autonomous ---\n";
// 	// cout << "wheelSpeed: " << CarController::Get()->GetWheelSpeed() << "\n";

// 	// GlobalPathManager::Get()->ReadJson();
// 	ros::init(argc, argv, "main");
//     ros::NodeHandle nh;

// 	//////////////////////vision///////////////////////
	
// 	//main.cpp
// 	ros::Subscriber steering_sub = nh.subscribe("lane_msg2", 100, setVisionAngle);
//   //ros::Subscriber isStop_sub = nh.subscribe("isStop_lane", 100, setStopLane);
//     ros::Subscriber Vision_class = nh.subscribe("signs", 100, setVisionClass);

// 	//camera_test.cpp
// 	ros::Subscriber steering_sub_park = nh.subscribe("parking_test_msg", 100, setParkingAngle);
//     ros::Subscriber isStop_sub_park = nh.subscribe("parking_lane_stop", 100, setParkingStop);
// 	ros::Subscriber speed_sub = nh.subscribe("park_speed", 100, speed_cb);


//    //lidar(velodyne)
// 	ros::Subscriber sub_3d_park = nh.subscribe ("/can_park", 10, park_cb);
// 	ros::Subscriber sub_3d_lidar = nh.subscribe ("/velodyne_cluster", 10, velodyne_cb);
// //lidar(sick)
// 	ros::Subscriber sub_2d_lidar = nh.subscribe ("/sick_cluster_info", 10, sick_cb);
// 	ros::Subscriber sub_2d_isStop = nh.subscribe ("/isStop", 10, isStop_cb);

	
// 	//encoder(parking_test111.py)
// 	ros::Publisher isRecord = nh.advertise<std_msgs::Int16>("isRecord", 100);
// 	ros::Subscriber Finish_Record = nh.subscribe("Finish_Encoder", 100, Fin_Encoder);

// 	//Local(heading_using_vector.cpp)
// 	ros::Subscriber Mission_FLAG = nh.subscribe("mission_flag", 100, SetMissionFlag);

// 	//Serial
// 	ros::Publisher steering_pub = nh.advertise<std_msgs::Float64>("steering", 100);
//     // ros::Publisher isStop_pub = nh.advertise<std_msgs::Bool>("isStop_Lane_Pub", 10);
// 	ros::Publisher speed_pub = nh.advertise<std_msgs::Int16>("Speed_Pub", 10);

// 	ros::Publisher Stop_pub = nh.advertise<std_msgs::Bool>("Plan_Stop", 10);
// 	ros::Publisher Flag_pub = nh.advertise<std_msgs::Int16>("Plan_Flag", 10);
// 	std_msgs::Bool Plan_Stop_;
//     std_msgs::Float64 _steering;
//     std_msgs::Bool _isStop;
// 	std_msgs::Bool isStart;
// 	std_msgs::Int16 Speed_;
// 	std_msgs::Int16 Flag_;
// 	std_msgs::Int16 isRecord_;

// 	ros::Rate loop_rate(10);
// 	control->SetWheelSpeed(0.0);
// 	while(ros::ok()){
// 		if(can_park)
// 			cout << "can park" << '\n';
// 		else
// 			cout << "can not park" << '\n';
		
// 		if(existob)
// 			cout << "exist obstacle" << '\n';
// 		else
// 			cout << "none exist obstacle" << '\n';
// 		cout << "-------------------------------" << endl;
// 		cout<<"cnt : "<<cnt<<endl;

// 		///////////////////////////////////////////////test start

// 		// if( cnt < 10)
// 		// {

// 		// }
// 		// else if(cnt < 150)
// 		// {
// 		// 	Flag = 1;
// 		// 	MissionFlag = 1;
			
// 		// }
// 		// else if(cnt < 200)
// 		// {
// 		// 	can_park = true;
// 		// }
// 		if(MissionFlag == 1)
// 		{
// 			isFisrt_canpark = true;
// 			MissionFlag = 0;
// 		}
// 		///////////////////////////////////////////////test end
// 		if(can_park && isFisrt_canpark)
// 		{
// 			Flag = 2;
// 			isFisrt_canpark = false;
		
// 		}
// 		for(int i = 0; i < 12; i++)
// 		{
// 			if(input_data->getVisionClass()[i])
// 			{	
// 				if(VisionClassCount[i] < 10)
// 				{
// 					VisionClassCount[i]++;
// 				}
// 			}
// 			else
// 			{
// 				if(VisionClassCount[i] > 0)
// 				{
// 					VisionClassCount[i]--;
// 				}
// 			}
// 		}
// 		for(int i = 0; i< 12; i++)
// 		{
// 			if(i == 0)
// 			printf("[ %d, ", VisionClassCount[i]);
// 			else if( i < 11)
// 			{
// 				printf("%d, ", VisionClassCount[i]);
// 			}
// 			else
// 			{
				
// 				printf("%d ]\n", VisionClassCount[i]);
// 			}
			
// 		}
		
//         cout << "Vision Angle : " << input_data->getVisionAngle() << endl;
//         cout << "Distance : " << input_data->getVisionDistance() << endl;
// 		//cout<<"test : "<<input_data->getVisionClass()[2]<<" "<<GreenLight<<endl;
//         cout << "VisionClass : ";
// 		input_data->printVisionClass();
// 		cout<<"MissionFlag : "<<MissionFlag<<endl;
// 		if(Flag == 1)
// 		{
// 			if(isConer)
// 			{
// 				control->SetWheelSpeed(80);
// 				control->SetWheelAngle(input_data->getVisionAngle()); //80 -> 0.6, 
// 			}
// 			else
// 			{
// 				control->SetWheelSpeed(200);
// 				control->SetWheelAngle((input_data->getVisionAngle()*0.2));
// 			}
// 			Plan_Stop_.data = control->isBrake();
// 			Stop_pub.publish(Plan_Stop_);
			
			
// 		}
// 		else if(Flag == 2)
// 		{
// 			if(isFirst_park)
// 			{
// 				isFirst_park = false;
// 				control->SetWheelSpeed(0);
// 				control->SetBrake(true);
// 				control->SetWheelAngle(0.0);
// 				_steering.data = control->GetWheelAngle();
// 				Plan_Stop_.data = control->isBrake();
// 				Speed_.data = (int)(control->GetWheelSpeed());
				
// 				speed_pub.publish(Speed_);
// 				Stop_pub.publish(Plan_Stop_);
// 				steering_pub.publish(_steering);
// 				cout<<"Flag : "<<Flag<<endl;
// 				cout<<"\n-------Serial--------"<<endl;
// 				cout<<"Angle : "<<control->GetWheelAngle()<<endl;
// 				cout<<"Brake : "<<control->GetBrake()<<endl;
// 				cout<<"Speed : "<<control->GetWheelSpeed()<<endl;
// 				sleep(3);
// 				isRecord_.data = 1;
// 				isRecord.publish(isRecord_);
// 				continue;
// 			}
// 			else
// 			{
// 				control->SetWheelSpeed(50);
				
// 			}
			
// 			control->SetBrake(Stop);
// 			control->SetWheelAngle(input_data->getVisionAngle());
// 		}
// 		else if(Flag == 3)
// 		{
// 			isRecord_.data = 2;
// 			isRecord.publish(isRecord_);
// 			//Encoder
// 		}

// 		cout<<"Flag : "<<Flag<<endl;
// 		cout<<"\n-------Serial--------"<<endl;
// 		cout<<"Angle : "<<control->GetWheelAngle()<<endl;
// 		cout<<"Brake : "<<control->GetBrake()<<endl;
// 		cout<<"Speed : "<<control->GetWheelSpeed()<<endl;
// 		Flag_.data = Flag;
		
//         _steering.data = control->GetWheelAngle();
//         Plan_Stop_.data = control->isBrake();
// 		Speed_.data = control->GetWheelSpeed();
// 		Flag_pub.publish(Flag_);
// 		//publish data
// 		if((Flag != 3 && Flag != 4) || encoder_stop)
// 		{
// 			if(Plan_Stop_.data == true)
// 			{
// 				Speed_.data = 0;
// 			}
			
// 			//speed_pub.publish(Speed_);
// 			if(Flag == 3)
// 			{
// 				cout<<"encoder"<<endl;
// 				Flag = 4;
// 				encoder_stop = false;
// 				Speed_.data = 0;
// 				Plan_Stop_.data = true;
// 				speed_pub.publish(Speed_);
// 				steering_pub.publish(_steering);
// 				Stop_pub.publish(Plan_Stop_);
// 				sleep(2);
// 			}
// 			else{
// 				speed_pub.publish(Speed_);
// 			steering_pub.publish(_steering);
// 			Stop_pub.publish(Plan_Stop_);
// 			}
			
// 		}
// 		if(MissionFlag == 5) // 동적장애물
// 		{
// 			if(existob)
// 			{
// 				Plan_Stop_.data = true;
// 				Stop_pub.publish(Plan_Stop_);
// 				sleep(3);
// 			}
		
// 		}
// 		Stop_pub.publish(Plan_Stop_);
// 		if(Stop)
// 		{
// 			cout<<"Plan_Stop"<<endl;
// 			sleep(3); // 정지시 3초간 정지
// 			cout<<"123"<<endl;
// 			Stop = false;
// 			if(Flag == 2)
// 				Flag = 3;
		
// 		}
		
// 		if(input_data->getVisionClass()[RedLight])
// 		{
// 		}
// 		if(input_data->getVisionClass()[GreenLeft] || input_data->getVisionClass()[GreenLight])
// 		{
// 			cout<<"Start"<<endl;
// 		}
// 		if(input_data->getVisionClass()[CrossWalkSign])
// 		{
// 		}
// 		cnt++;
// 		ros::spinOnce();
// 		loop_rate.sleep();
// 		system("clear");
// 	}

// 	ros::spin();
// 	return 0;
// }

// int main()
// {
// 	return 0;
// }
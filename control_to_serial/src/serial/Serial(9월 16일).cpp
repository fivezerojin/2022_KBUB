#include <ros/ros.h>

#include <serial/serial.h>

#include <std_msgs/String.h>

#include <std_msgs/Empty.h>

#include <std_msgs/Float32.h>

#include <std_msgs/Float64.h>

#include <std_msgs/Bool.h>

#include <std_msgs/Int16.h>

#include <std_msgs/Int32.h>

#include <time.h>

#include <iostream>

#include <unistd.h>
// #include <Windows.h>

using namespace std;

serial::Serial ser;

int i;

uint8_t receive_data[1000];

int receive_data_cnt = 0;

uint8_t control_mode_r;
uint8_t e_stop_r;
uint8_t gear_r;
uint16_t speed_r;
int16_t steer_r;
uint8_t brake_r;
int enc_r;

//========================== transmit_data_value ==========================

uint8_t control_mode_t = 1;
uint8_t e_stop_t = 0;
uint8_t gear_t = 0;
uint16_t speed_t = 0; 
int16_t steer_t = 0;
uint8_t brake_t = 0;
uint8_t alive_t = 0;

int16_t del_brake = 0; 
int16_t delta = 0;	   
int16_t speed = 0;	   
int16_t brake = 0;	   

//========================== mission variable ==========================

int mission = 0;
uint16_t theta;
uint16_t theta2;
bool parking;
double stopline_parking = 10;
double POM;
bool PM;
bool flag1 = true;
double stopline= 100;
bool stop=false;
int d_start=0;
int traffic_light=0;
bool enc_fail;
bool brake_full = false;
bool red = false;
bool green = false;
bool greenleft=false;
bool parkingstop = false;
bool L_stop = false;

int l_brake=0;

//========================== stanley ==========================

void steer_callback(const std_msgs::Float32::ConstPtr& float_msgs)	
{
    del_brake = (float_msgs->data);
	
	if (del_brake > 50 && parking==false)
	{
		delta = del_brake - 100;
		brake = 0;
	}
	else if(del_brake<=50 && parking==false)
	{
		delta = del_brake;
		brake = 0;
	}

	// if(stop && d_start==0){

	// 	int stop_count =0;
	// 	ros::Rate stop_rate(50);
	// 	do
	// 	{
	// 	d_start=1;
	// 	brake = 200;
	// 	speed=0;
	// 	stop_rate.sleep();
	// 	stop_count++;
	// 	} while (stop_count <151);
	// }
	// else if(d_start==1) {
	// 	brake=0;
	// 	d_start=0;
	// }

	if (mission == 10 || mission == 6) //6 주차 10 주차예비
	{
		if (!parking)
		{
			speed = 8;
			brake = 5;
		}
		// else
		// {
		// 	speed = 0;
		// 	brake = 200;
		// 	parkingstop = true;
		// }
	}
	else if (mission == 7 || mission == 8) //7 배달받기 8 배달하기
	{
		speed = 5;//8
		brake = 0;
	}
	else if (mission == 5)
	{
		speed = 5;//8
		brake = 8;
	}
	else
	{
		// if (speed_r > 17)
		// {
		// 	speed = 17; //15
		// }

		if (abs(delta) >= 0 && abs(delta) <= 4)
		{
			speed = 15; //15   //5
			//delta = delta * 0.9;
			// if(red ==true)
			// {
			// 	cout<<"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;
			// 	if(stopline == 100)
			// 	{
			// 		speed = 15;
			// 	}
			// 	else if(stopline <= 11 && stopline >8)
			// 	{
			// 		speed = 15; //15
			// 		brake = 10;
			// 	}
			// 	else if(stopline <= 8 && stopline >= 6) 
			// 	{
			// 		speed = 6;
			// 		brake = 5;
			// 	} 
				// else if (stopline < 6)
				// {
				// 	speed = 0;
				// 	int stop_count =0;
				// 	ros::Rate stop_rate(50);
				// 	do
				// 	{
				// 	brake = 200;
				// 	stop_rate.sleep();
				// 	cout<<"brake~~~~~~~~~~~~~~~~~~~~~~~~~"<<endl;
				// 	stop_count++;
				// 	} while (stop_count <151);		
				// }
			// }
		}
		
		else if (abs(delta) > 4 && abs(delta) <= 6)
		{
			speed = 15; //10
		}

		else if (abs(delta) > 6 && abs(delta) < 10)
		{
			speed = 10; //8
		}

		else
		{
			speed = 10; //6
		}
	}


	steer_t = -delta * 71;
	speed_t = speed * 10;
	brake_t = brake;
}

//========================== mission callback ==========================

void mission_callback(const std_msgs::Int16::ConstPtr &msg)
{
	mission = msg->data;
}

//========================== parking ==========================


void parking_cb(const std_msgs::Bool::ConstPtr &msg)     //주차 1단계
{
	parking = (msg->data); //0 go 1 stop
	// if (flag1)
	// {
		////////0915 주석	
		if (parking == true)
		{
			printf("333333333333333333333333333333333333333333\n");	
			delta = 0; //0
			speed = 0; //0
			brake = 200; //200
			parkingstop=true;			
		}
		// else
		// {
		// 	delta = 0; //0
		// 	speed = 7; //0
		// 	brake = 0;
		// }
		/////////////////////////////////////////////////////


        // if(l_brake==0)
		// {
		// 	if (parking == true)
		// 	{
		// 		printf("333333333333333333333333333333333333333333\n");	
		// 		delta = 0; //0
		// 		speed = 0; //0
		// 		brake = 200; //200
		// 		parkingstop=true;			
		// 	}
		// 	else
		// 	{
		// 		delta = 0; //0
		// 		speed = 6; //0
		// 		brake = 0;
		// 	}

		// 	// else if(parking ==false){
		// 	// 	cout<<"879845155111331131"<<endl;
		// 	// }
		
		// 	if(brake_r>100)
		// 	{
		// 		sleep(2);
		// 		cout<<"sssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssss"<<'\n';
		// 		std_msgs::Bool s;
		// 		s.data=true;
		// 		lidar_s.publish(s);
		// 		brake=0;
		// 		l_brake=1;
				

		// 	}

			
		// }
		steer_t = delta * 71;
		speed_t = speed * 10;
		brake_t = brake;
	// }
}

void theta_callback(const std_msgs::Float64::ConstPtr &float_msgs)     //주차 3단계
{
	theta = (float_msgs->data);

	if (!L_stop)
	{
		sleep(1);
		L_stop=true;
		parking = false;
	}
	else
	{
		if (theta >= 65) //65
		{
			delta = 28; //28
			speed = 8; //8
			brake = 0; //0
		}
		else if (theta < 65 && theta >= 58) //60~65
		{
			delta = 28; //28
			speed = 8; //6 //중요
			brake = 0; //0
		}
		else if (theta < 58) //60
		{
			delta = 2; //1
			speed = 3; //2
			brake = 0; //0
		}
	}

	    // if(parkingstop==true){  
		// 	if (theta >= 65) //65
		// {
		// 	delta = 28; //28
		// 	speed = 8; //8
		// 	brake = 0; //0
		// }
		// else if (theta < 65 && theta >= 58) //60~65
		// {
		// 	delta = 28; //28
		// 	speed = 8; //6 //중요
		// 	brake = 0; //0
		// }
		// else if (theta < 58) //60
		// {
		// 	delta = 2; //1
		// 	speed = 3; //2
		// 	brake = 0; //0
		// }
	

	steer_t = delta * 71;
	speed_t = speed * 10;
	brake_t = brake;
	
	
		
}

void POM_callback(const std_msgs::Float64::ConstPtr &float_msgs)     //주차 4단계 +/-
{
	POM = (float_msgs->data);
	if (POM >= 0)
	{
		PM = true;
	}
	else
	{
		PM = false;
	}
}

void stopline_parking_callback(const std_msgs::Float64::ConstPtr &float_msgs)     //주차 정지선
{
	stopline_parking = (float_msgs -> data);

	if (stopline_parking <= 5.0 || enc_fail == true) //5
	{
		delta = 0;	 //0
		speed = 0;	 //0
		brake = 200; //200
		printf("======================================================");
		steer_t = delta * 71;
		speed_t = speed * 10;
		brake_t = brake;
	}
}

void theta2_callback(const std_msgs::Float64::ConstPtr &float_msgs)     //주차 4단계
{
	theta2 = (float_msgs->data);
	
	if (stopline_parking > 5)
	{
		if (theta2 >= 15) //15
		{
			delta = 10; //13
			speed = 8;	//8
			brake = 0;
		}
		else if (theta2 < 15 && theta2 > 8) //8~15
		{
			delta = 7; //10
			speed = 8;	//8
			brake = 0;	//0
		}
		else if (theta2 <= 8 && theta2 >4) //8
		{
			delta = 3; //3
			speed = 4; //4
			brake = 0; //0
		}
		else if (theta2 <= 4) //4
		{
			delta = 0; //0
			speed = 4; //4
			brake = 0; //0
		}
	}

	if (PM)
	{
		steer_t = delta * 71;
	}
	else
	{
		steer_t = -delta * 71;
	}
	speed_t = speed * 10;
	brake_t = brake;
}

//========================== encoder ==========================

void enc_fail_cb(const std_msgs::Bool::ConstPtr& msg)
{
	enc_fail = msg -> data;
}

void encodersteering_cb(const std_msgs::Float32::ConstPtr &float_msgs)
{
	steer_t = (float_msgs->data);
}

void encoderspeed_cb(const std_msgs::Float32::ConstPtr &msg)
{
	speed_t = msg->data;
}

void encodergear_cb(const std_msgs::Float32::ConstPtr &msg)
{
	gear_t = msg->data;
}

void encoderbrake_cb(const std_msgs::Float32::ConstPtr &msg)
{
	brake_t = msg->data;
}

//========================== stopline ==========================

void stopline_callback(const std_msgs::Float64::ConstPtr & msgs)     //주행 중 정지선
{
	stopline = msgs -> data;
	// int stop_count =0;
	// ros::Rate stop_rate(50);
	// if(brake_full)
	// {
	// 	do
	// 	{
	// 	brake_t = 200;
	// 	stop_rate.sleep();
	// 	stop_count++;
	// 	} while (stop_count <151);
	// 	brake_full = false;
	// }	
}

void traffic_light_callback(const std_msgs::Int32::ConstPtr & msgs)     //주행 중 정지선 신호등
{
	traffic_light = msgs -> data;

	if(traffic_light==1)
	{
		red=true;
		green=false;
		greenleft=false;
	}
	else if(traffic_light==2)
	{
		red=false;
		green=true;
		greenleft=false;
	}
}

//========================== planning ==========================

void plan_steer_callback(const std_msgs::Float64::ConstPtr& msg)
{
    steer_t = int(msg -> data * 71); 
    //steer_t = -(float_msgs -> data * 71); 
}

void plan_speed_callback(const std_msgs::Float64::ConstPtr& msg)
{
    speed_t = uint16_t(msg -> data)*10;    
}

void plan_brake_callback(const std_msgs::Float64::ConstPtr& msg)
{
    brake_t = (msg -> data);
}

void plan_gear_callback(const std_msgs::Float64::ConstPtr& msg)
{
    gear_t = (msg -> data);
}

//========================== delivery ==========================

void delivert_cb(const std_msgs::Bool::ConstPtr& msg)
{
    stop = msg -> data;
}

//========================== main ==========================

int main(int argc, char **argv)
{
	ros::init(argc, argv, "Serial");

	ros::NodeHandle n;

	//========================== mission callback ==========================
	ros::Subscriber mission_sub = n.subscribe("mission", 100, mission_callback);
	
	//========================== for stanley sub ==========================
	ros::Subscriber steering_sub = n.subscribe<std_msgs::Float32>("delta", 1, steer_callback);
	
	//========================== for parking and encoder sub ==========================
	ros::Subscriber speed_sub_encoder = n.subscribe("speed1", 10, encoderspeed_cb);
	ros::Subscriber Steer_sub_encoder = n.subscribe("steer1", 30, encodersteering_cb);
	ros::Subscriber brake_sub = n.subscribe("brake1", 10, encoderbrake_cb);
	ros::Subscriber gear_sub = n.subscribe("gear1", 10, encodergear_cb);	
	ros::Subscriber enc_fail_sub = n.subscribe("enc_fail", 10, enc_fail_cb);
	ros::Subscriber pariking_sub = n.subscribe("/Lidar_Stop", 10, parking_cb);
	ros::Subscriber stopline_parking_sub = n.subscribe("stopline_parking", 10, stopline_parking_callback);
	ros::Subscriber theta_sub = n.subscribe("ParkingAngle_RIGHT", 10, theta_callback);
	ros::Subscriber theta2_sub = n.subscribe("ParkingAngle_FRONT", 10, theta2_callback);
	ros::Subscriber POM_sub = n.subscribe("Plus_or_Minus", 10, POM_callback);

	
	//========================== for stopline sub ==========================
	ros::Subscriber stopline_sub = n.subscribe("stopline", 1, stopline_callback);
	ros::Subscriber traficlight_sub = n.subscribe("traffic_sign_test", 1, traffic_light_callback);

	//========================== for planning sub ==========================
	ros::Subscriber plan_steering_sub = n.subscribe<std_msgs::Float64>("steerP", 10, plan_steer_callback);
    ros::Subscriber plan_speed_sub = n.subscribe<std_msgs::Float64>("speedP", 10, plan_speed_callback);
    ros::Subscriber plan_brake_sub = n.subscribe<std_msgs::Float64>("brakeP", 10, plan_brake_callback);
    ros::Subscriber plan_gear_sub = n.subscribe<std_msgs::Float64>("gearP", 10, plan_gear_callback);

	ros::Subscriber lidar_stop_sub = n.subscribe<std_msgs::Bool>("/Lidar_Stop1", 10, delivert_cb);
	
	//========================== publisher ==========================
	ros::Publisher encoder_pub = n.advertise<std_msgs::Float32>("encoder",1000);
	ros::Publisher speed_pub = n.advertise<std_msgs::Float32>("speed",1000);
	ros::Publisher steer_pub = n.advertise<std_msgs::Float32>("steer",1000);
	ros::Publisher gear_pub = n.advertise<std_msgs::Float32>("gear",1000);
	ros::Publisher control_pub = n.advertise<std_msgs::Float32>("control_mode",1000);

	ros::Publisher vision_pub = n.advertise<std_msgs::Bool>("parkingstop",1000);

	try

	{

		ser.setPort("/dev/ttyUSB0"); //"/dev/ttyUSB0"

		ser.setBaudrate(115200);

		serial::Timeout to = serial::Timeout::simpleTimeout(1000);

		ser.setTimeout(to);

		ser.open();
	}

	catch (serial::IOException &e)

	{

		ROS_ERROR_STREAM("Unable to open port ");

		return -1;
	}

	if (ser.isOpen())
	{

		ROS_INFO_STREAM("Serial Port initialized");
	}
	else
	{

		return -1;
	}

	ros::Rate loop_rate(50);

	while (ros::ok())
	{

		ros::spinOnce();

		if (ser.available())

		{

			// ERP42_data_receive_start-----------------------------------------------------------------------------------------------------

			std_msgs::String result;
			std_msgs::Float32 encoder1;
			std_msgs::Float32 speed1;
			std_msgs::Float32 steer1;
			std_msgs::Float32 gear1;
			std_msgs::Float32 control1;
			std_msgs::Bool parking_stop;

			result.data = ser.read(ser.available());

			encoder1.data = enc_r;

			speed1.data = speed_r;

			steer1.data = steer_r;

			control1.data = control_mode_r;

			gear1.data = gear_r;

			//parking_stop.data = parkingstop;

			for (i = 0; i < sizeof(result.data); i++)

			{

				receive_data[i + receive_data_cnt] = uint8_t(result.data[i]);
			}

			receive_data_cnt += i;

			if (receive_data_cnt >= 18)

			{

				for (i = 0; i < receive_data_cnt - 3; i++)

				{

					if (receive_data[i] == 0x53 & receive_data[i + 1] == 0x54 & receive_data[i + 2] == 0x58)

					{

						if (receive_data_cnt > i + 17)

						{

							if (receive_data[i + 16] == 0x0D & receive_data[i + 17] == 0x0A)

							{

								// system("clear");
								// ROS_INFO_STREAM("DATA_RECEIVED~!!!!!!!!!!!!!!!!!");

								receive_data_cnt = 0;

								control_mode_r = receive_data[i + 3];

								e_stop_r = receive_data[i + 4];

								gear_r = receive_data[i + 5];

								speed_r = receive_data[i + 7] * 256 + receive_data[i + 6];

								steer_r = receive_data[i + 9] * 256 + receive_data[i + 8];

								brake_r = receive_data[i + 10];

								enc_r = (receive_data[i + 14] << 24) + (receive_data[i + 13] << 16) + (receive_data[i + 12] << 8) + receive_data[i + 11];

								speed_pub.publish(speed1);
								encoder_pub.publish(encoder1);
								steer_pub.publish(steer1); 
								control_pub.publish(control1);
								gear_pub.publish(gear1);
								// if (!parking)
								// {
								// 	vision_pub.publish(parking_stop);
								// }
								// else
								// {
								// 	sleep(3);
								// 	vision_pub.publish(parking_stop);
								// 	parking = false;
								// }
								// printf("control_mode : %d | e_stop : %d | gear_r : %d | speed_r : %d | steer_r : %d |brake_r : %d | enc_r : %d", control_mode_r, e_stop_r, gear_r, speed_r, steer_r, brake_r, enc_r);

								for (i = 0; i < receive_data_cnt; i++)

								{

									receive_data[i] = 0;
								}

								break;
							}
						}
					}
				}
			}
		}

		// ERP42_data_receive_end-----------------------------------------------------------------------------------------------------

		control_mode_t = 1;

		e_stop_t = 0;

		speed_t = speed_t;

		gear_t = gear_t;

		steer_t =steer_t;

		brake_t = brake_t;

		uint8_t transmit[14];

		transmit[0] = 'S';

		transmit[1] = 'T';

		transmit[2] = 'X';

		transmit[3] = control_mode_t;

		transmit[4] = e_stop_t;

		transmit[5] = gear_t;

		transmit[6] = speed_t / 256;

		transmit[7] = speed_t % 256;

		transmit[8] = steer_t / 256;

		transmit[9] = steer_t % 256;

		transmit[10] = brake_t;

		transmit[11] = alive_t;

		transmit[12] = 0x0D;

		transmit[13] = 0x0A;

		ser.write(transmit, 14);

		alive_t++;

		alive_t %= 256;

		printf("----------------------------\n");
		printf("Steer : %d\n", steer_t);
		printf("Steer_r : %d\n", steer_r/71);
		printf("Speed : %d\n", speed_t);
		printf("Brake : %d\n", brake_t);
		printf("gear : %d\n", gear_t); 	
		printf("enc_r : %d\n", enc_r);
		printf("mission : %d\n", mission);
	}
	loop_rate.sleep();
}

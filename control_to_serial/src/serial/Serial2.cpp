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

// #include <Windows.h>
using namespace std;

serial::Serial ser;
bool brake_test = false;
bool plan_stop_check = false;
int Flag=0;
bool check_flag = false;
bool flag = false;
bool park_flag = false;
bool isConer = true;
int MissionFlag = 0;
double static_steer_temp = 0.f;
bool static_check = false;
bool static_check2 = true;
bool static_check3 = false;
bool static_check4 = true;
bool brake_check2 = false;
int second_rotate_count  = 0;
bool second_rotate = false;
int i;
clock_t starttime;
clock_t endtime;
clock_t count3;
int count1 = 0;
int count2 = 0;

int j;

uint8_t receive_data[1000];

int receive_data_cnt = 0;

// recevie_data_value

uint8_t control_mode_r;

uint8_t e_stop_r;

uint8_t gear_r;

uint16_t speed_r;

int16_t steer_r;

uint8_t brake_r;

int enc_r;

/////////////////////////

// transmit_data_value

uint8_t control_mode_t = 1;

uint8_t e_stop_t = 0;

uint8_t gear_t = 0;

uint16_t speed_t = 0; 

int16_t steer_t = 0;

uint8_t brake_t = 0;

uint8_t alive_t = 0;

int16_t del_speed = 0; 
int16_t delta = 0;	   
int16_t speed = 0;	   
int16_t brake = 0;	   

int temp = 1;
bool brake_flag = 0;
int StopLineDistance;

// steer value subscribe callback function
void callback_StopLineDistance(const std_msgs::Int32::ConstPtr & msgs)
{
	StopLineDistance = msgs->data;
	
	if(StopLineDistance <= 4)
	{
		delta = 0;
		speed = 0;
		brake = 200;
	}
	// else if(StopLineDistance > 3 && StopLineDistance <= 4)
	// {
		// delta = 0;
		// speed = 5;
		// brake = 15;
	// }
	else
	{
		delta = 0;
		speed = 15;
		brake = 0;
	}
	steer_t = -delta * 71;
	speed_t = speed * 10;
	brake_t = brake;
}

// void steer_callback(const std_msgs::Float32::ConstPtr& float_msgs)
// 
// {
	// del_speed = (float_msgs->data);
	// if(StopLineDistance <= 3)
	// {
		// delta = 0;
		// speed = 0;
		// brake = 200;
	// }
	// 
	// else
	// {	
		// if (del_speed >= 50 && del_speed < 150)
		// {
			// delta = del_speed - 100;
			// speed = 6;
			// brake = 0;
		// }
		// else if (del_speed >= 150 && del_speed < 250)
		// {
			// delta = del_speed - 200;
			// speed = 6;
			// brake = 0;
		// }
		// else if (del_speed >= 250 && del_speed < 350)
		// {
			// delta = del_speed - 300;
			// speed = 6;
			// brake = 0;
		// }
		// else if (del_speed >= 350 && del_speed < 450)
		// {
			// delta = del_speed - 400;
			// speed = 6;
			// brake = 0;
		// }
		// else if (del_speed >= 450)
		// {
			// delta = del_speed - 500;
			// speed = 6;
			// brake = 0;
		// }
	// }
// 

	// steer_t = -delta * 71;
	// speed_t = speed * 10;
	// brake_t = brake;
// 
// 
// }


void encodersteering_callback(const std_msgs::Float32::ConstPtr &float_msgs)

{

	steer_t= (float_msgs->data);

	// steer_t = -(float_msgs -> data * 71);
}


void isStop_Lane_cb(const std_msgs::Bool::ConstPtr &msg)
{
	if (msg->data)
	{
		check_flag = true;
	}
	else
	{
		check_flag = false;
		brake_t = 1;
	}
}
void steer_cb(const std_msgs::Float64::ConstPtr &msg)
{

		steer_t = (msg->data * 71);
	
	
}
void Plan_Stop_cb(const std_msgs::Bool::ConstPtr &msg)
{
	if(Flag == 1 || Flag == 2 || Flag == 4)
	{
		
		plan_stop_check = msg->data;
		if(msg->data)
		{
			brake_check2 = true;
			speed_t = 0;
			brake_t = 200;
		}
		else
		{
			brake_check2 = false;
			brake_t = 1;
		}
	}
}
void encoderspeed_callback(const std_msgs::Float32::ConstPtr &msg)
{
	int speed_t = msg -> data;
	speed_t = speed_t;
}
void Plan_Speed_cb(const std_msgs::Int16::ConstPtr &msg)
{
	int speed_ = msg -> data;
	cout<<"*%^*%*^"<<endl;
	cout<<speed_<<endl;
	speed_t = speed_;
}
void callbackgear(const std_msgs::Float32::ConstPtr &msg)
{
	gear_t = msg->data;
}
void callbackbrake(const std_msgs::Float32::ConstPtr &msg)
{
	brake_t = msg->data;
}

void SetFlag(const std_msgs::Int16::ConstPtr &msg)
{
	Flag = msg->data;
}

void SetMissionFlag(const std_msgs::Int16::ConstPtr &msg)
{
	MissionFlag = msg -> data;
	printf("Mission %d\n", msg->data);
}
void Static_steer(const std_msgs::Float32::ConstPtr &msg)
{
	static_steer_temp = (msg->data) * 71;
	cout<<"123123131231312312313123"<<endl;
	
}

void obstacle()
{
	if(MissionFlag == 4)
	{
		if(static_check2)//small
		{
			if(static_steer_temp != 0 && !static_check)
			{
				static_check = true;
			}
			if(static_check && static_steer_temp == 0.f)
			{
				count1++;
				if(count1 > 300 )
				{
				static_check = false;
				static_check2 = false;
				second_rotate = true;
				}
			}
			else if(static_check)
			{
				count1 = 0;
				
				static_steer_temp = 10.0 * 71; //-20
			}
		}
	}
	if(MissionFlag == 6)
	{
		if(static_check4)
		{
			if(static_steer_temp != 0 && !static_check3)
			{
				static_check3 = true;
			}
			if(static_check3 && static_steer_temp == 0.f)
			{
				count2++;
				if(count2 >600)
				{
					static_check3 = false;
					static_check4 = false;
				}
			}
			if(static_steer_temp != 0.f)
			{
				count2=0;
				static_steer_temp = 20 * 71;
			}
		}
	}
}
int main(int argc, char **argv)
{
	ros::init(argc, argv, "Serial");

	ros::NodeHandle n;

	// steer value subscribe
	// ros::Subscriber steering_sub = n.subscribe<std_msgs::Float32>("delta", 1, steer_callback);

	ros::Subscriber isStop_sub_plan = n.subscribe("isStop_Lane_Pub", 100, isStop_Lane_cb);//isStop_Lane_Pub
	ros::Subscriber steering_sub_plan = n.subscribe("steering", 100, steer_cb);
	ros::Subscriber Park_lane_cb = n.subscribe("Plan_Stop", 10, Plan_Stop_cb);
	ros::Subscriber Speed_cb = n.subscribe("/Speed_Pub", 10, Plan_Speed_cb);

    ros::Subscriber speed_sub_encoder = n.subscribe("speed1", 10, encoderspeed_callback);
	ros::Subscriber Steer_sub_encoder = n.subscribe("steer1", 10, encodersteering_callback);
	ros::Subscriber brake_sub = n.subscribe("brake1", 10, callbackbrake);
	ros::Subscriber gear_sub = n.subscribe("gear1", 10, callbackgear);
	ros::Subscriber Flag_sub = n.subscribe("Plan_Flag", 10, SetFlag);
	ros::Subscriber Static_steer_sub = n.subscribe("sick_static_steer", 10, Static_steer);
	ros::Subscriber vision_sub1 = n.subscribe("stopline", 100, callback_StopLineDistance);

	ros::Publisher encoder_pub = n.advertise<std_msgs::Float32>("encoder",1000);
	ros::Publisher speed_pub = n.advertise<std_msgs::Float32>("speed",1000);
	ros::Publisher steer_pub = n.advertise<std_msgs::Float32>("steer",1000);
	ros::Publisher gear_pub = n.advertise<std_msgs::Float32>("gear",1000);
	ros::Publisher control_pub = n.advertise<std_msgs::Float32>("control_mode",1000);
	
	ros::Subscriber Mission_FLAG = n.subscribe("mission_flag", 100, SetMissionFlag);
	// ros::Publisher gear_pub = n.advertise<std_msgs::Float32>("gear",1000);

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

			result.data = ser.read(ser.available());

			encoder1.data=enc_r;

			speed1.data = speed_r;

			steer1.data = steer_r;

			control1.data = control_mode_r;

			gear1.data = gear_r;

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

		speed_t=speed_t;

		gear_t = gear_t;

		// speed_value ex)3km/h = 30,  5km/h = 50
		if(Flag == 3)
		{
			speed_t = 50;
		}
		else{
			speed_t = speed_t;
		}
		obstacle();
		cout<<"M : "<<MissionFlag<<" s:"<<static_steer_temp<<endl;
		if(MissionFlag == 4 && static_steer_temp != 0)
		{
			steer_t = static_steer_temp;
			printf("12312312312312312\n");
		}
		else if(MissionFlag == 6 && static_steer_temp != 0)
		{
			steer_t = static_steer_temp;
		}
		else if(MissionFlag == 4 && second_rotate)
		{
			steer_t = -25 * 71;
			second_rotate_count++;
			if(second_rotate_count > 300)
			{
				second_rotate = false;
			}
		}
		else
		{
			steer_t = steer_t;
		}
		
		

		
		brake_t = brake_t;
		if(brake_test)
		{
			brake_t = 50;
		}
		if(plan_stop_check)
		{
			brake_t = 180;
			steer_t = 0;
			speed_t = 0;
		}

		if (brake_flag == 1)
		{
			brake_flag = 0;

			if (brake_t < 149)
				brake_t += 3;
			else if (brake_t >= 150)
				e_stop_t = 1;
		}
		if(MissionFlag == 5)
			speed_t = 70;
		if(MissionFlag == 4)
			speed_t = 50;
		// steer_t -= (0.6*71);
		// if(steer_t < -28*71)
		// 	steer_t = -28*71;
		// if(Flag == 2 && brake_check2)
		// {
		// 	brake_t = 180;
		// }
		// double speed_offset = 0.f;
		// speed_offset = 1/(double)speed_r;
		// speed_t = speed_t - speed_offset;
		if(brake_t > 150)
		{
			speed_t = 0;
		}
		if(brake_t == 0 && speed_t == 0)
		{
			speed_t = 80;
		}
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
		printf("Speed : %d\n", speed_t);
		printf("Brake : %d\n", brake_t);
		printf("Flag : %d\n", Flag);
			
		
		//printf("\n");
	}


	loop_rate.sleep();
}

// #include <ros/ros.h>

// #include <serial/serial.h>

// #include <std_msgs/String.h>

// #include <std_msgs/Empty.h>

// #include <std_msgs/Float32.h>

// #include <std_msgs/Float64.h>

// #include <std_msgs/Bool.h>

// #include <std_msgs/Int16.h>

// #include <iostream>

// // #include <Windows.h>
// using namespace std;

// serial::Serial ser;
// bool brake_test = false;
// bool plan_stop_check = false;
// int Flag=0;
// bool check_flag = false;
// bool flag = false;
// bool park_flag = false;
// bool isConer = true;
// int MissionFlag = 0;
// double static_steer_temp = 0.f;
// bool static_check = false;
// bool static_check2 = true;
// int i;

// int j;

// uint8_t receive_data[1000];

// int receive_data_cnt = 0;

// // recevie_data_value

// uint8_t control_mode_r;

// uint8_t e_stop_r;

// uint8_t gear_r;

// uint16_t speed_r;

// int16_t steer_r;

// uint8_t brake_r;

// int enc_r;

// /////////////////////////

// // transmit_data_value

// uint8_t control_mode_t = 1;

// uint8_t e_stop_t = 0;

// uint8_t gear_t = 0;

// uint16_t speed_t = 0; 

// int16_t steer_t = 0;

// uint8_t brake_t = 1;

// uint8_t alive_t = 0;

// int16_t del_brake = 0; 
// int16_t delta = 0;	   
// int16_t speed = 0;	   
// int16_t brake = 0;	   

// int temp = 1;
// bool brake_flag = 0;

// // steer value subscribe callback function
// void steer_callback(const std_msgs::Float32::ConstPtr& float_msgs)

// {

//         del_brake = (float_msgs->data);
// 		if (del_brake > 50)
// 		{
// 			delta = del_brake - 100;
// 			brake = 25;
// 		}
// 		else
// 		{
// 			delta = del_brake;
// 			brake = 0;

// 		}
// 		// delta = del_brake;



// 		if (speed_r > 15)
// 		{
// 			speed=15;//15
// 			// brake=0;
// 		}
		
//         if(abs(delta) >= 0 && abs(delta) < 7 )
//         {
//             speed = 15;//15
			
//         }


//         else if(abs(delta) >= 7 && abs(delta) < 15)
//         {
//             speed = 8;//8
// 			// delta = delta ;
//         }

//         else
//         {
//             speed =4;
// 			// brake = 20;
//         }



// 	steer_t = -delta * 71;
// 	speed_t = speed * 10;
// 	brake_t = brake;

// 	//  (oneborn)  steer_t = (float_msgs->data * 71);

// 	//if angle is reverse use under command instead of up command

// 	// steer_t = -(float_msgs -> data * 71); 

// }


// void encodersteering_callback(const std_msgs::Float32::ConstPtr &float_msgs)

// {

// 	steer_t= (float_msgs->data);

// 	// steer_t = -(float_msgs -> data * 71);
// }


// void isStop_Lane_cb(const std_msgs::Bool::ConstPtr &msg)
// {
// 	if (msg->data)
// 	{
// 		check_flag = true;
// 	}
// 	else
// 	{
// 		check_flag = false;
// 		brake_t = 1;
// 	}
// }
// void steer_cb(const std_msgs::Float64::ConstPtr &msg)
// {

// 		steer_t = (msg->data * 71);
	
	
// }
// void Plan_Stop_cb(const std_msgs::Bool::ConstPtr &msg)
// {
// 	if(Flag == 1 || Flag == 2 || Flag == 4)
// 	{
// 		plan_stop_check = msg->data;
// 		if(msg->data)
// 		{
// 			brake_t = 200;
// 		}
// 		else
// 		{
// 			brake_t = 1;
// 		}
// 	}
// }
// void encoderspeed_callback(const std_msgs::Float32::ConstPtr &msg)
// {
// 	int speed_t = msg -> data;
// 	speed_t = speed_t;
// }
// void Plan_Speed_cb(const std_msgs::Int16::ConstPtr &msg)
// {
// 	int speed_ = msg -> data;
// 	cout<<speed_<<endl;
// 	speed_t = speed_;
// }
// void callbackgear(const std_msgs::Float32::ConstPtr &msg)
// {
// 	gear_t = msg->data;
// }
// void callbackbrake(const std_msgs::Float32::ConstPtr &msg)
// {
// 	brake_t = msg->data;
// }

// void SetFlag(const std_msgs::Int16::ConstPtr &msg)
// {
// 	Flag = msg->data;
// }

// void SetMissionFlag(const std_msgs::Int16::ConstPtr &msg)
// {
// 	MissionFlag = msg -> data;
// 	printf("Mission %d\n", msg->data);
// }
// void Static_steer(const std_msgs::Float32::ConstPtr &msg)
// {
// 	if(MissionFlag == 4)
// 	{
// 		if(static_check2)
// 		{
// 			static_steer_temp = (int)(msg->data) * 71;
// 			if(static_steer_temp != 0 && !static_check)
// 			{
// 				static_check = true;
// 			}
// 			if(static_check && static_steer_temp == 0.f)
// 			{
// 				static_check = false;
// 				static_check2 = false;
// 			}
// 		}
// 	}
// }

// int main(int argc, char **argv)
// {
// 	ros::init(argc, argv, "Serial");

// 	ros::NodeHandle n;

// 	// steer value subscribe
// 	ros::Subscriber steering_sub = n.subscribe<std_msgs::Float32>("delta", 1, steer_callback);

// 	ros::Subscriber isStop_sub_plan = n.subscribe("isStop_Lane_Pub", 100, isStop_Lane_cb);//isStop_Lane_Pub
// 	ros::Subscriber steering_sub_plan = n.subscribe("steering", 100, steer_cb);
// 	ros::Subscriber Park_lane_cb = n.subscribe("Plan_Stop", 10, Plan_Stop_cb);
// 	ros::Subscriber Speed_cb = n.subscribe("Speed_Pub", 10, Plan_Speed_cb);

//     ros::Subscriber speed_sub_encoder = n.subscribe("speed1", 10, encoderspeed_callback);
// 	ros::Subscriber Steer_sub_encoder = n.subscribe("steer1", 10, encodersteering_callback);
// 	ros::Subscriber brake_sub = n.subscribe("brake1", 10, callbackbrake);
// 	ros::Subscriber gear_sub = n.subscribe("gear1", 10, callbackgear);
// 	ros::Subscriber Flag_sub = n.subscribe("Plan_Flag", 10, SetFlag);
// 	ros::Subscriber Static_steer_sub = n.subscribe("sick_static_steer", 10, Static_steer);

// 	ros::Publisher encoder_pub = n.advertise<std_msgs::Float32>("encoder",1000);
// 	ros::Publisher speed_pub = n.advertise<std_msgs::Float32>("speed",1000);
// 	ros::Publisher steer_pub = n.advertise<std_msgs::Float32>("steer",1000);
// 	ros::Publisher gear_pub = n.advertise<std_msgs::Float32>("gear",1000);
// 	ros::Publisher control_pub = n.advertise<std_msgs::Float32>("control_mode",1000);
	
// 	ros::Subscriber Mission_FLAG = n.subscribe("mission_flag", 100, SetMissionFlag);
// 	// ros::Publisher gear_pub = n.advertise<std_msgs::Float32>("gear",1000);

// 	try

// 	{

// 		ser.setPort("/dev/ttyUSB0"); //"/dev/ttyUSB0"

// 		ser.setBaudrate(115200);

// 		serial::Timeout to = serial::Timeout::simpleTimeout(1000);

// 		ser.setTimeout(to);

// 		ser.open();
// 	}

// 	catch (serial::IOException &e)

// 	{

// 		ROS_ERROR_STREAM("Unable to open port ");

// 		return -1;
// 	}

// 	if (ser.isOpen())
// 	{

// 		ROS_INFO_STREAM("Serial Port initialized");
// 	}
// 	else
// 	{

// 		return -1;
// 	}

// 	ros::Rate loop_rate(50);

// 	while (ros::ok())
// 	{

// 		ros::spinOnce();

// 		if (ser.available())

// 		{

// 			// ERP42_data_receive_start-----------------------------------------------------------------------------------------------------

// 			std_msgs::String result;
// 			std_msgs::Float32 encoder1;
// 			std_msgs::Float32 speed1;
// 			std_msgs::Float32 steer1;
// 			std_msgs::Float32 gear1;
// 			std_msgs::Float32 control1;

// 			result.data = ser.read(ser.available());

// 			encoder1.data=enc_r;

// 			speed1.data = speed_r;

// 			steer1.data = steer_r;

// 			control1.data = control_mode_r;

// 			gear1.data = gear_r;

// 			for (i = 0; i < sizeof(result.data); i++)

// 			{

// 				receive_data[i + receive_data_cnt] = uint8_t(result.data[i]);
// 			}

// 			receive_data_cnt += i;

// 			if (receive_data_cnt >= 18)

// 			{

// 				for (i = 0; i < receive_data_cnt - 3; i++)

// 				{

// 					if (receive_data[i] == 0x53 & receive_data[i + 1] == 0x54 & receive_data[i + 2] == 0x58)

// 					{

// 						if (receive_data_cnt > i + 17)

// 						{

// 							if (receive_data[i + 16] == 0x0D & receive_data[i + 17] == 0x0A)

// 							{

// 								// system("clear");
// 								// ROS_INFO_STREAM("DATA_RECEIVED~!!!!!!!!!!!!!!!!!");

// 								receive_data_cnt = 0;

// 								control_mode_r = receive_data[i + 3];

// 								e_stop_r = receive_data[i + 4];

// 								gear_r = receive_data[i + 5];

// 								speed_r = receive_data[i + 7] * 256 + receive_data[i + 6];

// 								steer_r = receive_data[i + 9] * 256 + receive_data[i + 8];

// 								brake_r = receive_data[i + 10];

// 								enc_r = (receive_data[i + 14] << 24) + (receive_data[i + 13] << 16) + (receive_data[i + 12] << 8) + receive_data[i + 11];

// 								speed_pub.publish(speed1);
// 								encoder_pub.publish(encoder1);
// 								steer_pub.publish(steer1); 
// 								control_pub.publish(control1);
// 								gear_pub.publish(gear1);
// 								// printf("control_mode : %d | e_stop : %d | gear_r : %d | speed_r : %d | steer_r : %d |brake_r : %d | enc_r : %d", control_mode_r, e_stop_r, gear_r, speed_r, steer_r, brake_r, enc_r);

// 								for (i = 0; i < receive_data_cnt; i++)

// 								{

// 									receive_data[i] = 0;
// 								}

// 								break;
// 							}
// 						}
// 					}
// 				}
// 			}
// 		}

// 		// ERP42_data_receive_end-----------------------------------------------------------------------------------------------------

// 		control_mode_t = 1;

// 		e_stop_t = 0;

// 		speed_t=speed_t;

// 		gear_t = gear_t;

// 		// speed_value ex)3km/h = 30,  5km/h = 50
// 		if(Flag == 3)
// 		{
// 			speed_t = 50;
// 		}
// 		else{
// 			speed_t = speed_t;
// 		}
// 		cout<<"M : "<<MissionFlag<<" s:"<<static_steer_temp<<endl;
// 		if(MissionFlag == 4 && static_steer_temp != 0)
// 		{
// 			steer_t = static_steer_temp;
// 			printf("12312312312312312\n");
// 		}
// 		else
// 		{
// 			steer_t = steer_t;
// 		}
		
		

		
// 		brake_t = brake_t;
// 		if(brake_test)
// 		{
// 			brake_t = 50;
// 		}
// 		if(plan_stop_check)
// 		{
// 			brake_t = 180;
// 			steer_t = 0;
// 			speed_t = 0;
// 		}
// 		if (check_flag)
// 		{
// 			if (brake_t < 190)
// 			{
// 				brake_t += 10;
// 			}
// 			else
// 			{
// 				sleep(3);
// 			}
// 		}

// 		if (brake_flag == 1)
// 		{
// 			brake_flag = 0;

// 			if (brake_t < 149)
// 				brake_t += 3;
// 			else if (brake_t >= 150)
// 				e_stop_t = 1;
// 		}

// 		//steer_t -= (0.6*71);
// 		//i//f(steer_t < -28*71)
// 		//	steer_t = -28*71;
// 		uint8_t transmit[14];

// 		transmit[0] = 'S';

// 		transmit[1] = 'T';

// 		transmit[2] = 'X';

// 		transmit[3] = control_mode_t;

// 		transmit[4] = e_stop_t;

// 		transmit[5] = gear_t;

// 		transmit[6] = speed_t / 256;

// 		transmit[7] = speed_t % 256;

// 		transmit[8] = steer_t / 256;

// 		transmit[9] = steer_t % 256;

// 		transmit[10] = brake_t;

// 		transmit[11] = alive_t;

// 		transmit[12] = 0x0D;

// 		transmit[13] = 0x0A;

// 		ser.write(transmit, 14);

// 		alive_t++;

// 		alive_t %= 256;

// 		printf("----------------------------\n");
// 		printf("Steer : %d\n", steer_t);
// 		printf("Speed : %d\n", speed_t);
// 		printf("Brake : %d\n", brake_t);
// 		printf("Flag : %d\n", Flag);
			
		
// 		//printf("\n");
// 	}


// 	loop_rate.sleep();
// }

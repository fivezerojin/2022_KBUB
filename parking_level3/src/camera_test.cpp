#include "ros/ros.h"
//#include "line_parking/parking_line.h"
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <stdlib.h>
#include "std_msgs/UInt16.h"
#include <iostream>
#include <math.h>
//#include <serial_example/ParkPoint.h>
#include <velodyne_filter/ParkPoint.h>

using namespace cv;
using namespace std;
int temp_count = 0;
int tx = 0;
int ty = 120;
int offset = 0;
bool check = false;
bool Parking = false;
const double PI = 3.1415926535897932384626433832795028841971693993751058209;
bool lane_stop = false;
Mat preprocessing(Mat img);
Mat bird_eyes_view(Mat img);
Mat img =  Mat::zeros(640, 480, CV_8UC3);

bool foo = false;
//void parking_callback(std_msgs::UInt16 msg){
//	if(msg.data==1) foo=true;
//}
bool isStart = false;
int temp = 0;
ros::Subscriber sub_can_park;

void Start(const velodyne_filter::ParkPoint::ConstPtr &park_point_info)
{
	//if(!temp)
	//{

	isStart = park_point_info->can_park.data;

	//////cout << "isStart" << input->can_park.data << '\n';
	////cout << "isStart" << '\n';
	//}
	//}
}

void isStart_cb(const std_msgs::Int16::ConstPtr &msg)
{
	if(msg->data == 2)
	{
		isStart = true;
	}
	else
	{
		isStart = false;
	}
	
}
void img_cb(const sensor_msgs::ImageConstPtr& source)
{
	cv_bridge::CvImagePtr img_ = cv_bridge::toCvCopy(source, sensor_msgs::image_encodings::BGR8);
	img = img_ -> image;
	//imshow("src", img);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "line_parking");
	ros::NodeHandle nh;
	ros::Publisher parking_test_pub = nh.advertise<std_msgs::Float64>("parking_test_msg", 100);
	ros::Publisher park_lane_stop = nh.advertise<std_msgs::Bool>("parking_lane_stop", 100);

	ros::Subscriber Image_sub = nh.subscribe("/image_converter/output_video", 100, img_cb);
	
	ros::Subscriber Park_start_sub = nh.subscribe("Plan_Flag", 100, isStart_cb);
	//ros::Publisher line_parking_pub = nh.advertise<line_parking::parking_line>("line_parking_msg", 100);
	ros::Rate loop_rate(30);
	sub_can_park = nh.subscribe("/park_point", 10, Start);

	//ros::Subscriber parking_flagSub = nh.subscribe<std_msgs::UInt16>("/parking_flag", 10, parking_callback);

	//line_parking::parking_line msg;
	std_msgs::Float64 msgT;
	int cnt_temp = 0;
	int cnt = 0;
	std_msgs::Bool msg_lane;

//	VideoCapture cap1(0);
	//	VideoCapture cap1("/home/usera/park2.avi");
//	cap1.set(cv::CAP_PROP_FRAME_WIDTH, 640); //이미지 사이즈 정해줌
//	cap1.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
	//VideoCapture cap1("/home/pear/Downloads/challenge.mp4");
	VideoCapture cap1("/home/usera/catkin_ws/src/parking_level3/src//test6.mp4");

	
	Mat park_img;
	bool check = false;

	float p_slope = -2;
	float p_bias = 0;
	// float p_theta = -20;
	float p_theta = -20; //-20
	int start_y = 0;
	int thresh1 = 50;
	int thresh2 = 150;
	//isStart = 1;1
	//while (cap1.i111sOpened() && ros::ok())
	while(ros::ok())
	{
		// isStart = 1;
		if (isStart)
		{
			
			// webcam test
			// cap1 >> park_img;
			// cv::resize(std_msgs::UInt::UIntmg, park_img, cv::Size(640, 360), 0, 0);
			// cv::inRange(park_img, cv::Scalar(0, 0, 0), cv::Scalar(100, 100, 100), park_img);
			cv::imshow("src", img);
			// blur(park_img, park_img, Size(5, 5));
			// Canny(park_img, park_img, 50, 150, 3);

			// video test, real
			//cap1 >> img;

			//.//.//cv::imshow("src", img);
			Mat park_img = preprocessing(img);
			//park_img = park_img(Range(0, park_img.rows), Range(int(park_img.cols*0.4), park_img.cols));
			blur(park_img, park_img, Size(5, 5));
			Canny(park_img, park_img, thresh1, thresh2, 3); //50 150
			printf("%d %d\n", thresh1, thresh2);
			//printf("canny thresh : %d %d\n", thresh1, thresh2);

			// cvtColor(park_img, park_img, COLOR_BGR2GRAY);

			vector<Vec4i> lines;
			HoughLinesP(park_img, lines, 1, CV_PI / 180, 30, 30, 3);

			cvtColor(park_img, park_img, COLOR_GRAY2BGR);

			int cnt = 0, sum_slope = 0, sum_bias = 0;
			float bias = 0;
			float theta = 0;
			int p_height = 0;

			float max = 0;

			int p[4] = {
				0,
			};


			int b_bias = 0;
			int b_slope = 0;
			int b_theta = 0;
			int b_start_y = 0;
			int b_p[4];

			int min = 2100000000;
			int diff = 0;

			for (int i = 0; i < lines.size(); i++)
			{
				Vec4i L = lines[i];
				float dx = L[2] - L[0];
				float dy = L[3] - L[1];
				float slope = dy / dx; // inverse slope

				int x_inter = -int(bias/slope);

				bias = L[3] - L[2] * slope;

				cv::line(park_img, Point(L[0], L[1]), Point(L[2], L[3]), Scalar(0, 255, 0), 1, LINE_AA);

				float radian = atan(slope);
				theta = (radian * 180) / 3.141592;
				//std:://cout << "slope : "<<slope<<"    theta : "<<theta<<endl;
				
				if (theta < 10 && theta > -10)
				{
					int center = int((L[0]+L[2])/2);
					if(center<park_img.cols*0.6 && center>park_img.cols*0.4)
						p_height = int((L[1] + L[3]) / 2);
				//	cv::line(park_img, Point(L[2], 0), Point(L[2], 150), Scalar(255, 255, 255), 1, LINE_AA);
				//	cv::line(park_img, Point(L[0], 0), Point(L[2], 150), Scalar(255, 255, 0), 1, LINE_AA);
					//printf("==== height : %d ====\n", p_height);
				}
				
				
				//if ((theta > p_theta - 40 && theta < p_theta + 40))
				
				int new_theta=0, new_ptheta = 0;
				if(theta<0){
					new_theta =theta;
				}
				else{
					new_theta = theta;
				}
				if(p_theta<0){
					new_ptheta = p_theta;
				}
				else{
					new_ptheta =  p_theta;
				}


				if(abs(new_theta - new_ptheta) < 5.0)
				{
					//cout<<"theta : " <<theta<<"   p_theta : "<<p_theta<<endl; 
					//std::cout << "slope : "<<slope<<"    theta : "<<theta<<endl;
					cv::line(park_img, Point(L[0], L[1]), Point(L[2], L[3]), Scalar(255, 0, 0), 3, LINE_AA);
					//if (bias > max) {
					//cout<<"bias : "<<bias<<", p_bias :  "<<p_bias<<"check : "<<check<<endl;
					int temp = int((120 - bias) / slope);
					if(((temp < tx + 50) && (temp > tx-50)) || !check)
					//if ((bias < p_bias*1.1 && bias > p_bias*0.9) || !check)
					{
						// cv::line(park_img, Point(0, L[1]), Point(park_img.cols, L[1]), Scalar(255, 255, 0), 3, LINE_AA);
						// cv::line(park_img, Point(L[0], L[1]), Point(L[2], L[3]), Scalar(255, 0, 255), 3, LINE_AA);
						check = true;
						diff = abs(p_bias - bias);	
						//cout<<"diff : "<<diff<<", min : " <<min<<endl;
						
						for (int i = 0; i < 4; i++)
						{
							p[i] = L[i];
						}
						p_bias = bias;
						p_slope = slope;
						p_theta = theta;
						start_y = p[1];

						// ------------ 여기부터 수정중 -------------
						if((diff < min) ){
							min = diff;
							//cout<<"test : "<<min<<endl;
							//return 0;					
							for (int i = 0; i < 4; i++)
							{
								b_p[i] = L[i];
							}
							b_bias = bias;
							b_slope = slope;
							b_theta = theta;
							b_start_y = L[1];
						}
						// ------------ 여기까지 수정중 -------------
						// marker.pose.orientation.w = 1;
						// max = bias;
						
					}
				}
			}
			// for (int i = 0; i < 4; i++)
			// {
			// 	p[i] = b_p[i];
			// }
			// p_bias = b_bias;
			// p_slope = b_slope;
			// p_theta = b_theta;
			// start_y = b_p[1];
			// //cout<<"p+bias : "<< p_bias <<", p_slope : "<<p_slope<<", p_theta : "<<p_theta<<", start_y : " <<start_y<<endl;



			int sx, sy, ex, ey;
			sy = park_img.rows;
			sx = int((sy - p_bias) / p_slope);
			ey = 0;
			ex = int((ey - p_bias) / p_slope);

		//	int tx, ty;
		//	ty = 120;
			tx = int((ty - p_bias) / p_slope);

			

			int px = 0, py = 120;
			float line_slope = float((park_img.rows) / ((float(park_img.cols / 2) + 100) - (ex + offset)));
			float line_bias = -line_slope * (offset + ex);
			
			px = int((120 - line_bias) / line_slope);

			// --------------------------------- please write ros code here ----------------------------------------
			float Data_x, Data_y, data = 0;
			// Data_x = ((px - (int(park_img.cols/2)+100)) * 3.75) / 290; //3.75m = 290pixel // 315 is offset
			Data_x = int((px - (int(park_img.cols / 2) + 100))) * 30; //3.75m = 290pixel // 315 is offset
			data = 1;

			//msg.Data_x = Data_x;
			//msg.Data_x1 = Data_x;
			//msg.Data_x2 = Data_x;
			//msg.Data_y = 0;
			//msg.Data_y1 = 2;
			//msg.Data_y2 = 3;

			// msgT.data = 0;

			////printf("%f\n", Data_x);

			//ROS_INFO("%f", msg.Data_x);
			//ROS_INFO("%f", msg.Data_y);
			//ROS_INFO("%f", msgT.data);

			//printf("foo = %d\n", foo);
			// ------------------------------------------------------------------------------------------------------
			// msg.Data_x = 0;
			// msg.Data_y = 3;

			// if(foo){
			// 	msgT.data= 1;
			// 	line_parking_pub.publish(msg);
			// }

			//line_parking_pub.publish(msg);

			// else {
			// 	msg.Data_x = 0;
			// 	msg.Data_y = 3;
			// 	line_parking_pub.publish(msg);
			// }
			double distance = sqrt(pow((ex + offset - (park_img.cols / 2)), 2) + pow(park_img.rows, 2));
			double angle = acos(park_img.rows / distance) * 180 / PI;
			//printf("distance : %f\n", distance);
			//printf("angle : %f\n", angle);
			if (angle > 28.0)
			{
				angle = 28.0;
			}
			if (ex + offset < park_img.cols / 2)
			{
				angle *= -1;
			}
			msgT.data = angle;
			temp_count++;
			if (angle < 5.0 && angle > -5.0)
			{
				if (p_height > int(park_img.rows * 0.6))
				{
					cv::line(park_img, Point(0, p_height), Point(park_img.cols, p_height), Scalar(255, 0, 255), 5, LINE_AA);
					msg_lane.data = true;
					cout<<"true"<<endl;
				}
				
			}
			else
			{
				msg_lane.data = false;
				//cout<<"false"<<endl;
			}


			//}
			//	else
			//	{
			//		msg_lane.data = false;
			//	}

			parking_test_pub.publish(msgT);
			park_lane_stop.publish(msg_lane);
			//printf("angle' : %f\n", angle);
			cv::line(park_img, Point(sx, sy), Point(ex, ey), Scalar(0, 0, 255), 1, LINE_AA);
			cv::line(park_img, Point(sx + offset, sy), Point(ex + offset, ey), Scalar(0, 0, 255), 1, LINE_AA);
			cv::line(park_img, Point(park_img.cols / 2, 0), Point(park_img.cols / 2, park_img.rows), Scalar(255, 255, 255), 1, LINE_AA);
			cv::line(park_img, Point(ex + offset, ey), Point(park_img.cols / 2, park_img.rows), Scalar(255, 255, 0), 1, LINE_AA);
			//cv::line(park_img, Point(0, ty), Point(park_img.cols, ty), Scalar(175, 175, 175), 1, LINE_AA);
			// cv::line(park_img, Point(0, start_y), Point(park_img.cols, start_y), Scalar(255, 255, 0), 1, LINE_AA);
			cv::circle(park_img, Point(tx, ty), 5, Scalar(255, 255, 255), 3);
			
			// printf("----------------tx :  %d   theta : %lf------------\n", tx, theta);    //------------------------------------------출력값 확인--------------------
			// cv::circle(park_img, Point(px, py), 5, Scalar(0, 255, 255), 3);
			cv::resize(park_img, park_img, Size(park_img.cols * 1, park_img.rows * 1));
			cv::imshow("park", park_img);
			char k = cv::waitKey(1);
			//printf("-----------------------------------%d----------------\n", k);
			if (k == 'k')
			{
				thresh1 += 10;
			}
			else if (k == 'l')
			{
				thresh2 += 10;
			}
			else if (k == 'n')
			{
				thresh1 -= 10;
			}
			else if (k == 'm')
			{
				thresh2 -= 10;
			}

			//loop_rate.sleep();
		}

		
		////cout << isStart << endl;
		ros::spinOnce();
		loop_rate.sleep();
		// system("clear");
	}
}

Mat preprocessing(Mat img)
{
	Mat img_resize;
	cv::resize(img, img_resize, cv::Size(640, 360), 0, 0);
	////.//.//imshow("src", img_resize);
	//cvtColor(img_resize, img_resize, COLOR_BGR2GRAY);

	 Mat img_park;
	 img_park = bird_eyes_view(img_resize);
	//imshow("bird", img_park);
	Mat img_bgr;
	cv::inRange(img_park, cv::Scalar(180, 180, 180), cv::Scalar(255, 255, 255), img_bgr); //220~250
	//cv::rectangle(img_bgr, Point(img_bgr.cols*0.6,0), Point(img_bgr.cols, img_bgr.rows), Scalar(0, 0, 0), -1);
	Mat img_binary;
	cvtColor(img_bgr, img_binary, COLOR_GRAY2BGR);
	//.//.//imshow("binary", binary);
//	imshow("binary", img_binary);
	return img_bgr;
}

// Mat bird_eyes_view(Mat img) {
// 	int width = img.cols;
// 	int height = img.rows;
// 	img = img(Range(height * 3 / 5, height), Range(0, width));
// 	width = img.cols;
// 	height = img.rows;
// 	Mat warp_matrix;
// 	Point2f warp_src_point[4];
// 	Point2f warp_dst_point[4];
// 	warp_src_point[0].x = 0; warp_src_point[0].y = height;
// 	warp_src_point[1].x = width; warp_src_point[1].y = height;
// 	warp_src_point[2].x = 0; warp_src_point[2].y = 0;
// 	warp_src_point[3].x = width; warp_src_point[3].y = 0;
// 	warp_dst_point[0].x = width * 2 / 5; warp_dst_point[0].y = height;
// 	warp_dst_point[1].x = width * 3 / 5; warp_dst_point[1].y = height;
// 	warp_dst_point[2].x = 0; warp_dst_point[2].y = 0;
// 	warp_dst_point[3].x = width; warp_dst_point[3].y = 0;
// 	warp_matrix = cv::getPerspectiveTransform(warp_src_point, warp_dst_point);
// 	Mat dst;
// 	cv::warpPerspective(img, dst, warp_matrix, cv::Size(width, height));
// 	return dst;
// }

Mat bird_eyes_view(Mat img)
{
	int width = img.cols;
	int height = img.rows;
	//이미지의 일부 영역을 제거함(상단 2/5지점까지)
	img = img(Range(height * 0.4, height), Range(0, width)); //k-bub기준
	//img = img(Range(height * 3 / 5, height), Range(0, width)); //digist기준
	width = img.cols;
	height = img.rows;
	Mat warp_matrix;
	Point2f warp_src_point[4];
	Point2f warp_dst_point[4];
	//원본의 좌표(좌하단, 우하단, 좌상단, 우상단)
	warp_src_point[0].x = 0;
	warp_src_point[0].y = height;
	warp_src_point[1].x = width;
	warp_src_point[1].y = height;
	warp_src_point[2].x = 0;
	warp_src_point[2].y = 0;
	warp_src_point[3].x = width;
	warp_src_point[3].y = 0;

	//목표이미지의 좌표(좌하단, 우하단, 좌상단, 우상단)
	//warp_dst_point[0].x = width * 0.43; warp_dst_point[0].y = height;
	//warp_dst_point[1].x = width * 0.57; warp_dst_point[1].y = height;
	warp_dst_point[0].x = width * 0.38;
	warp_dst_point[0].y = height;
	warp_dst_point[1].x = width * 0.55;
	warp_dst_point[1].y = height;
	warp_dst_point[2].x = 0;
	warp_dst_point[2].y = 0;
	warp_dst_point[3].x = width;
	warp_dst_point[3].y = 0;
	warp_matrix = cv::getPerspectiveTransform(warp_src_point, warp_dst_point);
	Mat dst;
	cv::warpPerspective(img, dst, warp_matrix, cv::Size(width, height)); //버드아이뷰 전환
	return dst;
}
#include "opencv2/opencv.hpp"
#include <iostream>
#include <ros/ros.h>
#include "std_msgs/Bool.h"
#include "std_msgs/Float64.h"

#include <sstream>
#define PI 3.141592
using namespace std;
using namespace cv::ml;
using namespace cv;
Mat preprocessing_second_level(Mat img);
void parking_cb(const std_msgs::Bool::ConstPtr &msgs);

Mat preprocessing_forth_level(Mat img);
Mat bird_eyes_view_inverse_for_front_camara(Mat img);
Mat bird_eyes_view_for_front_camara(Mat img);
Mat mask_filter(Mat img, int _mask_w, int _mask_h, int thresh);
Mat start_level_4;
bool start_level4 =false;
Mat preprocessing(Mat img);
Mat bird_eyes_view(Mat img);
Mat warp_matrix_inv;
void on_mouse(int event, int x, int y, int flags, void *);
Point ptOld1;
Mat bird_eyes_view_inverse(Mat img);
int isStop = 0;
int SecondLevel = 0;
void mouse_callback(int event, int x, int y, int flags, void *param);
Mat img_color;
Mat img_resize5;
Mat img_resize11;
Mat img_resize_new;
Mat img_resize100;
Mat img_resize1000;
Mat img_hsv;
Mat OutputImage;
int H, S, V;
int a1 = 0, a2 = 0, a3 = 0, a4 = 0, a5 = 0;
int x11 = 0, x22 = 0, x33 = 0, y11 = 0, y22 = 0, y33 = 0;
bool first_level = true;
bool second_level = false;
Mat after_first_level;
bool parking1_ = false;

ros::Publisher parking_pub;
ros::Subscriber Parking_level1_sub;
ros::Publisher parking_level3_pub;
ros::Publisher parking_level4_pub;
ros::Publisher parking_level4_pub1;
ros::Publisher stopline_pub;

void parking_cb(const std_msgs::Bool::ConstPtr &msgs)
{
	parking1_ = msgs->data;
}

int main(int argc, char **argv)
{
	VideoCapture cap("/home/kroad/catkin_ws/src/parking/src/park2.mp4");
	VideoCapture cap1("/home/kroad/catkin_ws/src/parking/src/park3.mp4");
	//-------------------------side camara
	// VideoCapture cap(4); //side camara
	cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
	cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
	//-------------------------front camara
	// VideoCapture cap1(5); //front camara
	cap1.set(cv::CAP_PROP_FRAME_WIDTH, 640);
	cap1.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
	ros::init(argc, argv, "parking_publisher");

	ros::NodeHandle nh;
	parking_pub = nh.advertise<std_msgs::Bool>("parking", 100);
	Parking_level1_sub = nh.subscribe("parking1", 10, parking_cb);
	parking_level3_pub = nh.advertise<std_msgs::Float64>("ParkingAngle_RIGHT", 10);
	parking_level4_pub = nh.advertise<std_msgs::Float64>("ParkingAngle_FRONT", 10);
	parking_level4_pub1 = nh.advertise<std_msgs::Float64>("Plus_or_Minus", 10);
	stopline_pub = nh.advertise<std_msgs::Float64>("stopline", 100);
	ros::Rate loop_rate(50);

	if (!cap.isOpened())
	{
		cerr << "cap load fail!!\n"
			 << endl;
	}

	if (!cap1.isOpened())
	{
		cerr << "cap1 load fail!!\n"
			 << endl;
	}

	Mat frame1, frame2, frame3, frame_for_front_camara;

	while (ros::ok())
	{
		cap >> frame1;					//side camara
		cap1 >> frame_for_front_camara; //front camara
		// imshow("frame_for_front_camara",frame_for_front_camara);
		if (!cap.isOpened())
		{
			cerr << "finish!\n"
				 << endl;
		}
		bool count = false;

		double fps = cap.get(CAP_PROP_FPS);
		cout << "FPS: " << fps << endl;
		int delay = cvRound(1000 / fps);

		Mat img_resize;
		cv::resize(frame1, img_resize, cv::Size(640, 480), 0, 0);

		Mat img_resize6;
		cv::resize(frame_for_front_camara, img_resize6, cv::Size(640, 480), 0, 0);
		// imshow("img_resize6",img_resize6);
		
		//imshow("img_resize", img_resize);
		// Mat dst333333;
		// cvtColor(img_resize,dst333333,CV_BGR2GRAY);

		// Mat adaptive;
		// adaptiveThreshold(dst333333,adaptive,255,ADAPTIVE_THRESH_MEAN_C,THRESH_BINARY,5,5);
		// // imshow("adaptive",adaptive);
		img_resize5 = img_resize.clone();
		img_resize100 = img_resize.clone();
		img_resize1000 = img_resize6.clone();
		img_color = img_resize.clone();

		namedWindow("img_color", WINDOW_AUTOSIZE); //window for output mask
		setMouseCallback("img_color", mouse_callback);
		imshow("img_color", img_resize);
		// waitKey();

		Mat HSV_image;
		cvtColor(img_resize, HSV_image, COLOR_BGR2HSV);

		Scalar lower_white = Scalar(H, S, V);
		Scalar upper_white = Scalar(255, 50, 255);

		Mat white_image;
		inRange(HSV_image, lower_white, upper_white, white_image);
		// imshow("white_image", white_image);

		Mat img_warp;
		img_warp = bird_eyes_view(white_image);
		//imshow("warp_img", img_warp);

		// namedWindow("white_image@@@");img_resize100
		// waitKey();
		Mat dx, dy;
		Sobel(img_warp, dx, CV_32FC1, 1, 0);
		Sobel(img_warp, dy, CV_32FC1, 0, 1);

		Mat fmag, mag;
		magnitude(dx, dy, fmag);
		fmag.convertTo(mag, CV_8UC1);
		Mat edge = mag > 130;
		// imshow("edge", edge);

		if (first_level == true)
		{
			frame2 = preprocessing(edge);
			// Mat img_resize1;
			// cv::resize(frame1, img_resize1, cv::Size(640, 480), 0, 0);

			Mat final1;
			addWeighted(img_resize100, 0.5, frame2, 0.5, 0, final1);

			if (isStop == 1)
			{
				count = true;
				putText(final1, "STOP!", Point(500, 50), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 255, 255));
				// cout << "end!!!!!!!!!!!!!!!!!!!" << endl;
				first_level = false;
				second_level = true;
				// cout << "second_level" << second_level << endl;
			}

			else if (isStop == 0)
			{
				count = false;
				putText(final1, "GO!", Point(500, 50), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 255, 255));
			}

			imshow("final1", final1);
			std_msgs::Bool msg;
			msg.data = count;
			// ROS_INFO("%s",msg.data); // data 메시지를표시한다
			cout << "msg.data: " << count << endl;
			parking_pub.publish(msg); // 메시지를발행한다
		}

		else if (first_level == false && second_level == true)
		{
			
			start_level_4 = preprocessing_second_level(edge);
			Mat final5;
			addWeighted(img_resize5, 0.5, start_level_4, 0.5, 0, final5);
			imshow("final5", final5);

			if(SecondLevel == 1)
			{
				second_level=false;
				start_level4=true;
			}
		}

		
		else if (second_level == false && start_level4 == true)
		{
			// cout << "44444444444444444444444" << endl;
			
			
			frame3=preprocessing_forth_level(img_resize6);
			// imshow("frame3",frame3);
			Mat final7;
			addWeighted(img_resize1000, 0.5, frame3, 0.5, 0, final7);
			imshow("final7", final7);
		}

		// cout << frame2.type() << img_resize1.type() << endl;
		// cout << frame2.size() << img_resize1.size() << endl;
		// imshow("frame2", frame2);

		if (waitKey(delay) == 27)
		{
			break;
			printf("end");
		}

		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}

Mat preprocessing(Mat img)
{
	vector<Vec4i> lines;
	HoughLinesP(img, lines, 1, CV_PI / 180, 160, 50, 5);

	Mat dst;
	cvtColor(img, dst, COLOR_GRAY2BGR);
	Mat dst1(Size(640, 480), CV_8UC1);
	Mat dst2(Size(640, 480), CV_8UC1);
	Mat dst3(Size(640, 480), CV_8UC1);

	for (Vec4i l : lines)
	{
		// line(dst,Point(l[0],l[1]),Point(l[2],l[3]),Scalar(0,0,255),2,LINE_AA);
		float dx = l[2] - l[0], dy = l[3] - l[1];
		float k = dy / dx;
		// cout<<"k: "<<k<<endl;
		float d = sqrt(pow(l[2] - l[0], 2) + pow(l[3] - l[1], 2));
		float y1 = l[1] - 10;
		float y2 = l[3] + 10;

		if (k > 0 && k < 0.2)
		{
			line(img, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 0, 255), 2, LINE_AA);
			// cout<<"k1: "<<k<<endl;
			// cout<<"Point(l[0],l[1])"<<Point(l[0],l[1])<<endl;
			// cout<<"Point(l[2], l[3])"<<Point(l[2], l[3])<<endl;
			if (d > 170)
			{
				line(dst1, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255, 255, 255), 20, LINE_AA);

				// rectangle(dst1, Point(l[0],y1),Point(l[2], y2),Scalar(0,255,255),10);
				// cout<<"Point(l[0], l[1]) : "<<Point(l[0], l[1])<<endl;
				// cout<<"Point(l[2], l[3]) : "<<Point(l[2], l[3])<<endl;
			}
			//circle(dst, Point(l[0], l[1]), 5, Scalar(0, 255, 0), -1, LINE_AA);
			//circle(dst, Point(l[2], l[3]), 5, Scalar(0, 255, 0), -1, LINE_AA);
			//cout << "Point(l[0], l[1])_red" << Point(l[0], l[1]) << endl;
			//cout << "Point(l[2], l[3])_red" << Point(l[2], l[3]) <<"\n"<< endl;f
		}
		else if (fabs(k) > 4)
		{
			line(img, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255, 0, 0), 2, LINE_AA);
			// cout<<"k2: "<<k<<endl;
			// cout<<"d: "<<d<<endl;
			// cout<<"Point(l[0],l[1])"<<Point(l[0],l[1])<<endl;
			// cout<<"Point(l[2], l[3])"<<Point(l[2], l[3])<<endl;
			if (d > 135)
			{
				line(dst2, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255, 255, 255), 20, LINE_AA);
				// cout<<"Point(l[0],l[1])"<<Point(l[0],l[1])<<endl;
				// cout<<"Point(l[2], l[3])"<<Point(l[2], l[3])<<endl;
			}
			//circle(dst, Point(l[0], l[1]), 5, Scalar(0, 255, 0), -1, LINE_AA);
			//circle(dst, Point(l[2], l[3]), 5, Scalar(0, 255, 0), -1, LINE_AA);
			//cout << "Point(l[0], l[1])_blue" << Point(l[0], l[1]) << endl;
			//cout << "Point(l[2], l[3])_blue" << Point(l[2], l[3]) << "\n" <<endl;
		}
		else if (k > -1.1 && k < -0.8)
		{
			line(img, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 255, 0), 2, LINE_AA);
			// cout<<"k3: "<<k<<endl;
			// cout<<"Point(l[0],l[1])"<<Point(l[0],l[1])<<endl;
			// cout<<"Point(l[2], l[3])"<<Point(l[2], l[3])<<endl;
			if (d > 180)
			{
				line(dst3, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255, 255, 255), 20, LINE_AA);
				// cout<<"Point(l[0],l[1])"<<Point(l[0],l[1])<<endl;
				// cout<<"Point(l[2], l[3])"<<Point(l[2], l[3])<<endl;
			}
			//circle(dst, Point(l[0], l[1]), 5, Scalar(0, 255, 0), -1, LINE_AA);
			//circle(dst, Point(l[2], l[3]), 5, Scalar(0, 255, 0), -1, LINE_AA);
			//cout << "Point(l[0], l[1])_purple" << Point(l[0], l[1]) << endl;
			//cout << "Point(l[2], l[3])_purple" << Point(l[2], l[3]) << "\n" << endl;
		}
	}

	// imshow("dst", dst);
	//imshow("dst1", dst1);
	// imshow("dst2", dst2);
	//imshow("dst3", dst3);

	int num = (int)(dst1.total() * 0.1);
	for (int i = 0; i < num; i++)
	{
		int x1 = rand() % dst1.cols;
		int y1 = rand() % dst1.rows;
		dst1.at<uchar>(y1, x1) = (i % 2) * 255;

		int x2 = rand() % dst2.cols;
		int y2 = rand() % dst2.rows;
		dst2.at<uchar>(y2, x2) = (i % 2) * 255;

		int x3 = rand() % dst3.cols;
		int y3 = rand() % dst3.rows;
		dst3.at<uchar>(y3, x3) = (i % 2) * 255;
	}

	Mat dst_final_blur1, dst_final_blur2, dst_final_blur3;
	medianBlur(dst1, dst_final_blur1, 29);
	medianBlur(dst2, dst_final_blur2, 29);
	medianBlur(dst3, dst_final_blur3, 29);
	// imshow("dst_final_blur1",dst_final_blur1);
	// imshow("dst_final_blur2",dst_final_blur2);
	// imshow("dst_final_blur3",dst_final_blur3);

	Mat labels1, stats1, centroids1, labels2, stats2, centroids2, labels3, stats3, centroids3;
	int cnt1 = connectedComponentsWithStats(dst_final_blur1, labels1, stats1, centroids1);
	int cnt2 = connectedComponentsWithStats(dst_final_blur2, labels2, stats2, centroids2);
	int cnt3 = connectedComponentsWithStats(dst_final_blur3, labels3, stats3, centroids3);

	Mat dst_1, dst_2, dst_3;
	cvtColor(dst_final_blur1, dst_1, COLOR_GRAY2BGR);
	cvtColor(dst_final_blur2, dst_2, COLOR_GRAY2BGR);
	cvtColor(dst_final_blur3, dst_3, COLOR_GRAY2BGR);
	isStop = 0;

	for (int i = 1; i < cnt1; i++)
	{
		int *p = stats1.ptr<int>(i);
		x11 = p[0] + p[2] / 2;
		y11 = p[1] + p[3] / 2;
		if (p[4] > 4500)
		{
			rectangle(dst_1, Rect(p[0], p[1], p[2], p[3]), Scalar(0, 0, 255), 2);
			// cout<<"x11: "<<x11<<" y11: "<<y11<<endl;
			//cout << "stats1 : " << p[4] << endl;
			a1 = 1;
		}
		else
		{
			a1 = 0;
		}
	}
	cout << "a1: " << a1 << endl;
	// cout<<"x11: "<<x11<<"\n"<<endl;

	for (int i = 1; i < cnt2; i++)
	{
		int *p = stats2.ptr<int>(i);
		x22 = p[0] + p[2] / 2;
		y22 = p[1] + p[3] / 2;
		if (p[4] > 4000 && p[4] < 13000)
		{
			rectangle(dst_2, Rect(p[0], p[1], p[2], p[3]), Scalar(255, 0, 0), 2);
			// cout<<"x22: "<<x22<<" y22: "<<y22<<endl;
			//cout << "stats2 : " << p[4] << endl;
			a2 = 1;
		}
		else
		{
			a2 = 0;
		}
	}
	cout << "a2: " << a2 << endl;
	// cout<<"x22: "<<x22<<"\n"<<endl;

	for (int i = 1; i < cnt3; i++)
	{
		int *p = stats3.ptr<int>(i);
		x33 = p[0] + p[2] / 2;
		y33 = p[1] + p[3] / 2;
		if (p[4] > 5000 && p[4] < 11000)
		{
			rectangle(dst_3, Rect(p[0], p[1], p[2], p[3]), Scalar(0, 255, 0), 2);
			// cout<<"x33: "<<p[0]+p[2]/2<<" y33: "<<p[1]+p[3]/2<<endl;
			//cout << "stats3 : " << p[4] << endl;
			a3 = 1;
		}
		else
		{
			a3 = 0;
		}
	}
	cout << "a3: " << a3 << endl;
	// cout<<"x33: "<<x33<<"\n"<<endl;

	Mat final = dst_1 + dst_2 + dst_3;

	if (x33 < x22)
	{
		a4 = 1;
	}
	else
	{
		a4 = 0;
	}
	cout << "a4: " << a4 << endl;

	a5 = a1 + a2 + a3 + a4;
	cout << "a5: " << a5 << endl;

	if (a5 == 4)
	{
		isStop = 1;
		cout << "stop!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << endl;
		//putText(final,"STOP!",Point(500,50),FONT_HERSHEY_SIMPLEX,1,Scalar(0,255,255));
	}
	else if (a5 != 4)
	{
		isStop = 0;
		//putText(final,"GO!",Point(500,50),FONT_HERSHEY_SIMPLEX,1,Scalar(0,255,255));
	}
	//cout<<"isStop: "<<isStop<<"\n"<<endl;

	//imshow("dst_1",dst_1);
	//imshow("dst_2",dst_2);
	//imshow("dst_3",dst_3);
	// imshow("final",final);

	Mat warp_inv;
	warp_inv = bird_eyes_view_inverse(final);
	//imshow("warp_inv", warp_inv);

	return warp_inv;
}

Mat bird_eyes_view(Mat img)
{
	int width = img.cols;
	int height = img.rows;

	width = img.cols;
	height = img.rows;
	Mat warp_matrix;
	Point2f warp_src_point[4];
	Point2f warp_dst_point[4];

	warp_src_point[0].x = 15;
	warp_src_point[0].y = 350;
	warp_src_point[1].x = 470;
	warp_src_point[1].y = 350;
	warp_src_point[2].x = 15;
	warp_src_point[2].y = 90;
	warp_src_point[3].x = 470;
	warp_src_point[3].y = 90;

	warp_dst_point[0].x = 150;
	warp_dst_point[0].y = height * 0.8;
	warp_dst_point[1].x = width - 150;
	warp_dst_point[1].y = height * 0.8;
	warp_dst_point[2].x = 150;
	warp_dst_point[2].y = 0;
	warp_dst_point[3].x = width - 150;
	warp_dst_point[3].y = 0;
	warp_matrix = cv::getPerspectiveTransform(warp_src_point, warp_dst_point);

	Mat dst;
	cv::warpPerspective(img, dst, warp_matrix, cv::Size(width, height));
	//imshow("dst", dst); top view

	return dst;
}

void on_mouse(int event, int x, int y, int flags, void *)
{
	switch (event)
	{
	case EVENT_LBUTTONDOWN:
		ptOld1 = Point(x, y);
		cout << "EVENT_LBUTTONDOWN: " << x << ", " << y << endl;
		break;
	case EVENT_LBUTTONUP:
		cout << "EVENT_LBUTTONUP: " << x << ", " << y << endl;
		break;
	}
}

Mat bird_eyes_view_inverse(Mat img)
{
	int width = img.cols;
	int height = img.rows;

	width = img.cols;
	height = img.rows;
	Mat warp_matrix;
	Point2f warp_src_point[4];
	Point2f warp_dst_point[4];

	warp_src_point[0].x = 15;
	warp_src_point[0].y = 350;
	warp_src_point[1].x = 470;
	warp_src_point[1].y = 350;
	warp_src_point[2].x = 15;
	warp_src_point[2].y = 90;
	warp_src_point[3].x = 470;
	warp_src_point[3].y = 90;

	warp_dst_point[0].x = 150;
	warp_dst_point[0].y = height * 0.8;
	warp_dst_point[1].x = width - 150;
	warp_dst_point[1].y = height * 0.8;
	warp_dst_point[2].x = 150;
	warp_dst_point[2].y = 0;
	warp_dst_point[3].x = width - 150;
	warp_dst_point[3].y = 0;

	warp_matrix = cv::getPerspectiveTransform(warp_src_point, warp_dst_point);
	invert(warp_matrix, warp_matrix_inv);

	Mat dst1;
	cv::warpPerspective(img, dst1, warp_matrix_inv, cv::Size()); 

	return dst1;
}

void mouse_callback(int event, int x, int y, int flags, void *param)
{
	if (event == EVENT_LBUTTONDBLCLK)
	{
		Vec3b color_pixel = img_color.at<Vec3b>(y, x);

		Mat bgr_color = Mat(1, 1, CV_8UC3, color_pixel);

		Mat hsv_color;
		cvtColor(bgr_color, hsv_color, COLOR_BGR2HSV);

		H = hsv_color.at<Vec3b>(0, 0)[0];
		S = hsv_color.at<Vec3b>(0, 0)[1];
		V = hsv_color.at<Vec3b>(0, 0)[2];

		cout << "H= " << H << endl;
		cout << "S= " << S << endl;
		cout << "V = " << V << "\n"
			 << endl;

		H = H - 200;
		S = S - 50;
		V = V - 70;

		if (H < 0)
			H = 0;

		if (S < 0)
			S = 0;

		if (V < 0)
			V = 0;
	}
}

Mat preprocessing_second_level(Mat img)
{		
	float theta = 89;
	float theta_live = 89;

	// while (theta_live > 57)
	// {
		vector<Vec4i> lines;
		HoughLinesP(img, lines, 1, CV_PI / 180, 160, 50, 5);

		Mat dst;
		cvtColor(img, dst, COLOR_GRAY2BGR);

		Mat dst4(Size(640, 480), CV_8UC1);

		for (Vec4i l : lines)
		{

			//line(img,Point(l[0],l[1]),Point(l[2],l[3]),Scalar(0,0,255),2,LINE_AA);
			//cout << "lines.size: " << lines.size() << endl;
			float dx = l[2] - l[0], dy = l[3] - l[1];
			float k = dy / dx;
			float radian = atan(k);
			float theta = (radian * 180) / PI;
			float d = l[1];
			float y1 = l[1] - 10;
			float y2 = l[3] + 10;
			float x0 = (l[0] + l[2]) / 2;
			float y0 = (l[1] + l[3]) / 2;
			//cout << "theta: " << theta << endl;

			// if (theta < 89 && theta>88)
			// {
			// 	cout << "stop!!!!!!!!!!!!!!!!" << theta << endl;
			// 	isStop = 1;
			// }
			// cout<<"theta: "<<theta<<endl;
			if ((theta > 50 && theta < 88) || (theta<-70 && theta > -89))
			{
				line(dst, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 0, 255), 2, LINE_AA);
				//cout << "Point(l[0], l[1]): " << Point(l[0], l[1]) << " Point(l[2], l[3]): " << Point(l[2], l[3]) << endl;
				line(dst4, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255, 255, 255), 15, LINE_AA);
				// cout << "theta_original: " << theta << endl;
			}

			// else if (isStop == 0)
			// {
			// 	cout << "nothing" << endl;
			// }
		}

		//  imshow("dst", dst);
		// imshow("dst4", dst4);

		int num = (int)(dst4.total() * 0.1);
		for (int i = 0; i < num; i++)
		{
			int x44 = rand() % dst4.cols;
			int y44 = rand() % dst4.rows;
			dst4.at<uchar>(y44, x44) = (i % 2) * 255;
		}
		// Mat dst4(Size(640, 480), CV_8UC1);

		Mat dst_final_blur4;
		medianBlur(dst4, dst_final_blur4, 29);
		// imshow("dst_final_blur4",dst_final_blur4);

		Mat labels4, stats4, centroids4;
		int cnt4 = connectedComponentsWithStats(dst_final_blur4, labels4, stats4, centroids4);

		Mat dst_4;
		cvtColor(dst_final_blur4, dst_4, COLOR_GRAY2BGR);

		for (int i = 1; i < cnt4; i++)
		{
			int *p = stats4.ptr<int>(i);
			// rectangle(dst_4, Rect(p[0], p[1], p[2], p[3]), Scalar(0, 0, 255), 2);
			// cout << "stats1 : " << p[4] << endl;
			// float theta = (atan((float)p[3] / p[2]) * 180) / PI;
			float x1 = p[0], y1 = p[1], x2 = p[0] + p[2], y2 = p[1] + p[3];
			float k1 = ((y2 - y1) / (x2 - x1));
			float radian1 = atan(k1);
			float theta_live = (radian1 * 180) / PI;

			std_msgs::Float64 ParkingAngle_RIGHT_msg;
			// cout<<"77777777777777777777777"<<endl;  
			if (p[4] > 5000)
			{
				if ((theta_live > 50 && theta_live < 88) || (theta_live<-70 && theta_live > -89))
				{
					// cout << "*************************************8" << endl;
					rectangle(dst_4, Rect(p[0], p[1], p[2], p[3]), Scalar(0, 0, 255), 2);
					//cout << "stats1 : " << p[4] << endl;img_resize10
					ParkingAngle_RIGHT_msg.data = theta_live;
					parking_level3_pub.publish(ParkingAngle_RIGHT_msg);
					ROS_INFO("%f", ParkingAngle_RIGHT_msg.data);
					
					if(theta_live<55)
					{
						cout<<"stop!!!!!!!!!!!!!!!!!!!!!111"<<endl;
						SecondLevel=1;
					}
				}
			}
		}
		// imshow("dst_4", dst_4);
		// cout<<"SecondLevel: "<<SecondLevel<<endl;
		Mat warp_inv4;
		warp_inv4 = bird_eyes_view_inverse(dst_4);
		//imshow("warp_inv4", warp_inv4);
		return warp_inv4;
	//}
}

Mat preprocessing_forth_level(Mat img)
{	
	
	// imshow("img_resize10",img_resize10);
	// cout<<"!!!!!!!!!!!!!!!!!!!!!111"<<endl;	
	img_color = img.clone();
	img_resize_new = img.clone();
	// img_resize2 = img_resize.clone();

	// namedWindow("img_color", WINDOW_AUTOSIZE); //window for output mask
	// setMouseCallback("img_color", mouse_callback);
	// imshow("img_color", img);
	// // waitKey();					

	Mat HSV_image;
	cvtColor(img, HSV_image, COLOR_BGR2HSV);

	Scalar lower_white = Scalar(0, 0, 220);
	Scalar upper_white = Scalar(255, 255, 255);

	Mat white_image;
	inRange(HSV_image, lower_white, upper_white, white_image);
	imshow("white_image", white_image);
	// cout<<"^^^^^^^^^^^^^^^^^^^"<<endl;	
	Mat img_warp11;
	img_warp11 = bird_eyes_view_for_front_camara(white_image);
	// imshow("warp_img", img_warp11);

	Mat img_warp111 = img_warp11.clone();

	Mat dx, dy;
	Sobel(img_warp11, dx, CV_32FC1, 1, 0);
	Sobel(img_warp11, dy, CV_32FC1, 0, 1);

	Mat fmag, mag;
	magnitude(dx, dy, fmag);
	fmag.convertTo(mag, CV_8UC1);
	Mat edge = mag > 130;
	//imshow("edge", edge);

	vector<Vec4i> lines;
	HoughLinesP(edge, lines, 1, CV_PI / 180, 160, 50, 5);

	Mat dst;
	cvtColor(edge, dst, COLOR_GRAY2BGR);
	Mat dst4(Size(640, 480), CV_8UC1);

	for (Vec4i l : lines)
	{
		// line(dst,Point(l[0],l[1]),Point(l[2],l[3]),Scalar(0,0,255),2,LINE_AA);
		//cout << "lines.size: " << lines.size() << endl;
		float dx = l[2] - l[0], dy = l[3] - l[1];
		float k = dy / dx;
		float radian = atan(k);
		float theta = (radian * 180) / PI;
		float d = l[1];
		float y1 = l[1] - 10;
		float y2 = l[3] + 10;
		float x0 = (l[0] + l[2]) / 2;
		float y0 = (l[1] + l[3]) / 2;
		// cout << "theta: " << theta << endl;
		std_msgs::Float64 Plus_or_Minus_msg;

		if (theta > -20 && theta < 20)
		{
			line(dst, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 0, 255), 2, LINE_AA);
			// cout << "theta: " << theta << endl;
			//cout << "Point(l[0], l[1]): " << Point(l[0], l[1]) << " Point(l[2], l[3]): " << Point(l[2], l[3]) << endl;
			line(dst4, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255, 255, 255), 15, LINE_AA);
			// cout << "theta_original: " << theta << endl;
			Plus_or_Minus_msg.data = theta;
			parking_level4_pub1.publish(Plus_or_Minus_msg);
			// ROS_INFO("%f",Plus_or_Minus_msg.data);
		}
	}
	// imshow("dst", dst);
	// imshow("dst4", dst4);

	int num = (int)(dst4.total() * 0.1);
	for (int i = 0; i < num; i++)
	{
		int x44 = rand() % dst4.cols;
		int y44 = rand() % dst4.rows;
		dst4.at<uchar>(y44, x44) = (i % 2) * 255;
	}

	Mat dst_final_blur4;
	medianBlur(dst4, dst_final_blur4, 29);
	// imshow("dst_final_blur4",dst_final_blur4);

	Mat labels4, stats4, centroids4;
	int cnt4 = connectedComponentsWithStats(dst_final_blur4, labels4, stats4, centroids4);

	Mat dst_4;
	cvtColor(dst_final_blur4, dst_4, COLOR_GRAY2BGR);

	for (int i = 1; i < cnt4; i++)
	{
		int *p = stats4.ptr<int>(i);
		// rectangle(dst_4, Rect(p[0], p[1], p[2], p[3]), Scalar(0, 0, 255), 2);
		// cout << "stats1 : " << p[4] << endl;
		// float theta = (atan((float)p[3] / p[2]) * 180) / PI;
		float x1 = p[0], y1 = p[1], x2 = p[0] + p[2], y2 = p[1] + p[3];
		float k1 = ((y2 - y1) / (x2 - x1));
		float radian1 = atan(k1);
		float theta_live = (radian1 * 180) / PI;

		// cout<<"theta: "<<theta<<endl;
		std_msgs::Float64 ParkingAngle_FRONT_msg;
		if (p[4] > 1000 && p[4] < 20000)
		{
			rectangle(dst_4, Rect(p[0], p[1], p[2], p[3]), Scalar(0, 0, 255), 2);
			// cout << "stats1 : " << p[4] << endl;
			//cout << "p[2]: " << p[2] << " p[3]: " << p[3] << endl;
			// cout << "theta_live: " << theta << endl;
			ParkingAngle_FRONT_msg.data = theta_live;
			parking_level4_pub.publish(ParkingAngle_FRONT_msg);
			ROS_INFO("%f", ParkingAngle_FRONT_msg.data);
			if (theta_live < 9)
			{
				cout << "stop change angle!" << endl;
			}
		}
	}
	// imshow("dst_4", dst_4);

	Mat warp_inv;
	warp_inv = bird_eyes_view_inverse_for_front_camara(dst_4);
	// imshow("warp_inv", warp_inv);

	Mat img_integral;
	cv::integral(img_warp111, img_integral);

	Mat img_mask;
	img_mask = mask_filter(img_integral, 5, 5, 95);
	//imshow("img_mask", img_mask);

	Mat warp_inv11;
	warp_inv11 = bird_eyes_view_inverse_for_front_camara(img_mask);
	// imshow("warp_inv11", warp_inv11);

	Mat final2;
	cv::resize(warp_inv11, final2, cv::Size(640, 480), 0, 0);
	// imshow("img_resize",final2);

	Mat final3;
	cv::resize(warp_inv, final3, cv::Size(640, 480), 0, 0);
	// imshow("img_resize",final3);
	// cout<<"final3.size: "<<final3.size()<<endl;
	// cout<<"final3.type: "<<final3.type()<<endl;
	Mat final4 = final2 + final3;
	// imshow("final4",final4);

	Mat final5;
	addWeighted(img_resize_new, 0.5, final4, 0.5, 0, final5);

	int count = 0;

	if (isStop == 7)
	{
		count = 7;
		putText(final5, "7M", Point(500, 100), FONT_HERSHEY_SIMPLEX, 3, Scalar(0, 0, 255));
	}

	else if (isStop == 6)
	{
		count = 6;
		putText(final5, "6M", Point(500, 100), FONT_HERSHEY_SIMPLEX, 3, Scalar(0, 0, 255));
	}

	else if (isStop == 5)
	{
		count = 5;
		putText(final5, "5M", Point(500, 100), FONT_HERSHEY_SIMPLEX, 3, Scalar(0, 0, 255));
	}

	else if (isStop == 4)
	{
		count = 4;
		putText(final5, "4M", Point(500, 100), FONT_HERSHEY_SIMPLEX, 3, Scalar(0, 0, 255));
	}

	else if (isStop == 3)
	{
		count = 3;
		putText(final5, "STOP!!", Point(380, 100), FONT_HERSHEY_SIMPLEX, 3, Scalar(0, 0, 255));
	}
	std_msgs::Float64 stopline_msg;
	stopline_msg.data = count;
	ROS_INFO("%f", stopline_msg.data);	// data 메시지를표시한다
	stopline_pub.publish(stopline_msg); // 메시지를발행한다

	// imshow("final5", final5);
	
	return final5;
}

Mat bird_eyes_view_for_front_camara(Mat img)
{
	int width = img.cols;
	int height = img.rows;

	width = img.cols;
	height = img.rows;
	Mat warp_matrix;
	Point2f warp_src_point[4];
	Point2f warp_dst_point[4];

	// warp_src_point[0].x = 26;
	// warp_src_point[0].y = 395;
	// warp_src_point[1].x = 635;
	// warp_src_point[1].y = 420;
	// warp_src_point[2].x = 26;
	// warp_src_point[2].y = 120;
	// warp_src_point[3].x = 635;
	// warp_src_point[3].y = 120;

	warp_src_point[0].x = 15;
	warp_src_point[0].y = 320;
	warp_src_point[1].x = 630;
	warp_src_point[1].y = 320;
	warp_src_point[2].x = 220;
	warp_src_point[2].y = 140;
	warp_src_point[3].x = 470;
	warp_src_point[3].y = 140;

	// warp_dst_point[0].x = 150;
	// warp_dst_point[0].y = height * 0.8;bird_eyes_view_inverse
	// warp_dst_point[2].x = 150;
	// warp_dst_point[2].y = 0;
	// warp_dst_point[3].x = width - 150;
	// warp_dst_point[3].y = 0;

	warp_dst_point[0].x = 150;
	warp_dst_point[0].y = height;
	warp_dst_point[1].x = width - 150;
	warp_dst_point[1].y = height;
	warp_dst_point[2].x = 150;
	warp_dst_point[2].y = 0;
	warp_dst_point[3].x = width - 150;
	warp_dst_point[3].y = 0;

	warp_matrix = cv::getPerspectiveTransform(warp_src_point, warp_dst_point);

	Mat dst;
	cv::warpPerspective(img, dst, warp_matrix, cv::Size(width, height));
	//imshow("dst", dst); top view

	return dst;
}

Mat bird_eyes_view_inverse_for_front_camara(Mat img)
{
	int width = img.cols;
	int height = img.rows;

	width = img.cols;
	height = img.rows;
	Mat warp_matrix;
	Point2f warp_src_point[4];
	Point2f warp_dst_point[4];

	// warp_src_point[0].x = 26;
	// warp_src_point[0].y = 395;
	// warp_src_point[1].x = 635;
	// warp_src_point[1].y = 420;
	// warp_src_point[2].x = 26;
	// warp_src_point[2].y = 120;
	// warp_src_point[3].x = 635;
	// warp_src_point[3].y = 120;

	warp_src_point[0].x = 15;
	warp_src_point[0].y = 320;
	warp_src_point[1].x = 630;
	warp_src_point[1].y = 320;
	warp_src_point[2].x = 220;
	warp_src_point[2].y = 140;
	warp_src_point[3].x = 470;
	warp_src_point[3].y = 140;

	// warp_dst_point[0].x = 150;
	// warp_dst_point[0].y = height * 0.8;
	// warp_dst_point[1].x = width - 150;
	// warp_dst_point[1].y = height * 0.8;
	// warp_dst_point[2].x = 150;
	// warp_dst_point[2].y = 0;
	// warp_dst_point[3].x = width - 150;
	// warp_dst_point[3].y = 0;

	warp_dst_point[0].x = 150;
	warp_dst_point[0].y = height;
	warp_dst_point[1].x = width - 150;
	warp_dst_point[1].y = height;
	warp_dst_point[2].x = 150;
	warp_dst_point[2].y = 0;
	warp_dst_point[3].x = width - 150;
	warp_dst_point[3].y = 0;

	warp_matrix = cv::getPerspectiveTransform(warp_src_point, warp_dst_point);
	invert(warp_matrix, warp_matrix_inv);

	Mat dst1;
	cv::warpPerspective(img, dst1, warp_matrix_inv, cv::Size()); //������̺� ����ȯ
	//imshow("dst1", dst1);

	return dst1;
}

Mat mask_filter(Mat img, int _mask_w, int _mask_h, int thresh)
{
	int height = img.rows;
	int width = img.cols;
	Mat img_maskfilter;
	img_maskfilter = Mat::zeros(height, width, CV_8UC1);
	Mat img_stop;
	img_stop = Mat::zeros(height, width, CV_8UC3);
	float mask[3];
	int sx = 0;
	isStop = 10;

	putText(img_stop, "7M", Point(0, 40), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255));
	line(img_stop, Point(3, 40), Point(600, 40), Scalar(0, 0, 255));
	putText(img_stop, "6M", Point(0, 140), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 0, 255));
	line(img_stop, Point(3, 140), Point(600, 140), Scalar(255, 0, 255));
	putText(img_stop, "5M", Point(0, 240), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 255, 255));
	line(img_stop, Point(3, 240), Point(600, 240), Scalar(0, 255, 255));
	putText(img_stop, "4M", Point(0, 340), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 255, 0));
	line(img_stop, Point(3, 340), Point(600, 340), Scalar(255, 255, 0));
	putText(img_stop, "3M", Point(0, 440), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255));
	line(img_stop, Point(3, 440), Point(600, 440), Scalar(0, 0, 255));

	uint *image = (uint *)img.data;
	uchar *score_data = (uchar *)img_maskfilter.data;
	int mask_w = _mask_w, mask_h = _mask_h;

	int sy = 0;

	int roi_w = 100;
	int histo = 0;
	for (int y = 20; y < height - 17; y++)
	{
		histo = 0;
		for (int x = int(width / 2) - roi_w; x <= int(width / 2) + roi_w; x++)
		{
			for (int i = 0; i < 3; i++)
			{
				sy = y + (2 * mask_h + 1) * (i - 1);
				int dx, cx, bx, ax;
				int dy, cy, by, ay;
				dy = sy + mask_h;
				dx = x + mask_w;
				cy = sy - mask_h - 1;
				cx = x + mask_w;
				by = sy + mask_h;
				bx = x - mask_w - 1;
				ay = sy - mask_h - 1;
				ax = x - mask_w - 1;
				mask[i] = image[(dy)*width + dx] - image[(cy)*width + cx] - image[(by)*width + bx] + image[(ay)*width + ax];
			}

			float sum = ((mask[1] - mask[0]) + (mask[1] - mask[2])) / 2;
			if (sum > 10000)
			{
				score_data[width * y + x] = 255;
				histo++;
			}
		}

		if (histo > thresh)
		{
			line(img_stop, Point(int(width / 2) - roi_w, y), Point(int(width / 2) + roi_w, y), Scalar(255, 0, 0), 30);
			if (y < 140)
			{
				cout << "stop line distance : 7M\n"
					 << endl;
				isStop = 7;
			}
			else if (y < 240)
			{
				cout << "stop line distance : 6M\n"
					 << endl;
				isStop = 6;
			}
			else if (y < 340)
			{
				cout << "stop line distance : 5M\n"
					 << endl;
				isStop = 5;
			}
			else if (y < 440)
			{
				cout << "stop line distance : 4M\n"
					 << endl;
				isStop = 4;
			}
			else if (y < 470)
			{
				cout << "stop line!!!!\n"
					 << endl;
				isStop = 3;
			}
			break;
		}
	}
	// imshow("img_stop",img_stop);
	return img_stop;
}
#include <opencv2/opencv.hpp>
#include <iostream>
#include <stdio.h>
#include <ros/ros.h>
#include "std_msgs/Int32.h"
#include <std_msgs/Float64.h>
#include <sstream>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

using namespace cv;
using namespace std;

Mat bird_eyes_view(Mat img);
Mat bird_eyes_view_inverse(Mat img);
Mat warp_matrix_inv;
Mat warp_matrix;

Mat mask_filter(Mat img, int w, int h, int thresh);
vector<Point> sliding_window(Mat img);
vector<double> polyFit(vector<Point> px, int i, int degree);
void on_mouse(int event, int x, int y, int flags, void *userdata);
Point ptOld1;
int isStop = 100;
void mouse_callback(int event, int x, int y, int flags, void *param);
Mat img_color;
Mat img_hsv;
Mat OutputImage;
int H, S, V;
Mat frame1, frame2, left, left_gray;
bool callback = false;
void imageCallback(const sensor_msgs::ImageConstPtr &msg);
Scalar RGB_mean(Mat img, int X, int width, int Y, int height);
int first_run = 1;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "stopline_publisher");
	// ros::NodeHandle nh;
	ros::NodeHandle nh1;
	ros::Publisher stopline_pub = nh1.advertise<std_msgs::Float64>("stopline", 100); //int형 메시지
	image_transport::ImageTransport it(nh1);
	image_transport::Publisher image_raw_pub = it.advertise("camera/stopline/image_raw", 100); //카메라에서 이미지 읽어서 송신
	sensor_msgs::ImagePtr msg1;
	ros::Rate loop_rate(50);
	int count = 100;
	VideoCapture cap1(2); //전방 정면캠

	cap1.set(cv::CAP_PROP_FRAME_WIDTH, 640);
	cap1.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
	while (ros::ok())
	{
		waitKey(1);
		cap1 >> frame1;
		msg1 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame1).toImageMsg();
		image_raw_pub.publish(msg1);
		if (!cap1.isOpened())
		{
			cerr << "finish!\n"
				 << endl;
		}
		// imshow("frame1", frame1);
		Mat img_resize;
		cv::resize(frame1, img_resize, cv::Size(640, 480), 0, 0);
		// imshow("img_resize", img_resize);

		img_color = img_resize.clone();

		Mat img_warp, img_warp_clone;
		if (first_run == 1)
		{
			img_warp = bird_eyes_view(img_resize);
			first_run = 0;
		}
		else
		{
			cv::warpPerspective(img_resize, img_warp, warp_matrix, cv::Size());
		}
		img_warp_clone = img_warp.clone();
		// imshow("img_warp", img_warp_clone); // 시점 변환

		Mat img_warp_color_mean(img_warp.size(), img_warp.type());
		rectangle(img_warp_clone, Rect(Point(img_warp.cols / 5, img_warp.rows * 3 / 5), Point(img_warp.cols * 2 / 5, img_warp.rows * 4 / 5)), Scalar(0, 0, 255), 1, 8, 0);
		rectangle(img_warp_clone, Rect(Point(img_warp.cols * 3 / 5, img_warp.rows * 3 / 5), Point(img_warp.cols * 4 / 5, img_warp.rows * 4 / 5)), Scalar(0, 0, 255), 1, 8, 0);
		// imshow("img_warp_color_mean", img_warp_clone); // 색 평균 영역 설정

		Scalar mean_color1 = RGB_mean(img_warp, img_warp.rows * 3 / 5, img_warp.rows / 5, img_warp.cols / 5, img_warp.cols / 5);
		Scalar mean_color2 = RGB_mean(img_warp, img_warp.rows * 3 / 5, img_warp.rows / 5, img_warp.cols * 3 / 5, img_warp.cols * 1 / 5);

		Mat img_binary;
		cv::inRange(img_warp, (mean_color1 + mean_color2) / 2 + cv::Scalar(30, 30, 30), cv::Scalar(255, 255, 255), img_binary);
		// imshow("img_binary", img_binary);

		Mat img_integral;
		cv::integral(img_binary, img_integral);

		Mat img_mask;
		img_mask = mask_filter(img_integral, 5, 8, 100); // 5,5,85   120      check!!!!!!!!!!!!!!!!!!!!@@@@@@@@@@@@@@@@@@@@@@@@@@@@
		// imshow("img_mask", img_mask);   //mask filter 이거 체크!!!!!!!!!!!!!

		Mat warp_inv;
		cv::warpPerspective(img_mask, warp_inv, warp_matrix_inv, cv::Size());

		Mat img_resize1;
		cv::resize(img_color, img_resize1, cv::Size(641, 481), 0, 0);

		Mat final;
		addWeighted(warp_inv, 1, img_resize1, 1, 0, final);

		if (isStop == 12)
		{
			count = 12;
			putText(final, "12M", Point(500, 100), FONT_HERSHEY_SIMPLEX, 3, Scalar(0, 0, 255));
		}

		// else if (isStop == 10)
		// {
		// 	count = 10;
		// 	putText(final, "10M", Point(500, 100), FONT_HERSHEY_SIMPLEX, 3, Scalar(0, 0, 255));
		// }

		// else if (isStop == 9)
		// {
		// 	count = 9;
		// 	putText(final, "9M", Point(500, 100), FONT_HERSHEY_SIMPLEX, 3, Scalar(0, 0, 255));
		// }
		// else if (isStop == 8)
		// {
		// 	count = 8;
		// 	putText(final, "8M", Point(500, 100), FONT_HERSHEY_SIMPLEX, 3, Scalar(0, 0, 255));
		// }
		else if (isStop == 10)
		{
			count = 10;
			putText(final, "10M", Point(500, 100), FONT_HERSHEY_SIMPLEX, 3, Scalar(0, 0, 255));
		}
		else if (isStop == 9)
		{
			count = 9;
			putText(final, "9M", Point(500, 100), FONT_HERSHEY_SIMPLEX, 3, Scalar(0, 0, 255));
		}
		else if (isStop == 8)
		{
			count = 8;
			putText(final, "8M", Point(500, 100), FONT_HERSHEY_SIMPLEX, 3, Scalar(0, 0, 255));
			// isStop=100;
		}
		else if (isStop == 7)
		{
			count = 7;
			putText(final, "7M", Point(500, 100), FONT_HERSHEY_SIMPLEX, 3, Scalar(0, 0, 255));
			// isStop=100;
		}

		else if (isStop == 6)
		{
			count = 6;
			putText(final, "6M", Point(500, 100), FONT_HERSHEY_SIMPLEX, 3, Scalar(0, 0, 255));
			// isStop=100;
		}
		else if (isStop == 5)
		{
			count = 5;
			putText(final, "5M", Point(500, 100), FONT_HERSHEY_SIMPLEX, 3, Scalar(0, 0, 255));
			// isStop=100;
		}
		else if (isStop == 4)
		{
			count = 4;
			putText(final, "4M", Point(500, 100), FONT_HERSHEY_SIMPLEX, 3, Scalar(0, 0, 255));
			isStop = 100;
		}

		else if (isStop == 100)
		{
			count = 100;
			putText(final, "Go", Point(500, 100), FONT_HERSHEY_SIMPLEX, 3, Scalar(0, 0, 255));
		}

		imshow("final", final);
		std_msgs::Float64 msg;

		msg.data = count;
		ROS_INFO("%f", msg.data); // data 메시지를표시한다
		stopline_pub.publish(msg);
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}

Mat bird_eyes_view(Mat img)
{
	int width = img.cols;
	int height = img.rows;

	width = img.cols;
	height = img.rows;
	Point2f warp_src_point[4];
	Point2f warp_dst_point[4];

	//원본의 좌표(좌하단, 우하단, 좌상단, 우상단)
	warp_src_point[0].x = 145;
	warp_src_point[0].y = height;
	warp_src_point[1].x = width - warp_src_point[0].x;
	warp_src_point[1].y = warp_src_point[0].y;
	warp_src_point[2].x = 290;
	warp_src_point[2].y = 300;
	warp_src_point[3].x = width - warp_src_point[2].x;
	warp_src_point[3].y = warp_src_point[2].y;

	//목표이미지의 좌표(좌하단, 우하단, 좌상단, 우상단)
	warp_dst_point[0].x = 150;
	warp_dst_point[0].y = height;
	warp_dst_point[1].x = width - warp_dst_point[0].x;
	warp_dst_point[1].y = height;
	warp_dst_point[2].x = 150;
	warp_dst_point[2].y = 0;
	warp_dst_point[3].x = width - warp_dst_point[2].x;
	warp_dst_point[3].y = 0;

	warp_matrix = cv::getPerspectiveTransform(warp_src_point, warp_dst_point);
	invert(warp_matrix, warp_matrix_inv);

	Mat dst;
	cv::warpPerspective(img, dst, warp_matrix, cv::Size(width, height));
	//imshow("dst", dst); top view

	return dst;
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
	isStop = 100;

	uint *image = (uint *)img.data;
	uchar *score_data = (uchar *)img_maskfilter.data;
	int mask_w = _mask_w, mask_h = _mask_h;

	int sy = 0;

	int roi_w = 100; // 80         check!!!!!!!!!!!!!!! 꼭 체크!!!!!!!!!!@@@@@@@@@@@@@@@@@@@@@@@
	int histo = 0;

	for (int y = 20; y < height - 15; y++)
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
			if (sum > 10000) // 20000    check!!!!!!!@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@2
			{
				score_data[width * y + x] = 255;
				histo++;
			}
		}
		line(img_stop, Point(int(width / 2) + roi_w, 20), Point(int(width / 2) + roi_w, height), Scalar(255, 255, 0), 5);
		line(img_stop, Point(int(width / 2) - roi_w, 20), Point(int(width / 2) - roi_w, height), Scalar(255, 255, 0), 5);

		if (histo > thresh)
		{
			line(img_stop, Point(int(width / 2) - roi_w, y), Point(int(width / 2) + roi_w, y), Scalar(255, 0, 0), 30);
			cout << "histo : " << histo << endl;
			// if (y < 45)
			// {
			// 	cout << "stop line distance : 11M\n"
			// 		 << endl;
			// 	isStop = 11;
			// }
			// else if (y < 90)
			// {
			// 	cout << "stop line distance : 10M\n"
			// 		 << endl;
			// 	isStop = 10;
			// }
			// else if (y < 135)
			// {
			// 	cout << "stop line distance : 9M\n"
			// 		 << endl;
			// 	isStop = 9;
			// }
			// else if (y < 180)
			// {
			// 	cout << "stop line distance : 8M\n"
			// 		 << endl;
			// 	isStop = 8;
			// }
			// else if (y < 225)
			// {
			// 	cout << "stop line distance : 7M\n"
			// 		 << endl;
			// 	isStop = 7;
			// }
			if (y < 270)
			{
				cout << "stop line distance : 12M\n" ///12m
					 << endl;
				isStop = 14;
			}
			else if (y < 290)
			{
				cout << "stop line distance : 10M\n"
					 << endl;
				isStop = 13;
			}
			else if (y < 310)
			{
				cout << "stop line distance : 9M\n"
					 << endl;
				isStop = 12;
			}
			else if (y < 330)
			{
				cout << "stop line distance : 8M\n"
					 << endl;
				isStop = 11;
			}
			else if (y < 350)
			{
				cout << "stop line distance : 7M\n"
					 << endl;
				isStop = 10;
			}
			else if (y < 370)
			{
				cout << "stop line distance : 6M\n"
					 << endl;
				isStop = 9;
			}
			else if (y < 390)
			{
				cout << "stop line distance : 5M\n"
					 << endl;
				isStop = 8;
			}
			else if (y < 410)
			{
				cout << "stop line distance : 7M\n"
					 << endl;
				isStop = 7;
			}
			else if (y < 430)
			{
				cout << "stop line distance : 6M\n"
					 << endl;
				isStop = 6;
			}
			else if (y < 450)
			{
				cout << "stop line distance : 5M\n"
					 << endl;
				isStop = 5;
			}
			else if (y < 470)
			{
				cout << "stop line distance : 4M\n"
					 << endl;
				isStop = 4;
			}
			break;
		}
	}
	// imshow("img_stop", img_stop);
	return img_stop;
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
		V = V - 50;

		if (H < 0)
			H = 0;

		if (S < 0)
			S = 0;

		if (V < 0)
			V = 0;
	}
}

Scalar RGB_mean(Mat img, int X, int width, int Y, int height)
{
	Mat img_roi = img(Rect(Point(Y, X), Point(Y + height, X + width)));
	// imshow("img_roi",img_roi);
	Scalar average = mean(img_roi);
	// std::cout << average << std::endl;
	return average;
}
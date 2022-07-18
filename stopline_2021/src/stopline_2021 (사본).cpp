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

Mat preprocessing(Mat img);
Mat bird_eyes_view(Mat img);
Mat bird_eyes_view_inverse(Mat img);
Mat warp_matrix_inv;
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

int main(int argc, char **argv)
{
	VideoCapture cap1("/home/usera/catkin_ws/src/stopline_2021/src/stopline_kcity1.mp4");
	// VideoCapture cap1(2); //전방 정면캠
	cap1.set(cv::CAP_PROP_FRAME_WIDTH, 640);
	cap1.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
	ros::init(argc, argv, "stopline_publisher");
	ros::NodeHandle nh;
	ros::NodeHandle nh1;
	ros::Publisher stopline_pub = nh.advertise<std_msgs::Float64>("stopline", 100); //int형 메시지
	image_transport::ImageTransport it(nh1);
	image_transport::Publisher image_raw_pub = it.advertise("camera/image_raw", 10);	  //카메라에서 이미지 읽어서 송신
	image_transport::Publisher image_final_pub1 = it.advertise("camera/image_final", 10); // 차선 최종본 송신
	// image_transport::Subscriber sub = it.subscribe("camera/rgb/image_raw", 1, imageCallback);
	sensor_msgs::ImagePtr msg1;
	sensor_msgs::ImagePtr msg2;
	
	ros::Rate loop_rate(50);
	int count = 100;

	Mat frame1, frame2, left, left_gray;
	// if (!view.isOpened())
	// {
	// 	cerr << "finish!\n"
	// 		 << endl;
	// }

	while (ros::ok())
	{
		cap1>>frame1;
		msg1 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame1).toImageMsg();
		image_raw_pub.publish(msg1);
		if (!cap1.isOpened())
		{
			cerr << "finish!\n"
				 << endl;
		}

		double fps = cap1.get(CAP_PROP_FPS);
		cout << "FPS: " << fps << endl;
		int delay = cvRound(1000 / fps);

		// imshow("frame1", frame1);
		frame2 = preprocessing(frame1);
		Mat img_resize1;
		cv::resize(frame1, img_resize1, cv::Size(641, 481), 0, 0);
		//imshow("frame2", frame2);
		Mat final;
		addWeighted(img_resize1, 0.5, frame2, 0.5, 0, final);
		msg2 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", final).toImageMsg();
		image_final_pub1.publish(msg2);

		if (waitKey(delay) == 27)
		{
			break;
			printf("end");
		}

		if (isStop == 11)
		{
			count = 11;
			putText(final, "11M", Point(500, 100), FONT_HERSHEY_SIMPLEX, 3, Scalar(0, 0, 255));
		}

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
		}
		else if (isStop == 7)
		{
			count = 7;
			putText(final, "7M", Point(500, 100), FONT_HERSHEY_SIMPLEX, 3, Scalar(0, 0, 255));
		}
		else if (isStop == 6)
		{
			count = 6;
			putText(final, "6M", Point(500, 100), FONT_HERSHEY_SIMPLEX, 3, Scalar(0, 0, 255));
		}
		else if (isStop == 5)
		{
			count = 5;
			putText(final, "5M", Point(500, 100), FONT_HERSHEY_SIMPLEX, 3, Scalar(0, 0, 255));
		}
		else if (isStop == 4)
		{
			count = 4;
			putText(final, "4M", Point(500, 100), FONT_HERSHEY_SIMPLEX, 3, Scalar(0, 0, 255));
		}

		else if (isStop == 3)
		{
			count = 3;
			putText(final, "3M", Point(500, 100), FONT_HERSHEY_SIMPLEX, 3, Scalar(0, 0, 255));
		}
		else if (isStop == 2)
		{
			count = 2;
			putText(final, "2M", Point(500, 100), FONT_HERSHEY_SIMPLEX, 3, Scalar(0, 0, 255));
			
		}
		// else if (isStop == 1)
		// {
		// 	count = 1;
		// 	putText(final, "1M", Point(500, 100), FONT_HERSHEY_SIMPLEX, 3, Scalar(0, 0, 255));
		// 	isStop = 100;
		// }
		
		else if (isStop == 100)
		{
			count = 100;
			putText(final, "Go", Point(500, 100), FONT_HERSHEY_SIMPLEX, 3, Scalar(0, 0, 255));
		}
		// else if(isStop==3)
		// {
		// 	count = 3;
		// 	putText(final,"STOP!!",Point(380,100),FONT_HERSHEY_SIMPLEX,3,Scalar(0,0,255));
		// }
		imshow("final", final);
		std_msgs::Float64 msg;

		msg.data = count;
		ROS_INFO("%f", msg.data);  // data 메시지를표시한다
		stopline_pub.publish(msg); // 메시지를발행한다
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}

Mat preprocessing(Mat img)
{
	Mat img_resize;
	cv::resize(img, img_resize, cv::Size(640, 480), 0, 0);
	// imshow("img_resize", img_resize);

	// namedWindow("img_resize");
	// setMouseCallback("img_resize", on_mouse);
	// imshow("img_resize", img_resize);
	// waitKey();

	img_color = img_resize.clone();

	// namedWindow("img_color", WINDOW_AUTOSIZE); //window for output mask
	// setMouseCallback("img_color", mouse_callback);
	// imshow("img_color",img_resize);
	// waitKey();
	
	// Mat img_binary;
	// cv::inRange(img_resize, cv::Scalar(120, 120, 120), cv::Scalar(255, 255, 255), img_binary);
	// imshow("img_binary",img_binary);
	Mat HSV_image;
	cvtColor(img_resize, HSV_image, COLOR_BGR2HSV);

	Scalar lower_white = Scalar(0, 0, 220);
	Scalar upper_white = Scalar(255, 50, 255);

	Mat white_image;
	inRange(HSV_image, lower_white, upper_white, white_image);
	imshow("white_image", white_image);

	Mat img_warp;
	img_warp = bird_eyes_view(white_image);
	imshow("warp_img", img_warp);

	Mat img_integral;
	cv::integral(img_warp, img_integral);

	Mat img_mask;
	img_mask = mask_filter(img_integral, 5, 5, 85); // 5,5,85   120      check!!!!!!!!!!!!!!!!!!!!
	imshow("img_mask", img_mask);

	Mat warp_inv;
	warp_inv = bird_eyes_view_inverse(img_mask);
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

	warp_src_point[0].x = 145;
	warp_src_point[0].y = 470;
	warp_src_point[1].x = 480;
	warp_src_point[1].y = 470;
	warp_src_point[2].x = 260;
	warp_src_point[2].y = 270;
	warp_src_point[3].x = 380;
	warp_src_point[3].y = 270;

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

	// putText(img_stop, "11M", Point(0, 40), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255));
	// line(img_stop, Point(3, 40), Point(600, 40), Scalar(0, 0, 255));
	// putText(img_stop, "10M", Point(0, 80), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 0, 255));
	// line(img_stop, Point(3, 80), Point(600, 80), Scalar(255, 0, 255));
	// putText(img_stop, "9M", Point(0, 120), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255));
	// line(img_stop, Point(3, 120), Point(600, 120), Scalar(0, 0, 255));
	// putText(img_stop, "8M", Point(0, 160), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255));
	// line(img_stop, Point(3, 160), Point(600, 160), Scalar(0, 0, 255));
	// putText(img_stop, "7M", Point(0, 200), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255));
	// line(img_stop, Point(3, 200), Point(600, 200), Scalar(0, 0, 255));
	// putText(img_stop, "6M", Point(0, 240), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255));
	// line(img_stop, Point(3, 240), Point(600, 240), Scalar(0, 0, 255));
	// putText(img_stop, "5M", Point(0, 280), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 0, 255));
	// line(img_stop, Point(3, 280), Point(600, 280), Scalar(255, 0, 255));
	// putText(img_stop, "4M", Point(0, 320), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 255, 255));
	// line(img_stop, Point(3, 320), Point(600, 320), Scalar(0, 255, 255));
	// putText(img_stop, "3M", Point(0, 360), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 255, 0));
	// line(img_stop, Point(3, 360), Point(600, 360), Scalar(255, 255, 0));
	// putText(img_stop, "2M", Point(0, 400), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 255, 0));
	// line(img_stop, Point(3, 400), Point(600, 400), Scalar(255, 255, 0));
	// putText(img_stop, "1M", Point(0, 440), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 255, 0));
	// line(img_stop, Point(3, 440), Point(600, 440), Scalar(255, 255, 0));
	

	uint *image = (uint *)img.data;
	uchar *score_data = (uchar *)img_maskfilter.data;
	int mask_w = _mask_w, mask_h = _mask_h;

	int sy = 0;

	int roi_w = 100; // 80         check!!!!!!!!!!!!!!!
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
			if (sum > 10000) // 20000    check!!!!!!!
			{
				score_data[width * y + x] = 255;
				histo++;
			}
		}

		if (histo > thresh)
		{
			line(img_stop, Point(int(width / 2) - roi_w, y), Point(int(width / 2) + roi_w, y), Scalar(255, 0, 0), 30);
			cout<<"histo : "<<histo<<endl;
			if (y < 45)
			{
				cout << "stop line distance : 11M\n"
					 << endl;
				isStop = 11;
			}
			else if (y < 90)
			{
				cout << "stop line distance : 10M\n"
					 << endl;
				isStop = 10;
			}
			else if (y < 135)
			{
				cout << "stop line distance : 9M\n"
					 << endl;
				isStop = 9;
			}
			else if (y < 180)
			{
				cout << "stop line distance : 8M\n"
					 << endl;
				isStop = 8;
			}
			else if (y < 225)
			{
				cout << "stop line distance : 7M\n"
					 << endl;
				isStop = 7;
			}
			else if (y < 270)
			{
				cout << "stop line distance : 6M\n"
					 << endl;
				isStop = 6;
			}
			else if (y < 315)
			{
				cout << "stop line distance : 5M\n"
					 << endl;
				isStop = 5;
			}
			else if (y < 360)
			{
				cout << "stop line distance : 4M\n"
					 << endl;
				isStop = 4;
			}
			// else if (y < 405)
			// {
			// 	cout << "stop line distance : 3M\n"
			// 		 << endl;
			// 	isStop = 3;
			// }
			// else if (y < 450)
			// {
			// 	cout << "stop line distance : 2M\n"
			// 		 << endl;
			// 	isStop = 2;
			// }
			// else if (y < 480)
			// {
			// 	cout << "stop line distance : 1M\n"
			// 		 << endl;
			// 	isStop = 1;
			// }
			break;
		}
	}
	// imshow("img_stop", img_stop);
	return img_stop;
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

	warp_src_point[0].x = 145;
	warp_src_point[0].y = 470;
	warp_src_point[1].x = 480;
	warp_src_point[1].y = 470;
	warp_src_point[2].x = 260;
	warp_src_point[2].y = 270;
	warp_src_point[3].x = 380;
	warp_src_point[3].y = 270;

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
	cv::warpPerspective(img, dst1, warp_matrix_inv, cv::Size());

	return dst1;
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
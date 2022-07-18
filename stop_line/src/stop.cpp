#include <opencv2/opencv.hpp>
#include <iostream>
#include <stdio.h>
#include "ros/ros.h"// ROS 기본헤더파일
#include <sstream>
#include <cv_bridge/cv_bridge.h>
#include "stop_line/stopmsg.h"// MsgTutorial메시지파일헤더(빌드후자동생성됨)
using namespace cv;
using namespace std;

Mat preprocessing(Mat img);
Mat bird_eyes_view(Mat img);
Mat mask_filter(Mat img, int w, int h, int thresh);
vector<Point> sliding_window(Mat img);
vector<double> polyFit(vector<Point> px, int i, int degree);

int isStop = 0;
int main(int argc, char **argv)
{
	VideoCapture cap1(2);
	cap1.set(cv::CAP_PROP_FRAME_WIDTH, 640); //이미지 사이즈 정해줌
	cap1.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
	Mat frame1, left, left_gray;
	ros::init(argc, argv, "stop_line_publisher");
	ros::NodeHandle nh;
	ros::Publisher stop_pub= nh.advertise<stop_line::stopmsg>("stop_msg", 100);
	ros::Rate loop_rate(10);
	stop_line::stopmsg msg;
	
	while (ros::ok())
	{
		//��ĸ���κ��� �� �������� �о��
		cap1 >> frame1;
		imshow("src", frame1);
		frame1 = preprocessing(frame1);
		cv::cvtColor(frame1, frame1, COLOR_GRAY2BGR);
		imshow("result", frame1);
		if(isStop){
			msg.x = "stop line";}
		else
		{
			msg.x = "XXXX";
		}
		
		ROS_INFO("%s", msg.x.c_str()); // data 메시지를표시한다
		stop_pub.publish(msg);// 메시지를발행한다
		ros::spinOnce();
		loop_rate.sleep();// 위에서정한루프주기에따라슬립에들어간다
		if (waitKey(1) == 27) break; //ESCŰ ������ ����
	}
	return 0;
}

Mat preprocessing(Mat img) {
	Mat img_resize;
	cv::resize(img, img_resize, cv::Size(640, 360), 0, 0);
	Mat img_binary;
	cv::inRange(img_resize, cv::Scalar(120, 120, 120), cv::Scalar(255, 255, 255), img_binary);
	Mat img_warp;
	img_warp = bird_eyes_view(img_binary);
	//imshow("warp_img", img_warp);
	Mat img_integral;
	cv::integral(img_warp, img_integral);
	Mat img_mask;
	img_mask = mask_filter(img_integral, 5, 5, 85);
	return img_mask;
}


vector<Point> sliding_window(Mat img)
{
	//position_initialize();
	vector<Point> left_peak(11);
	int left_pos[11];
	int right_pos[11];
	for (int i = 0; i < 11; i++)
	{
		left_pos[i] = 190;
		left_peak[i].x = 0;
		left_peak[i].y = 1;
	}

	bool check = false;

	int roi_width = 250;
	int height = img.rows;
	int width = img.cols;
	int left_roi = width * 0.6;

	//��� ������ ������ roi ����
	Mat image = img.clone();
	Mat left = image(Range(0, height), Range(0, left_roi));

	//find_peak ��ħ(������ find_peak�κ�)
	int max_value = -1;
	int max_index = 0;

	int left_count = 0;
	//���� �����̵� ������
	int left_width = left.cols;
	int no_cnt = 0;

	cv::cvtColor(img, img, COLOR_GRAY2BGR);
	for (int i = 10; i > -1; i--)
	{
		int w1 = left_pos[i] - roi_width / 2;
		int w2 = left_pos[i] + roi_width / 2;
		if (w1 < 0)  w1 = 0;
		else if (w1 > left_width) w1 = left_width;
		if (w2 > left_width) w2 = left_width;
		else if (w2 < 0) w2 = 0;
		//roi 30 x 9
		Mat roi = left(Range((i + 1) * 10 + 1, (i + 2) * 10), Range(w1, w2));
		rectangle(img, Point(w1, (i + 1) * 10 + 1), Point(w2, (i + 2) * 10), Scalar(0, 255, 0), 1);
		//cout << roi.cols << endl;
		for (int x = 0; x < roi.cols; x++)
		{
			// int peaks[31] = { 0, };
			int cnt = 0;
			for (int y = 0; y < roi.rows; y++)
			{
				//����ȭ�� ���󿡼� 0�� �ƴҰ�� cnt�߰�
				if (roi.at<uchar>(y, x) != 0)  cnt++;
			}
			//peaks[x] = cnt;
			if (cnt >= max_value)
			{
				max_value = cnt;
				max_index = x;
			}
		}

		if (max_value > 0)
		{
			roi_width = 30;
			//peak�� ����
			left_peak[left_count].x = max_index + w1;
			left_peak[left_count++].y = (i + 1) * 10 + 5;
			//���� roi�� �߽� ����
			if (i > 0)   left_pos[i - 1] = max_index + w1;
		}
		if (roi_width == 250) {
			no_cnt++;
		}
		if (no_cnt > 0) {
			break;
		}
	}
	
	return left_peak;
}

Mat bird_eyes_view(Mat img) {
	int width = img.cols;
	int height = img.rows;
	img = img(Range(height * 3 / 5, height), Range(0, width));
	width = img.cols;
	height = img.rows;
	Mat warp_matrix;
	Point2f warp_src_point[4];
	Point2f warp_dst_point[4];
	warp_src_point[0].x = 0; warp_src_point[0].y = height;
	warp_src_point[1].x = width; warp_src_point[1].y = height;
	warp_src_point[2].x = 0; warp_src_point[2].y = 0;
	warp_src_point[3].x = width; warp_src_point[3].y = 0;
	warp_dst_point[0].x = width * 2 / 5; warp_dst_point[0].y = height;
	warp_dst_point[1].x = width * 3 / 5; warp_dst_point[1].y = height;
	warp_dst_point[2].x = 0; warp_dst_point[2].y = 0;
	warp_dst_point[3].x = width; warp_dst_point[3].y = 0;
	warp_matrix = cv::getPerspectiveTransform(warp_src_point, warp_dst_point);
	Mat dst;
	cv::warpPerspective(img, dst, warp_matrix, cv::Size(width, height));
	return dst;
}

Mat mask_filter(Mat img, int _mask_w, int _mask_h, int thresh) {
	int height = img.rows;
	int width = img.cols;
	Mat img_maskfilter;
	img_maskfilter = Mat::zeros(height, width, CV_8UC1);
	Mat img_stop;
	img_stop = Mat::zeros(height, width, CV_8UC3);
	float mask[3];
	int sx = 0;
	isStop = 0;
	// imshow("before mask", img);

	uint* image = (uint*)img.data;
	uchar* score_data = (uchar*)img_maskfilter.data;
	int mask_w = _mask_w, mask_h = _mask_h;

	int sy = 0;

	int roi_w = 80;
	int histo = 0;
	//cvtColor(img, img, COLOR_GRAY2BGR);
	for (int y = 20; y < height - 20;  y++) {
		histo = 0;
		for (int x = int(width/2) - roi_w; x <= int(width/2) + roi_w; x++) {
			for (int i = 0; i < 3; i++) {
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
				// printf(" dy : %d, cy : %d, by : %d, ay : %d\n", dy, cy, by, ay);
				// printf(" dx : %d, cx : %d, bx : %d, ax : %d\n", dx, cx, bx, ax);
				mask[i] = image[(dy) * width + dx] - image[(cy) * width + cx] - image[(by) * width + bx] + image[(ay) * width + ax];
			}

			float sum = ((mask[1] - mask[0]) + (mask[1] + mask[2])) / 2;
			// printf("sum : %f\n", sum);
			//cout << "sum : " << sum << endl;
			if (sum > 20000) {
				// cout << "sum :" << sum << endl;
				// circle(img_maskfilter, Point2i(x, y), 5, Scalar(255, 0, 0), 3);
				score_data[width * y + x] = 255;
				histo++;
			}
		}
		//printf("histo : %d\n");
		if (histo > thresh) {
			line(img_stop, Point(int(width / 2) - roi_w, y), Point(int(width / 2) + roi_w, y), (255, 0, 255), 3);
			printf("histo : %d\n", histo);
			printf("stop line!");
			isStop = 1;
			break;
		}
		
	}
	imshow("stop line", img_stop);
	// imshow("after mask", img_maskfilter);
	return img_maskfilter;
}

// � �ٻ� �Լ�
vector<double> polyFit(vector<Point> px, int iter, int deg) {
	//int X[5];
	vector<double> X(2 * deg + 1);
	for (int i = 0; i < 2 * deg + 1; i++)
	{
		X[i] = 0;
		for (int j = 0; j < iter; j++)
			X[i] = X[i] + pow(px[j].x, i);        //consecutive positions of the array will store N,sigma(xi),sigma(xi^2),sigma(xi^3)....sigma(xi^2n)
	}
	//double B[degree+1][degree+2];
	vector<vector<double>> B(deg + 1, vector<double>(deg + 2));
	vector<double> a(deg + 1);            //B is the Normal matrix(augmented) that will store the equations, 'a' is for value of the final coefficients
	for (int i = 0; i <= deg; i++)
		for (int j = 0; j <= deg; j++)
			B[i][j] = X[i + j];            //Build the Normal matrix by storing the corresponding coefficients at the right positions except the last column of the matrix
	vector<double> Y(deg + 1);
	for (int i = 0; i < deg + 1; i++)
	{
		Y[i] = 0;
		for (int j = 0; j < iter; j++)
			Y[i] = Y[i] + pow(px[j].x, i) * px[j].y;        //consecutive positions will store sigma(yi),sigma(xi*yi),sigma(xi^2*yi)...sigma(xi^n*yi)
	}
	for (int i = 0; i <= deg; i++)
		B[i][deg + 1] = Y[i];                //load the values of Y as the last column of B(Normal Matrix but augmented)
	int n = deg + 1;                //n is made n+1 because the Gaussian Elimination part below was for n equations, but here n is the degree of polynomial and for n degree we get n+1 equations
	for (int i = 0; i < n; i++)            //print the Normal-augmented matrix
	{
		for (int j = 0; j <= n; j++) {

		}
		/*cout << B[i][j] << setw(16);
	cout << "\n";*/
	}
	for (int i = 0; i < n; i++)                    //From now Gaussian Elimination starts(can be ignored) to solve the set of linear equations (Pivotisation)
		for (int k = i + 1; k < n; k++)
			if (B[i][i] < B[k][i])
				for (int j = 0; j <= n; j++)
				{
					double temp = B[i][j];
					B[i][j] = B[k][j];
					B[k][j] = temp;
				}

	for (int i = 0; i < n - 1; i++)            //loop to perform the gauss elimination
		for (int k = i + 1; k < n; k++)
		{
			double t = B[k][i] / B[i][i];
			for (int j = 0; j <= n; j++)
				B[k][j] = B[k][j] - t * B[i][j];    //make the elements below the pivot elements equal to zero or elimnate the variables
		}
	for (int i = n - 1; i >= 0; i--)                //back-substitution
	{                        //x is an array whose values correspond to the values of x,y,z..
		a[i] = B[i][n];                //make the variable to be calculated equal to the rhs of the last equation
		for (int j = 0; j < n; j++)
			if (j != i)            //then subtract all the lhs values except the coefficient of the variable whose value                                   is being calculated
				a[i] = a[i] - B[i][j] * a[j];
		a[i] = a[i] / B[i][i];            //now finally divide the rhs by the coefficient of the variable to be calculated
	}

	return a;
}
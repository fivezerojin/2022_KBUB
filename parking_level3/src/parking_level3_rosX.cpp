#include "opencv2/opencv.hpp"
#include <iostream>

using namespace std;
using namespace cv::ml;
using namespace cv;

Mat preprocessing(Mat img);
Mat bird_eyes_view(Mat img);
Mat warp_matrix_inv;
void on_mouse(int event, int x, int y, int flags, void*);
Point ptOld1;
Mat bird_eyes_view_inverse(Mat img);
int H, S, V;
Mat img_color;
Mat img_hsv;
Mat OutputImage;
int isStop = 0;
void mouse_callback(int event, int x, int y, int flags, void *param);
int a1 = 0, a2 = 0, a3 = 0, a4 = 0, a5 = 0;
int x11 = 0, x22 = 0, x33 = 0, x44=0, y11 = 0, y22 = 0, y33 = 0, y44=0;
const double PI = 3.1415926535897932384626433832795028841971693993751058209;


int main()
{
	VideoCapture cap1("/home/usera/catkin_ws/src/parking/src/park2.mp4");
	// VideoCapture cap1(2);
	Mat frame1,frame2;
	
	while (cap1.isOpened())
	{
		cap1 >> frame1;
		
		if (!cap1.isOpened())
		{
			cerr << "video load fail!!\n"
				<< endl;
		}
		
		
		double fps = cap1.get(CAP_PROP_FPS);
		cout << "FPS: " << fps << endl;
		int delay = cvRound(1000 / fps);
		
		frame2 = preprocessing(frame1);
		//imshow("frame2", frame2);
		/*Mat img_resize1;
		cv::resize(frame1, img_resize1, cv::Size(640, 480), 0, 0);
		// cout << frame2.type() << img_resize1.type() << endl;
		// cout << frame2.size() << img_resize1.size() << endl;
		// imshow("frame2", frame2);
		Mat final1;
		addWeighted(img_resize1, 0.5, frame2, 0.5, 0, final1);
		imshow("final1", final1);*/
		if (waitKey(delay) == 27) 
			break;
	}
	return 0;
}

Mat preprocessing(Mat img)
{
	Mat img_resize;
	cv::resize(img, img_resize, cv::Size(640, 480), 0, 0);
	//imshow("img_resize", img_resize);

	//namedWindow("white_image@@@");
	//setMouseCallback("white_image@@@", on_mouse);
	//imshow("white_image@@@", img_resize);
	//waitKey();
	
	img_color = img_resize.clone();

	namedWindow("img_color", WINDOW_AUTOSIZE); //window for output mask
	setMouseCallback("img_color", mouse_callback);
	imshow("img_color", img_resize);
	waitKey();
	
	Mat HSV_image;
	cvtColor(img_resize, HSV_image, COLOR_BGR2HSV);

	Scalar lower_white = Scalar(H, S, V);
	Scalar upper_white = Scalar(255, 255, 255);

	Mat white_image;
	inRange(HSV_image, lower_white, upper_white, white_image);
	//imshow("white_image", white_image);

	Mat img_warp;
	img_warp = bird_eyes_view(white_image);
	//imshow("warp_img", img_warp);	

	Mat dx, dy;
	Sobel(img_warp, dx, CV_32FC1, 1, 0);
	Sobel(img_warp, dy, CV_32FC1, 0, 1);

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
		//line(dst,Point(l[0],l[1]),Point(l[2],l[3]),Scalar(0,0,255),2,LINE_AA);
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

		if (theta < 89 && theta>88)
		{
			cout << "stop!!!!!!!!!!!!!!!!" << theta << endl;
			isStop = 1;
		}

		if (isStop == 1)
		{
			if (theta > 50 && theta < 88)
			{
				line(dst, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 0, 255), 2, LINE_AA);
				//cout << "Point(l[0], l[1]): " << Point(l[0], l[1]) << " Point(l[2], l[3]): " << Point(l[2], l[3]) << endl;
				line(dst4, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255, 255, 255), 15, LINE_AA);
				//cout << "theta_original: " << theta << endl;
			}
		}
		else if (isStop == 0)
		{
			cout << "nothing" << endl;
		}
	}

	imshow("dst", dst);
	//imshow("dst4", dst4);

	int num = (int)(dst4.total()*0.1);
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
		int* p = stats4.ptr<int>(i);
		//rectangle(dst_4, Rect(p[0], p[1], p[2], p[3]), Scalar(0, 0, 255), 2);
		float theta = (atan((float)p[3] / p[2]) * 180) / PI;

		if (p[4] > 5000)
		{
			rectangle(dst_4, Rect(p[0], p[1], p[2], p[3]), Scalar(0, 0, 255), 2);
			//cout << "stats1 : " << p[4] << endl;
			//cout << "p[2]: " << p[2] << " p[3]: " << p[3] << endl;
			cout << "theta_live: " << theta << endl;

			if (theta < 50)
			{
				cout << "convert to front cam!!!!!!!!!!!!" << endl;
			}
		}
	}
	imshow("dst_4", dst_4);
	return img_resize;
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

	
	/*warp_src_point[0].x = 10;
	warp_src_point[0].y = 415;
	warp_src_point[1].x = 590;
	warp_src_point[1].y = 415;
	warp_src_point[2].x = 290;
	warp_src_point[2].y = 260;
	warp_src_point[3].x = 370;
	warp_src_point[3].y = 260;
	warp_src_point[0].x = 40; //
	warp_src_point[0].y = 120;
	warp_src_point[1].x = 520;
	warp_src_point[1].y = 200;
	warp_src_point[2].x = 215;
	warp_src_point[2].y = 5;
	warp_src_point[3].x = 490;
	warp_src_point[3].y = 20;*/

	//test6
	// warp_src_point[0].x = 20;             
	// warp_src_point[0].y = 250;
	// warp_src_point[1].x = 620;
	// warp_src_point[1].y = 250;
	// warp_src_point[2].x = 20;
	// warp_src_point[2].y = 50;
	// warp_src_point[3].x = 620;
	// warp_src_point[3].y = 50; 

	//test4_carX
	/*warp_src_point[0].x = 20;
	warp_src_point[0].y = 350;
	warp_src_point[1].x = 640;
	warp_src_point[1].y = 350;
	warp_src_point[2].x = 20;
	warp_src_point[2].y = 100;
	warp_src_point[3].x = 640;
	warp_src_point[3].y = 100;
	*/
	warp_src_point[0].x = 15;
	warp_src_point[0].y = 350;
	warp_src_point[1].x = 470;
	warp_src_point[1].y = 350;
	warp_src_point[2].x = 15;
	warp_src_point[2].y = 90;
	warp_src_point[3].x = 470;
	warp_src_point[3].y = 90;
	
	/*warp_dst_point[0].x = 220;
	warp_dst_point[0].y = height; //???
	warp_dst_point[1].x = width - 220;
	warp_dst_point[1].y = height;
	warp_dst_point[2].x = 220;
	warp_dst_point[2].y = 0;
	warp_dst_point[3].x = width - 220;
	warp_dst_point[3].y = 0;
*/
// warp_dst_point[0].x = 100;
// warp_dst_point[0].y = height; 
// warp_dst_point[1].x = width-300;
// warp_dst_point[1].y = height;
// warp_dst_point[2].x = 100;
// warp_dst_point[2].y = 0;
// warp_dst_point[3].x = width-300;
// warp_dst_point[3].y = 0;
	//test4_carX
	/*warp_dst_point[0].x = 150;
	warp_dst_point[0].y = height * 0.8;
	warp_dst_point[1].x = width - 150;
	warp_dst_point[1].y = height * 0.8;
	warp_dst_point[2].x = 150;
	warp_dst_point[2].y = 0;
	warp_dst_point[3].x = width - 150;
	warp_dst_point[3].y = 0;
	*/
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


void on_mouse(int event, int x, int y, int flags, void*)
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

	//�̹����� �Ϻ� ������ ������(��� 0.6��������,�� 0.2��������)
	//img = img(Range(height*0.58, height), Range(width*0.2, width*0.8));
	width = img.cols;
	height = img.rows;
	Mat warp_matrix;
	Point2f warp_src_point[4];
	Point2f warp_dst_point[4];

	//������ ��ǥ(���ϴ�, ���ϴ�, �»��, ����)
	/*warp_src_point[0].x = 40;
	warp_src_point[0].y = 140;
	warp_src_point[1].x = 525;
	warp_src_point[1].y = 215;
	warp_src_point[2].x = 200;
	warp_src_point[2].y = 0;
	warp_src_point[3].x = 500;
	warp_src_point[3].y = 10;
	*/
	warp_src_point[0].x = 15;
	warp_src_point[0].y = 350;
	warp_src_point[1].x = 470;
	warp_src_point[1].y = 350;
	warp_src_point[2].x = 15;
	warp_src_point[2].y = 90;
	warp_src_point[3].x = 470;
	warp_src_point[3].y = 90;
	//��ǥ�̹����� ��ǥ(���ϴ�, ���ϴ�, �»��, ����)
	/*warp_dst_point[0].x = 100;
	warp_dst_point[0].y = height; //???
	warp_dst_point[1].x = width - 100;
	warp_dst_point[1].y = height;
	warp_dst_point[2].x = 100;
	warp_dst_point[2].y = 0;
	warp_dst_point[3].x = width - 100;
	warp_dst_point[3].y = 0;
	*/
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
	cv::warpPerspective(img, dst1, warp_matrix_inv, cv::Size()); //������̺� ����ȯ
	//imshow("dst1", dst1);

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
		cout << "V = " << V << "\n" << endl;

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
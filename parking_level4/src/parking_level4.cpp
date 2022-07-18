#include "opencv2/opencv.hpp"
#include <iostream>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "std_msgs/Float64.h"


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
Mat img_color,img_resize2;
Mat img_hsv;
Mat OutputImage;
float parking2 =0;
int isStop= 0;
int AngleStop=0;
void mouse_callback(int event, int x, int y, int flags, void *param);
int a1 = 0, a2 = 0, a3 = 0, a4 = 0, a5 = 0;
int x11 = 0, x22 = 0, x33 = 0, x44=0, y11 = 0, y22 = 0, y33 = 0, y44=0;
const double PI = 3.1415926535897932384626433832795028841971693993751058209;
Mat mask_filter(Mat img, int w, int h, int thresh);


void parking_cb1(const std_msgs::Float64::ConstPtr& msgs)
{
	parking2 = msgs->data;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "parking_level4");
	ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("camera/image_raw", 10);
    image_transport::Publisher pub1 = it.advertise("camera/image_final", 10);
	ros::Publisher parking_level4_pub = nh.advertise<std_msgs::Float64>("ParkingAngle_FRONT", 10);
	ros::Publisher parking_level4_pub1 = nh.advertise<std_msgs::Float64>("Plus_or_Minus", 10);
	ros::Publisher stopline_pub=nh.advertise<std_msgs::Float64>("stopline_parking", 100);
	ros::Subscriber Parking_level3_sub = nh.subscribe("ParkingAngle_RIGHT", 10, parking_cb1);
	std_msgs::Float64 ParkingAngle_FRONT_msg;
	std_msgs::Float64 Plus_or_Minus_msg;
	std_msgs::Float64 stopline_msg;

    ros::Rate loop_rate(50);
	int count = 10;
    sensor_msgs::ImagePtr msg;
    sensor_msgs::ImagePtr msg1;
	
	// VideoCapture cap1("/home/usera/catkin_ws/src/parking/src/parking_kcity_front2.mp4");
	VideoCapture cap1(4);

	if (!cap1.isOpened())
	{
		cerr << "video load fail!!\n"
				<< endl;
	}

	double fps = cap1.get(CAP_PROP_FPS);
	cout << "FPS: " << fps << endl;
	int delay = cvRound(1000 / fps);

	Mat frame1,frame2;
    
	while (ros::ok())
	{
		//if(parking2 < 52)
		//{
			//cout<<"parking2"<<parking2<<endl;
			cap1 >> frame1;
			// imshow("frame1",frame1);
			msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame1).toImageMsg();
			pub.publish(msg);

			Mat img_resize;
			cv::resize(frame1, img_resize, cv::Size(640, 480), 0, 0);
			imshow("img_resize", img_resize);

			img_color = img_resize.clone();
			img_resize2 = img_resize.clone();

			// namedWindow("img_color", WINDOW_AUTOSIZE); //window for output mask
			// setMouseCallback("img_color", mouse_callback);
			// imshow("img_color", img_resize);
			// waitKey();
			
			Mat HSV_image;
			cvtColor(img_resize, HSV_image, COLOR_BGR2HSV);

			// Scalar lower_white = Scalar(H, S, V);
			// Scalar lower_white = Scalar(0, 0, 220);
			// Scalar upper_white = Scalar(255, 255, 255);
			
			Scalar lower_white = Scalar(0, 0, 170);     
			Scalar upper_white = Scalar(255, 100, 255);

			Mat white_image;
			inRange(HSV_image, lower_white, upper_white, white_image);
			imshow("white_image", white_image);

			Mat img_warp;
			img_warp = bird_eyes_view(white_image);
			imshow("warp_img", img_warp);	

			Mat img_warp1=img_warp.clone();

			Mat dx, dy;
			Sobel(img_warp1, dx, CV_32FC1, 1, 0);
			Sobel(img_warp1, dy, CV_32FC1, 0, 1);

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

				
				if ((theta > -20 && theta < 20) && (d > 70) )  
				{
					line(dst, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 0, 255), 2, LINE_AA);
					// cout << "theta: " << theta << endl;
					//cout << "Point(l[0], l[1]): " << Point(l[0], l[1]) << " Point(l[2], l[3]): " << Point(l[2], l[3]) << endl;
					line(dst4, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255, 255, 255), 15, LINE_AA);
					cout << "theta_original: " << theta << endl;
					Plus_or_Minus_msg.data = theta;
					parking_level4_pub1.publish(Plus_or_Minus_msg);
					// ROS_INFO("%f",Plus_or_Minus_msg.data);
				}
				
			}
			imshow("dst", dst);
			// imshow("dst4", dst4);

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
				// rectangle(dst_4, Rect(p[0], p[1], p[2], p[3]), Scalar(0, 0, 255), 2);
				// cout << "stats1 : " << p[4] << endl;
				// float theta = (atan((float)p[3] / p[2]) * 180) / PI;
				float x1=p[0],y1=p[1],x2=p[0]+p[2],y2=p[1]+p[3];
				float k1=((y2-y1)/(x2-x1));
				float radian1=atan(k1);
				float theta_live=(radian1*180)/PI;

				// cout<<"theta: "<<theta<<endl;

				if (p[4] > 5000 && p[4]<10000)   //8000
				{
					rectangle(dst_4, Rect(p[0], p[1], p[2], p[3]), Scalar(0, 0, 255), 2);
					// cout << "stats1 : " << p[4] << endl;
					//cout << "p[2]: " << p[2] << " p[3]: " << p[3] << endl;
					// cout << "theta_live: " << theta << endl;
					ParkingAngle_FRONT_msg.data = theta_live;
					parking_level4_pub.publish(ParkingAngle_FRONT_msg);
					ROS_INFO("%f",ParkingAngle_FRONT_msg.data);
					if (theta_live < 9)
					{
						cout << "stop change angle!" << endl;

					}
				}
			}
			imshow("dst_4", dst_4);

			Mat warp_inv;
			warp_inv = bird_eyes_view_inverse(dst_4);
			imshow("warp_inv", warp_inv);

			Mat img_integral;
			cv::integral(img_warp, img_integral);

			Mat img_mask;
			img_mask = mask_filter(img_integral, 5,5, 95);
			//imshow("img_mask", img_mask);

			Mat warp_inv11;
			warp_inv11 = bird_eyes_view_inverse(img_mask);
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
			addWeighted(img_resize2, 0.5, final4, 0.5, 0, final5);

			if(isStop==5)
			{
				count = 5;
				putText(final5,"5M",Point(500,100),FONT_HERSHEY_SIMPLEX,3,Scalar(0,0,255));
			}

			else if(isStop==4)
			{
				count = 4;
				putText(final5,"4M",Point(500,100),FONT_HERSHEY_SIMPLEX,3,Scalar(0,0,255));
			}

			else if(isStop==3)
			{
				count = 3;
				putText(final5,"3M",Point(500,100),FONT_HERSHEY_SIMPLEX,3,Scalar(0,0,255));   //5.5
			}

			else if(isStop==2)
			{
				count = 2;
				putText(final5,"2M",Point(500,100),FONT_HERSHEY_SIMPLEX,3,Scalar(0,0,255));
			}

			else if(isStop==1)
			{
				count = 1;
				putText(final5,"STOP!!",Point(380,100),FONT_HERSHEY_SIMPLEX,3,Scalar(0,0,255));
			}

			stopline_msg.data = count;
			ROS_INFO("%f",stopline_msg.data); // data 메시지를표시한다
			stopline_pub.publish(stopline_msg);// 메시지를발행한다

			
			imshow("final5", final5);

			msg1 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", final5).toImageMsg();
			pub1.publish(msg1);
			
			ros::spinOnce();
			loop_rate.sleep();
			if (waitKey(delay) == 27) 
				break;
		//}
	}
	return 0;
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
	
	putText(img_stop, "7M",Point(0, 40), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255));
	line(img_stop, Point(3, 40), Point(600, 40), Scalar(0, 0, 255));
	putText(img_stop, "6M", Point(0, 140), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 0, 255));
	line(img_stop, Point(3, 140), Point(600, 140), Scalar(255, 0, 255));
	putText(img_stop, "5M", Point(0, 240), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 255, 255));
	line(img_stop, Point(3, 240), Point(600, 240), Scalar(0, 255, 255));
	putText(img_stop, "4M", Point(0, 340), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 255, 0));
	line(img_stop, Point(3, 340), Point(600, 340), Scalar(255,255,0));
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
			if ( y < 140 )
				{
					cout << "stop line distance : 7M\n" << endl;
					isStop = 5;
				}
			else if (y < 240)
				{
					cout << "stop line distance : 6M\n" << endl;
					isStop = 4;
				}
			else if (y < 340)
				{
					cout << "stop line distance : 5M\n" << endl;
					isStop = 3;
				}
			else if (y < 440)
				{
					cout << "stop line distance : 4M\n" << endl;
					isStop = 2;
				}
			else if (y < 470)
				{
					cout << "stop line!!!!\n" << endl;
					isStop = 1;
				}
	            break;
		}         
    }
    // imshow("img_stop",img_stop);
    return img_stop;
}

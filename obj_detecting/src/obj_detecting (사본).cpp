#include <ros/ros.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/ObjectCount.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/opencv.hpp"
#include "std_msgs/Bool.h"
#include "std_msgs/Int32MultiArray.h"
 
using namespace cv;
using namespace std;

void BoundingBox_callback(darknet_ros_msgs::BoundingBoxes boundingBox);
void ObjectCount_callback(darknet_ros_msgs::ObjectCount ObjectCount);

int obj_count;

std_msgs::Int32MultiArray sign_msg;
ros::Subscriber detect_obj;
ros::Subscriber obj_num;
ros::Publisher final_pub;
ros::Publisher Stop_final_pub;
ros::Publisher traffic_sign_pub;
float Max_Area = 0;
int mission_A = 0;
int mission_B = 0;
int count_mission = 0;
bool mission_stop = false;
int count_A1 = 0;
int count_A2 = 0;
int count_A3 = 0;
int sign[3]={0,0,0};
int red = 0;
int green = 0;
int greenleft = 0;


int main(int argc, char **argv)
{
    ros::init(argc, argv, "obj_detecting");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("camera/rgb/image_raw", 10);
    detect_obj = nh.subscribe<darknet_ros_msgs::BoundingBoxes>("/darknet_ros/bounding_boxes", 1000, BoundingBox_callback);
    obj_num = nh.subscribe<darknet_ros_msgs::ObjectCount>("/darknet_ros/found_object", 1000, ObjectCount_callback);
    traffic_sign_pub = nh.advertise<std_msgs::Int32MultiArray>("traffic_sign", 100);
    Stop_final_pub = nh.advertise<std_msgs::Bool>("/Mission_Stop", 1000);

    ros::Rate loop_rate(100);
    sensor_msgs::ImagePtr msg;

    VideoCapture cap(2);
    // VideoCapture cap("/home/usera/catkin_ws/src/obj_detecting/src/test.mp4");

    if (!cap.isOpened())
    {
        cerr << "video load fail!!\n"
             << endl;
    }

    Mat src;

    while (ros::ok())
    {
        cap >> src;
        imshow("src", src);
        msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", src).toImageMsg();
        pub.publish(msg);
        // Mat src_resize;
        // resize(src, src_resize, Size(1280, 720));

        if (src.empty())
        {
            printf("finish!!\n");
            break;
        }

        if (waitKey(10) == 27)
        {
            break;
            printf("강제 종료\n");
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

void ObjectCount_callback(darknet_ros_msgs::ObjectCount ObjectCount)
{
    printf("발견한 객체 수(YOLO) = %d\n", ObjectCount.count);
    obj_count = ObjectCount.count;
}

void BoundingBox_callback(darknet_ros_msgs::BoundingBoxes boundingBox)
{
    
    sign_msg.data = {red,green,greenleft};

    if (obj_count != 0)
    {
        std::string Class_name;
        Max_Area = 0;
        red = 0, green = 0, greenleft = 0;
        std_msgs::Bool mission_stop_msg;
        for (int i = 0; i < obj_count; i++)
        {
            Class_name = boundingBox.bounding_boxes[i].Class;

            if (Class_name == "A1")
            {
                count_A1 += 1;
            }
            else if (Class_name == "A2")
            {
                count_A2 += 1;
            }
            else if (Class_name == "A3")
            {
                count_A3 += 1;
            }
            else if (Class_name == "green")
            {
                green = 1;
            }
            else if (Class_name == "red")
            {
                red = 1;
            }
            else if (Class_name == "greenleft")
            {
                greenleft = 1;
            }
            else if (Class_name == "B1" || Class_name == "B2" || Class_name == "B3")
            {
                float xmin = boundingBox.bounding_boxes[i].xmin;
                float ymin = boundingBox.bounding_boxes[i].ymin;
                float xmax = boundingBox.bounding_boxes[i].xmax; red=0;
                greenleft=0;
                float ymax = boundingBox.bounding_boxes[i].ymax;
                float Area = ((xmax - xmin) * (ymax - ymin));
                cout << "Class_name [ " << i << "] : " << Class_name << " Area: " << Area << endl;

                if (Max_Area < Area)
                {
                    Max_Area = Area;

                    if (Class_name == "B1")
                    {
                        mission_B = 1;
                    }

                    else if (Class_name == "B2")
                    {
                        mission_B = 2;
                    }

                    else if (Class_name == "B3")
                    {
                        mission_B = 3;
                    }
                }
            }
        }

        mission_A = 1;
        if (count_A1 < count_A2)
        {
            mission_A = 2;
            if (count_A2 < count_A3)
                mission_A = 3;
        }
        else if (count_A1 < count_A3)
            mission_A = 3;

        cout << "mission_A: " << mission_A << endl;
        cout << "mission_B: " << mission_B << endl;

        if (mission_A == mission_B)
        {
            mission_stop = true;
        }
        else if (mission_A != mission_B)
        {
            mission_stop = false;
        }
        cout << "mission_stop: " << mission_stop << endl;
        mission_stop_msg.data = mission_stop;
        Stop_final_pub.publish(mission_stop_msg);

        cout<<"red : "<<sign_msg.data[0] << "\t\tgreen : "<<sign_msg.data[1]<<"\tgreenleft : "<<sign_msg.data[2]<<endl;
        traffic_sign_pub.publish(sign_msg);
    }
    
}

#pragma once
#include <iostream>
#include <vector>
#include "Position.cpp"
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/PointCloud.h>

#include <sensor_msgs/PointCloud2.h>
#include "velodyne_filter/PointArray.h"

using namespace std;

struct LidarInput
{
	public:
		velodyne_filter::PointArray Point;
		sensor_msgs::PointCloud cluster;
		bool isDynamicObject = false; 
		bool isBigObject = false;
		geometry_msgs::Point coord_point;


};

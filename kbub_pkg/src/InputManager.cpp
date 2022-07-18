#pragma once
#include "GCS.cpp"
#include "CameraInput.cpp"
#include "LidarInput.cpp"
#include "LocalInput.cpp"
#include "StateArbiter.cpp"
#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include "velodyne_filter/PointArray.h"
#include <cmath>
#include <utility>

using namespace std;

#define X first
#define Y second
using ld = long double;
using Point = pair<ld, ld>;

// ???? ?????? ?????
struct ParkingLot
{
	float P_first_x;
	float P_first_y;

	float P_second_x;
	float P_second_y;

	float P_third_x;
	float P_third_y;

	float P_fourth_x;
	float P_fourth_y;
};


// ????(????, LiDAR, GPS) ??��? ???? ?? ???
class InputManager
{
	
	// // Camera
	CameraInput cameraInput;

	// LiDAR
	LidarInput lidarInput;

	//Local
	GCS gcs;

	ParkingLot parkInput;



	// // GPS
	// GCS gcs;  // GPS?? ???? ???
	// Position position;  // ???? XY ???

	InputManager()
	{
		lidarInput.isDynamicObject = false;
		lidarInput.isBigObject = false;
	}


public:

	// Singleton
	static InputManager* GetInstance()
	{
		static InputManager* instance = new InputManager();
		return instance;
	}

	//==================== Vision =====================

	void setLaneDistance(double laneDistance)
	{
		cameraInput.laneDistance = laneDistance;
	}
	double getLaneDistance()
	{
		return cameraInput.laneDistance;
	}

	void setStopDistance(double stopDistance)
	{
		cameraInput.stopDistance = stopDistance;
	}
	int getStopDistance()
	{
		return cameraInput.stopDistance;
	}

	void setParkingAngle(double parkingAngle)
	{
		cameraInput.parkingAngle = parkingAngle;
	}
	double getParkingAngle()
	{
		cameraInput.parkingAngle;
	}

	void setVisionClass(int i, bool check)
	{
		cameraInput.label[i] = check;
	}
	bool getVisionClass(int i)
	{
		return cameraInput.label[i];
	}
	void printVisionClass()
	{
		printf("[");
		for(int i = 0 ; i< 11; i++)
		{
			printf("%d, ", cameraInput.label[i]);
		}
		printf("%d]\n", cameraInput.label[11]);
	}

	//====================lidar=====================
	void setDynamicObject(const bool& isStop){
		lidarInput.isDynamicObject = isStop;
	}
	bool getDynamicObject(){
		return lidarInput.isDynamicObject;
	}
	bool getBigObject()
	{
		return lidarInput.isBigObject;
	}
	void setBigObject(bool bigOb)
	{
		lidarInput.isBigObject = bigOb;
	}
	void setNarrowCoord(geometry_msgs::Point msg)
	{
		lidarInput.coord_point = msg;
	}
	geometry_msgs::Point getNarrowCoord()
	{
		return lidarInput.coord_point;
	}

	void setCluster(geometry_msgs::Point *cluster_center)
	{
		//pass
	}
	sensor_msgs::PointCloud getCluster(){
		return lidarInput.cluster;
	}

	bool hasDetectedObstacle()
	{
		return lidarInput.isDynamicObject;
	}

	void setGcs(double Latitude, double Longitude)
	{
		gcs.Latitude = Latitude;
		gcs.Longitude = Longitude;
	}
	double getGcs_x()
	{
		return gcs.Latitude;
	}
	double getGcs_y()
	{
		return gcs.Longitude;
	}

	void printData()
	{
		printf("Position : %lf, %lf\n", getGcs_x(), getGcs_y());
		printf("hasDetectedObstacle : %s\n", hasDetectedObstacle() ? "True" : "False");

		printf("VisionClass : ");
		printVisionClass();
	}

};

#pragma once
#include <string>
#include "Position.cpp"
using namespace std;
/*
* Sign
* 0 : Nothing
*
* 1 : GreenLight
* 2 : RedLight
* 3 : GreenLeft
*
* 4 : LeftSign
* 5 : RightSign
* 6 : StaticObstacle
* 7 : UnexpectedObstacle
* 8 : CrossWalkSign
* 9 : ParkingSign
* 10 : BusSign
* 11 : SchoolSign
* 12 : SpeedBump
*/
struct CameraInput
{
public:
	double laneDistance;
	double stopDistance;
	bool label[12] = {0,0,0,0,0,0,0,0,0,0,0,0};
	double parkingAngle;
	CameraInput(){
		for(int i = 0; i < 12; i++)
		{
			label[i] = 0;
		}
		laneDistance = 0.0;
		stopDistance = 0.0;
		parkingAngle = 0.0;
	}
};
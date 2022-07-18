#pragma once
#include <iostream>
#include <cmath>
#include <string>

using namespace std;

// 위도/경도 좌표 Geographic Coordinate System
struct GCS
{
public:

	double Latitude;  // 위도
	double Longitude;  // 경도

	GCS()
	{
		Latitude = 0.f;
		Longitude = 0.f;
	}
	GCS(double _latitude, double _longitude)
		: Latitude(_latitude), Longitude(_longitude)
	{
	}

	// UNDONE: GCS 거리 계산 정확도 개선
	static double GetDistance(GCS& c1, GCS& c2)
	{
		return sqrt(pow(c1.Latitude - c2.Latitude, 2) + pow(c1.Longitude - c2.Longitude, 2));
	}
	double GetDistance(GCS& other) const
	{
		return sqrt(pow(Latitude - other.Latitude, 2) + pow(Longitude - other.Longitude, 2));
	}
};

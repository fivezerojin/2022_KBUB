#pragma once
#include <iostream>
#include <cmath>
#include <string>

using namespace std;

// ����/�浵 ��ǥ Geographic Coordinate System
struct GCS
{
public:

	double Latitude;  // ����
	double Longitude;  // �浵

	GCS()
	{
		Latitude = 0.f;
		Longitude = 0.f;
	}
	GCS(double _latitude, double _longitude)
		: Latitude(_latitude), Longitude(_longitude)
	{
	}

	// UNDONE: GCS �Ÿ� ��� ��Ȯ�� ����
	static double GetDistance(GCS& c1, GCS& c2)
	{
		return sqrt(pow(c1.Latitude - c2.Latitude, 2) + pow(c1.Longitude - c2.Longitude, 2));
	}
	double GetDistance(GCS& other) const
	{
		return sqrt(pow(Latitude - other.Latitude, 2) + pow(Longitude - other.Longitude, 2));
	}
};

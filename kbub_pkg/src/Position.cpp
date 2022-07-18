#pragma once
#include <cmath>
#include "GCS.cpp"

using namespace std;

// XY ��ǥ
struct Position
{
public:

	double x;
	double y;

	static double CalcDistance(Position& p1, Position& p2)
	{
		return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
	}
	double CalcDistance(Position& other) const
	{
		return sqrt(pow(x - other.x, 2) + pow(y - other.y, 2));
	}
	////
	static Position fromGCS(GCS& gcs)
	{
		// TODO: GCS���� XY ��ǥ��
	}
};

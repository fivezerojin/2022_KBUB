#pragma once
#include <queue>
#include <fstream>
#include "Position.cpp"  // for 좌표계
#include "InputManager.cpp"  // for GPS
#include "Include/json/json.h"

using namespace std;

// 글로벌 경로 저장 및 경로 생성
class GlobalPathManager
{
	// 도착 판정 허용 거리 오차
	const float  ARRIVAL_TOLERANCE = 0.0001f;

	// 글로벌 경로 좌표(웨이포인트) 큐. front가 다음으로 이동할 좌표
	queue<Position> waypoints;

	Json::Value nodeRoot;
	Json::Value linkRoot;

	GlobalPathManager() {};
	GlobalPathManager(const GlobalPathManager& other) {};
	~GlobalPathManager() {};

public:

	// Singleton
	static GlobalPathManager* Get()
	{
		static GlobalPathManager* instance = new GlobalPathManager();
		return instance;
	}

	// 다음으로 이동할 웨이포인트
	Position GetNextWaypoint()
	{
		return waypoints.front();
	}

	// 웨이포인트에 도달했는지 확인하고 처리
	// pos: 현재 좌표
	bool CheckAndUpdateArrival(Position& pos)
	{
		if (pos.CalcDistance(waypoints.front()) < ARRIVAL_TOLERANCE)
		{
			ArriveWaypoint();
			return true;
		}
		return false;
	}

	// 웨이포인트 도달
	void ArriveWaypoint()
	{
		if (waypoints.size() == 0) return;
		waypoints.pop();
	}

	// json 읽어들이기
	void ReadJson()
	{
		ifstream ifstream;

		// Read node json.
		ifstream.open("Data/node.geojson", ifstream::binary);
		if (ifstream.is_open())
		{
			ifstream >> nodeRoot;
			ifstream.close();
			cout << "name: " << nodeRoot["name"] << '\n';
		}
		else
		{
			cout << "Failed to open nodeIFS\n";
		}

		// FIXME: link json 읽기 오류
		return;

		// Read link json.
		ifstream.open("Data/link.geojson", ifstream::binary);
		if (ifstream.is_open())
		{
			ifstream >> linkRoot;
			ifstream.close();
			cout << "name: " << linkRoot["name"] << '\n';
		}
		else
		{
			cout << "Failed to open linkIFS\n";
		}
	}

};

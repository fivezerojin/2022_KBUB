#pragma once
#include <queue>
#include <fstream>
#include "Position.cpp"  // for ��ǥ��
#include "InputManager.cpp"  // for GPS
#include "Include/json/json.h"

using namespace std;

// �۷ι� ��� ���� �� ��� ����
class GlobalPathManager
{
	// ���� ���� ��� �Ÿ� ����
	const float  ARRIVAL_TOLERANCE = 0.0001f;

	// �۷ι� ��� ��ǥ(��������Ʈ) ť. front�� �������� �̵��� ��ǥ
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

	// �������� �̵��� ��������Ʈ
	Position GetNextWaypoint()
	{
		return waypoints.front();
	}

	// ��������Ʈ�� �����ߴ��� Ȯ���ϰ� ó��
	// pos: ���� ��ǥ
	bool CheckAndUpdateArrival(Position& pos)
	{
		if (pos.CalcDistance(waypoints.front()) < ARRIVAL_TOLERANCE)
		{
			ArriveWaypoint();
			return true;
		}
		return false;
	}

	// ��������Ʈ ����
	void ArriveWaypoint()
	{
		if (waypoints.size() == 0) return;
		waypoints.pop();
	}

	// json �о���̱�
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

		// FIXME: link json �б� ����
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

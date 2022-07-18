#pragma once
#include "CarController.cpp"

// 운행 모드. 우선순위 오름차순
enum DRIVE_MODE
{
	STOP,  // 정차
	DRIVE,  // 차선을 따라 주행 (카메라)
	SIGNAL,  // 교통 신호 처리 (카메라)
	AVOID,  // 도로 장애물 회피 (LiDAR)
}; 

// 운행 모드 결정
class StateArbiter
{
private:

	static DRIVE_MODE driveMode;

	StateArbiter();

public:

	static DRIVE_MODE GetDriveMode()
	{
		return driveMode;
	}

	// 운행 모드 결정
	static DRIVE_MODE DetermineDriveMode(bool bSignal, bool bObstacle)
	{
		// UNDONE: DetermineDriveMode
		DRIVE_MODE driveMode = DRIVE_MODE::DRIVE;

		return driveMode;
	}
};

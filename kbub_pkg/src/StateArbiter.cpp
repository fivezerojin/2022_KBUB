#pragma once
#include "CarController.cpp"

// ���� ���. �켱���� ��������
enum DRIVE_MODE
{
	STOP,  // ����
	DRIVE,  // ������ ���� ���� (ī�޶�)
	SIGNAL,  // ���� ��ȣ ó�� (ī�޶�)
	AVOID,  // ���� ��ֹ� ȸ�� (LiDAR)
}; 

// ���� ��� ����
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

	// ���� ��� ����
	static DRIVE_MODE DetermineDriveMode(bool bSignal, bool bObstacle)
	{
		// UNDONE: DetermineDriveMode
		DRIVE_MODE driveMode = DRIVE_MODE::DRIVE;

		return driveMode;
	}
};

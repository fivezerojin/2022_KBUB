#pragma once
#include "GCS.cpp"

// ������ ���� ��ȣ ����
class CarController
{
	double wheelSpeed;  // ���� �ӵ�
	double wheelAngle;  // ���� ���� (0 ���� / - ���� / + ����) 
	double brake;
	double gear;
	CarController()
	{
		// Initialize
		wheelSpeed = 0.f;
		wheelAngle = 0.f;
		brake = 1.0;
		gear = 0.0f;
	};
	CarController(const CarController& other) {};
	~CarController() {};

public:

	// Singleton
	static CarController* Get()
	{
		static CarController* instance = new CarController();
		return instance;
	}
	void SetWheelSpeed(double speed)
	{
		wheelSpeed = speed;
	}
	void SetWheelAngle(double angle)
	{
		wheelAngle = angle;
	}
	void SetBrake(double brake_)
	{
		brake = brake_;
	}

	void SetBrake(bool isBrake)
	{
		if(isBrake)
		{
			brake = 180.0;
		}
		else
		{
			brake = 1.0;
		}
		
	}
	double GetWheelSpeed() const
	{
		return wheelSpeed;
	}

	double GetWheelAngle() const
	{
		return wheelAngle;
	}
	
	double GetBrake() const
	{
		return brake;
	}
	double GetGear()
	{
		return gear;
	}
	void SetGear(double gear_)
	{
		gear = gear_;
	}
	bool isBrake() const
	{
		
		if(brake > 100.0)
		{
			return true;
		}
		return false;
	}
};

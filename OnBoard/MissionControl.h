#include "Estimator.h"
#pragma once

namespace MissionControl
{

	class Point : Estimator::Matrix
	{
	private:
	public:
		Point();
		Point(float lat, float lon);
		float GetLatitude();
		float GetLongitude();
		void SetLatitude(float lat);
		void SetLongitude(float lon);
	};

#pragma region SquareMission
	class SquareMission
	{
	private:
	public:
		SquareMission &operator=(const SquareMission &other);
		SquareMission();
		SquareMission(float baseLat, float baseLong);
		float *GetCurrentError(float currentLat, float currentLon);
		Point points[4];
		int currentPosition = 0;
		float offset = 0.004;
	};
#pragma endregion

}

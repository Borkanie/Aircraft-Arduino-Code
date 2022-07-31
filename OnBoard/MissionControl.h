#include "Estimator.h"
#pragma once

namespace MissionControl
{

	/**
	 * An implementation of a coordinate point.
	 */
	class CoordinatePoint : Estimator::Matrix
	{
	private:
	public:
		CoordinatePoint();
		CoordinatePoint(float lat, float lon);
		// Returns the current latitude.
		float GetLatitude();
		// Returns the current longitude.
		float GetLongitude();
		// Updates the latitude.
		void SetLatitude(float lat);
		// Updates the longitude.
		void SetLongitude(float lon);
	};

#pragma region SquareMission

	/**
	 * An implementation of a basic square mission with a fixed size that is calculated relative to a starting point.
	 */
	class RectangleMission
	{
	private:
	public:
		RectangleMission &operator=(const RectangleMission &other);
		RectangleMission();
		RectangleMission(float baseLat, float baseLong);
		// Returns the current error relative to the desired square mission ath the current time.
		float *GetCurrentError(float currentLat, float currentLon);
		// The list of points that represent our mission.
		CoordinatePoint points[4];
		// Sets up thecurrent state of the mission in the rectangle.
		// Each state is represents a line.
		int missionPhase = 0;
		// Offset relative to latitude because of reading errors.
		float offset = 0.004;
	};
#pragma endregion

}

#include "MissionControl.h"
#include "Estimator.h"

namespace MissionControl
{
	CoordinatePoint::CoordinatePoint() : Matrix::Matrix(2, 1)
	{
		matrix[0][0] = 0;
		matrix[1][0] = 0;
	}
	CoordinatePoint::CoordinatePoint(float lat, float lon) : Matrix::Matrix(2, 1)
	{
		matrix[0][0] = lat;
		matrix[1][0] = lon;
	}
	float CoordinatePoint::GetLatitude()
	{
		return matrix[0][0];
	}
	float CoordinatePoint::GetLongitude()
	{
		return matrix[1][0];
	}
	void CoordinatePoint::SetLatitude(float lat)
	{
		matrix[0][0] = lat;
	}
	void CoordinatePoint::SetLongitude(float lon)
	{
		matrix[1][0] = lon;
	}

#pragma region SquareMission
	RectangleMission &RectangleMission::operator=(const RectangleMission &other)
	{
		this->points[0] = other.points[0];
		this->points[1] = other.points[1];
		this->points[2] = other.points[2];
		this->points[3] = other.points[3];
		this->missionPhase = other.missionPhase;
		this->offset = other.offset;
		return *this;
	}

	RectangleMission::RectangleMission()
	{
		float baseLat = 0;
		float baseLong = 0;
		// point 1 will be the current postion
		points[0].SetLatitude(baseLat);
		points[0].SetLongitude(baseLong);
		// Added on longitude  0.002
		// so point 1 will be with 150 meters more north
		points[1].SetLatitude(baseLat + offset);
		points[1].SetLongitude(baseLong);
		// Added on longitude  0.002
		// Added on latitude  0.002
		// so point 2 will be with 150 meters more north and 150 more east
		points[2].SetLatitude(baseLat + offset);
		points[2].SetLongitude(baseLong + offset);
		// Added on latitude  0.002
		// so point 1 will be with 150 meters more east
		points[3].SetLatitude(baseLat);
		points[3].SetLongitude(baseLong + offset);
	}

	RectangleMission::RectangleMission(float baseLat, float baseLong)
	{
		// point 1 will be the current postion
		points[0].SetLatitude(baseLat);
		points[0].SetLongitude(baseLong);
		// Added on longitude  0.002
		// so point 1 will be with 150 meters more north
		points[1].SetLatitude(baseLat + offset);
		points[1].SetLongitude(baseLong);
		// Added on longitude  0.002
		// Added on latitude  0.002
		// so point 2 will be with 150 meters more north and 150 more east
		points[2].SetLatitude(baseLat + offset);
		points[2].SetLongitude(baseLong + offset);
		// Added on latitude  0.002
		// so point 1 will be with 150 meters more east
		points[3].SetLatitude(baseLat);
		points[3].SetLongitude(baseLong + offset);
	}
	float *RectangleMission::GetCurrentError(float currentLat, float currentLon)
	{
		float Lat = currentLat * DegreesToRadianstConst;
		float Lon = currentLon * DegreesToRadianstConst;

		float x = earthRadius * Estimator::CosineInCluj(Lat) * Estimator::CosineInCluj(Lon);
		float y = earthRadius * Estimator::CosineInCluj(Lat) * Estimator::SineInCluj(Lon);
		float z = earthRadius * Estimator::SineInCluj(Lon);

		float pointLat;
		float pointLon;
		float pointx;
		float pointy;
		float pointz;

		float *result = new float[2];
		switch (missionPhase)
		{
		case 0:
			if (currentLat < points[1].GetLatitude())
			{
				// we go north to point 1
				pointLat = points[1].GetLatitude() * DegreesToRadianstConst;
				pointLon = points[1].GetLongitude() * DegreesToRadianstConst;
				pointx = earthRadius * Estimator::CosineInCluj(pointLat) * Estimator::CosineInCluj(pointLon);
				pointy = earthRadius * Estimator::CosineInCluj(pointLat) * Estimator::SineInCluj(pointLon);
				pointz = earthRadius * Estimator::SineInCluj(pointLon);
				result[0] = pointz - z;
				result[1] = pointy - y;
			}
			else
			{
				// we got over the point
				missionPhase = 1;
				// we go east to point 2
				pointLat = points[2].GetLatitude() * DegreesToRadianstConst;
				pointLon = points[2].GetLongitude() * DegreesToRadianstConst;
				pointx = earthRadius * Estimator::CosineInCluj(pointLat) * Estimator::CosineInCluj(pointLon);
				pointy = earthRadius * Estimator::CosineInCluj(pointLat) * Estimator::SineInCluj(pointLon);
				pointz = earthRadius * Estimator::SineInCluj(pointLon);
				result[0] = pointz - z;
				result[1] = pointx - x;
			}
			return result;
		case 1:
			if (currentLon < points[2].GetLongitude())
			{
				// we go east to point 2
				pointLat = points[2].GetLatitude() * DegreesToRadianstConst;
				pointLon = points[2].GetLongitude() * DegreesToRadianstConst;
				pointx = earthRadius * Estimator::CosineInCluj(pointLat) * Estimator::CosineInCluj(pointLon);
				pointy = earthRadius * Estimator::CosineInCluj(pointLat) * Estimator::SineInCluj(pointLon);
				pointz = earthRadius * Estimator::SineInCluj(pointLon);
				result[0] = pointz - z;
				result[1] = pointx - x;
			}
			else
			{
				// we got over the point
				missionPhase = 2;
				// we go south to point 3
				pointLat = points[3].GetLatitude() * DegreesToRadianstConst;
				pointLon = points[3].GetLongitude() * DegreesToRadianstConst;
				pointx = earthRadius * Estimator::CosineInCluj(pointLat) * Estimator::CosineInCluj(pointLon);
				pointy = earthRadius * Estimator::CosineInCluj(pointLat) * Estimator::SineInCluj(pointLon);
				pointz = earthRadius * Estimator::SineInCluj(pointLon);
				result[0] = pointz - z;
				result[1] = pointy - y;
			}
			return result;
		case 2:
			if (currentLat > points[3].GetLatitude())
			{
				// we go south to point 3
				pointLat = points[3].GetLatitude() * DegreesToRadianstConst;
				pointLon = points[3].GetLongitude() * DegreesToRadianstConst;
				pointx = earthRadius * Estimator::CosineInCluj(pointLat) * Estimator::CosineInCluj(pointLon);
				pointy = earthRadius * Estimator::CosineInCluj(pointLat) * Estimator::SineInCluj(pointLon);
				pointz = earthRadius * Estimator::SineInCluj(pointLon);
				result[0] = pointz - z;
				result[1] = pointy - y;
			}
			else
			{
				// we got over the point
				missionPhase = 3;
				// we go west to point 0
				pointLat = points[0].GetLatitude() * DegreesToRadianstConst;
				pointLon = points[0].GetLongitude() * DegreesToRadianstConst;
				pointx = earthRadius * Estimator::CosineInCluj(pointLat) * Estimator::CosineInCluj(pointLon);
				pointy = earthRadius * Estimator::CosineInCluj(pointLat) * Estimator::SineInCluj(pointLon);
				pointz = earthRadius * Estimator::SineInCluj(pointLon);
				result[0] = pointz - z;
				result[1] = pointx - x;
			}
			return result;
		case 3:
			if (currentLon > points[0].GetLongitude())
			{
				// we go west to point 0
				pointLat = points[0].GetLatitude() * DegreesToRadianstConst;
				pointLon = points[0].GetLongitude() * DegreesToRadianstConst;
				pointx = earthRadius * Estimator::CosineInCluj(pointLat) * Estimator::CosineInCluj(pointLon);
				pointy = earthRadius * Estimator::CosineInCluj(pointLat) * Estimator::SineInCluj(pointLon);
				pointz = earthRadius * Estimator::SineInCluj(pointLon);
				result[0] = pointz - z;
				result[1] = pointx - x;
			}
			else
			{
				// we got over the point
				missionPhase = 0;
				// we go north to point 1
				pointLat = points[1].GetLatitude() * DegreesToRadianstConst;
				pointLon = points[1].GetLongitude() * DegreesToRadianstConst;
				pointx = earthRadius * Estimator::CosineInCluj(pointLat) * Estimator::CosineInCluj(pointLon);
				pointy = earthRadius * Estimator::CosineInCluj(pointLat) * Estimator::SineInCluj(pointLon);
				pointz = earthRadius * Estimator::SineInCluj(pointLon);
				result[0] = pointz - z;
				result[1] = pointy - y;
			}
			return result;
		default:
			missionPhase = 0;
			return result;
		}
	}
#pragma endregion

}
#include "ShapeConvex.h"

Bounds ShapeConvex::GetBounds(const Vec3& position, const Quat& orientation) const
{
	Vec3 corners[8];

	corners[0] = Vec3(_Bounds.mins.x, _Bounds.mins.y, _Bounds.mins.z);
	corners[1] = Vec3(_Bounds.mins.x, _Bounds.mins.y, _Bounds.maxs.z);
	corners[2] = Vec3(_Bounds.mins.x, _Bounds.maxs.y, _Bounds.mins.z);
	corners[3] = Vec3(_Bounds.maxs.x, _Bounds.mins.y, _Bounds.mins.z);

	corners[4] = Vec3(_Bounds.maxs.x, _Bounds.maxs.y, _Bounds.maxs.z);
	corners[5] = Vec3(_Bounds.maxs.x, _Bounds.maxs.y, _Bounds.mins.z);
	corners[6] = Vec3(_Bounds.maxs.x, _Bounds.mins.y, _Bounds.maxs.z);
	corners[7] = Vec3(_Bounds.mins.x, _Bounds.maxs.y, _Bounds.maxs.z);

	Bounds bounds;

	//rebuilds bounds from corner.
	for (int i = 0; i < 7; i++)
	{
		corners[i] = orientation.RotatePoint(corners[i]) + position;
		bounds.Expand(corners[i]);

	}
	return bounds;
}

//Will be used for CCD , takes in a direction and angular velocity of object and returns us the velocity of the vertex travelling the fastest in that direction
float ShapeConvex::FastestLinearSpeed(const Vec3& angular_velocity, const Vec3& direction) const
{
	float max_speed = 0.0f;

	for (int i = 0; i < _Points.size(); i++)
	{
		Vec3 r = _Points[i] - _CentreOfMass;
		Vec3 linear_velocity = angular_velocity.Cross(r);

		float speed = direction.Dot(linear_velocity);

		if (speed > max_speed)
		{
			max_speed = speed;
		}
	}
	return max_speed;
}

#include "ShapeBox.h"



//Creates the 8 "vertex" points, considered bounds of the Box.
void ShapeBox::Build(const Vec3* points, const int num)
{
	for (int i = 0; i < num; i++)
	{
		_Bounds.Expand(points[i]);
	}

	_Points.clear();

	//Add points utilising mins & maxes to allign each point in the correct corresponding corner to create the box

	_Points.push_back(Vec3(_Bounds.mins.x, _Bounds.mins.y, _Bounds.mins.z));
	_Points.push_back(Vec3(_Bounds.maxs.x, _Bounds.mins.y, _Bounds.mins.z));
	_Points.push_back(Vec3(_Bounds.mins.x, _Bounds.maxs.y, _Bounds.mins.z));
	_Points.push_back(Vec3(_Bounds.mins.x, _Bounds.mins.y, _Bounds.maxs.z));

	_Points.push_back(Vec3(_Bounds.maxs.x, _Bounds.maxs.y, _Bounds.maxs.z));
	_Points.push_back(Vec3(_Bounds.mins.x, _Bounds.maxs.y, _Bounds.maxs.z));
	_Points.push_back(Vec3(_Bounds.maxs.x, _Bounds.mins.y, _Bounds.maxs.z));
	_Points.push_back(Vec3(_Bounds.maxs.x, _Bounds.maxs.y, _Bounds.mins.z));


	//Correctly position centre of mass , adding all points together, including negative and positive intergers will gives us a value that is in centre of  box
	//E.G -1 + 1 = 0 , centre of the value of these whole numbers 
	_CentreOfMass = (_Bounds.maxs + _Bounds.mins) * 0.5f;
}

//Remember the support function only takes direction , returns the vertex (point) on the shape that is furthest in that direction
Vec3 ShapeBox::Support(const Vec3& direction, const Vec3& position, const Quat& orientation, const float bias) const
{
	//Finding point furthest in this direction
	Vec3 max_point = orientation.RotatePoint(_Points[0]) + position;
	float max_distance = direction.Dot(max_point);

	for (int i = 1; i < _Points.size(); i++)
	{
		const Vec3 point = orientation.RotatePoint(_Points[i]) + position;
		const float distance = direction.Dot(point);

		if (distance > max_distance)
		{
			max_distance = distance;
			max_point = point;
		}
	}

	Vec3 normal = direction;
	normal.Normalize();
	normal *= bias;
	return max_point + normal;
}

Mat3 ShapeBox::InertiaTensor()
{
	//Inertia Tensor for box centered around zero
	const float dx = _Bounds.maxs.x - _Bounds.mins.x;
	const float dy = _Bounds.maxs.y - _Bounds.mins.y;
	const float dz = _Bounds.maxs.z - _Bounds.mins.z;

	Mat3 tensor;
	tensor.Zero();
	tensor.rows[0][0] = (dy * dy + dz * dz) / 12.0f;
	tensor.rows[1][1] = (dx * dx + dz * dz) / 12.0f;
	tensor.rows[2][2] = (dx * dx + dy * dy) / 12.0f;

	//We need  to use the parallel axis theorem to get the intertia tensor for a box
	//This is not centered around the origin

	Vec3 cm;
	cm.x = (_Bounds.maxs.x + _Bounds.mins.x) * 0.5f;
	cm.y = (_Bounds.maxs.y + _Bounds.mins.y) * 0.5f;
	cm.z = (_Bounds.maxs.z + _Bounds.mins.z) * 0.5f;

	//Displacement from centre of mass to the origin
	const Vec3 R = Vec3(0.0f, 0.0f, 0.0f) - cm;
	const float R2 = R.GetLengthSqr();

	Mat3 pat_tensor;
	pat_tensor.rows[0] = Vec3(R2 - R.x * R.x, R.x * R.y, R.x * R.z);
	pat_tensor.rows[1] = Vec3(R.y * R.x, R2 - R.y * R.y, R.y * R.z);
	pat_tensor.rows[2] = Vec3(R.z * R.x, R.z * R.y, R2 - R.z * R.z);

	//Adding the centre of mass tensor and the parallel axis theorm tensor together
	tensor += pat_tensor;
	return tensor;
}

Bounds ShapeBox::GetBounds(const Vec3& position, const Quat& orientation) const
{
	Vec3 corners[8];

	corners[0] = Vec3(_Bounds.mins.x, _Bounds.mins.y , _Bounds.mins.z);
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

Bounds ShapeBox::GetBounds() const
{
	return _Bounds;
}

//Will be used for CCD , takes in a direction and angular velocity of object and returns us the velocity of the vertex travelling the fastest in that direction
float ShapeBox::FastestLinearSpeed(const Vec3& angular_velocity, const Vec3& direction) const
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

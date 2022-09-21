#pragma once
#include "Shape.h"
/*
	Boxes are common in video games. Used to simulate them.

	


*/

class ShapeBox : public Shape
{
public:

	//Upon Initialisation , we build the Shape ,matched with points 
	explicit ShapeBox(const Vec3* points, const int num)
	{
		Build(points, num);
	}


	void Build(const Vec3* points, const int num) override;

	//This function will be used to support giving us the point on a given convex shape that is furthest in a particular direction
	Vec3 Support(const Vec3& direction, const Vec3& position, const Quat& orientation, const float bias) const override;

	Mat3 InertiaTensor() override;

	Bounds GetBounds(const Vec3& position, const Quat& orientation) const override;
	Bounds GetBounds() const override;

	float FastestLinearSpeed(const Vec3& angular_velocity, const Vec3& direction) const override;
private:
	std::vector<Vec3> _Points;
	Bounds _Bounds;
};

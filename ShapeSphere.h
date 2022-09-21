#pragma once
#include "Shape.h"


class ShapeSphere : public Shape
{
public:

	explicit ShapeSphere(float radius) : _Radius(radius)
	{
		_CentreOfMass.Zero();
	}

	//This function will be used to support giving us the point on a given convex shape that is furthest in a particular direction
    Vec3 Support(const Vec3& direction, const Vec3& position, const Quat& orientation, const float bias) const override;

	Mat3 InertiaTensor() override;

	Bounds GetBounds(const Vec3& pos, const Quat& orient)  const override;
	Bounds GetBounds() const override;
	
	ShapeType GetType() const override { return SHAPE_SPHERE; };

public: 
	float _Radius;
};
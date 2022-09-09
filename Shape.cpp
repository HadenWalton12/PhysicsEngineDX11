#include "Shape.h"
#include "Body.h"

struct point_t
{

	Vec3 xyz; //Point on minkowski sum
	Vec3 point_a;//Point on body A
	Vec3 point_b;//Point on body B


	point_t() : xyz(0.0f), point_a(0.0f), point_b(0.0f) {}

	const point_t& operator = (const point_t& rhs)
	{
		xyz = rhs.xyz;
		point_a = rhs.point_a;
		point_b = rhs.point_b;
		return *this;

	}

	bool operator == (const point_t& rhs) const
	{
		return((point_a == rhs.point_a) && (point_b == rhs.point_b) && (xyz == rhs.xyz));
	}

};

point_t Support(const Body* bodyA, const Body* bodyB, Vec3 direction, const float bias)
{
	direction.Normalize();

	point_t point;

	//Find the point in A furthest in direction
	point.point_a = bodyA->_Shape->Support(direction, bodyA->_Position, &bodyA->_Orientation, bias);


	direction *= -1.0f;

	//Find the point in B furthest in the opposite direction
	point.point_b = bodyB->_Shape->Support(direction, bodyB->_Position, &bodyB->_Orientation, bias);

	//Return the point in the minkowski sum , furthest in direction
	point.xyz = point.point_a - point.point_b;

	return point;
}

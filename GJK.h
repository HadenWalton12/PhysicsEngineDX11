#pragma once
#include "Body.h"
#include "Shape.h"


bool GJK_DoesIntersect(const Body* body_a, const Body* body_b);
bool GJK_DoesIntersect(const Body* body_a, const Body* body_b, const float bias , Vec3 & point_on_a , Vec3 & point_on_b);
bool GJK_ClosestPoints(const Body* body_a, const Body* body_b, Vec3& point_on_a, Vec3& point_on_b);

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

struct tri_t
{
	int a;
	int b;
	int c;
};

struct edge_t
{
	int a;
	int b;

	//used to assign edges with their values
	bool operator == (const edge_t& rhs) const
	{
		return ((a == rhs.a && b == rhs.b) || (a == rhs.b && b == rhs.a));
	}


};

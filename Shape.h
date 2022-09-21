#pragma once
#include "Object.h"
#include "Vector.h"
#include "Quanternion.h"
#include "Bounds.h"
class Shape
{
public:
	//New Changes 
	//In order to extend the collision detection to allow collision detection of general convex shapes & regular shapes, the additions will be waranted
	enum ShapeType
	{
		SHAPE_SPHERE,
		//Added two now shapes types
		SHAPE_BOX,
		SHAPE_CONVEX,
	};


	//
	virtual void Build(const Vec3 * points , const int num) {}

	//This function will bve used to support giving us the point on a given convex shape that is furthest in a particular direction
	virtual Vec3 Support(const Vec3& direction, const Vec3& position, const Quat& orientation, const float bias) const = 0;


	//Nessecary for CCD , objects that are "long" in size , an long object that is rotating can hit other objects, even though the lonear velocity can be zero.
	//For spheres, we dont care how quickly they rotate , on the speed/ linear velocity they travel

	//THis WIll be looked into further when programming conservative advanced algorithm.
	virtual float FastestLinearSpeed(const Vec3 & angular_velocity ,const Vec3 & direction) const
	{
		return 0.0f;
	}
	virtual Bounds GetBounds(const Vec3& position, const Quat& orientation) const = 0;

	virtual Vec3 GetCentreOfMass(const Vec3& angular_velocity, const Vec3& direction) const
	{
		return 0.0f;
	}


	virtual ShapeType GetType() const = 0;

	ShapeType type;


	virtual Bounds GetBounds(const Vec3& pos, const Quat& orient)  const = 0;
	virtual Bounds GetBounds() const = 0;

	virtual Vec3 GetCentreOfMass() const { return _CentreOfMass; }

	//Inate Objects Will Be Passed Down To Child CLasses, Security Access Modifer Enabling Inheritance Principle
	Object* _Object;

	//Will Be Explained Later - However an integral value to the Physics System
	Vec3 _CentreOfMass;

	 

	virtual Mat3 InertiaTensor()
	{
		Mat3 o;
		return o;
	}

};




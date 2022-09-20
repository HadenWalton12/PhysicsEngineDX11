#pragma once
#include "Shape.h"

class ShapeBox : public Shape
{
public:

	ShapeType _ShapeType;
	std::vector<Vec3> _BoxPoints;
	Bounds _BoxBounds;


	//Used To Initialise The Object

	explicit ShapeBox(const Vec3 * points , const int num, Surface surface, RenderCommands* render, TextureComponent* tex, XMFLOAT3 translate, XMFLOAT3 scale, XMFLOAT3 rotate)
	{
		Build(points , num);
		_CentreOfMass.Zero();

		//Sets Up THe Object Bound To THis Shape , Replacing having to individualy define a shape in a scene class to the same length as this
		_Object = new Object(render, L"Floor.dds", tex);
		_Object->SetSurface(surface.Ambient, surface.Diffuse, surface.Specular, surface.SpecularPower);

		_Object->SetVertexShader(L"DX11 Framework.fx");
		_Object->SetPixelShader(L"DX11 Framework.fx");

		_Object->SetTransformation(translate, scale, rotate);

		//Allows us to get the assigned type of Shape
		_ShapeType = SHAPE_BOX;
	}
	explicit ShapeBox(const Vec3* points, const int num, Surface surface, RenderCommands* render, TextureComponent* tex)
	{
		Build(points, num);
		_Object = new Object(render, L"Floor.dds", tex);
		_Object->SetSurface(surface.Ambient, surface.Diffuse, surface.Specular, surface.SpecularPower);

		_Object->SetVertexShader(L"DX11 Framework.fx");
		_Object->SetPixelShader(L"DX11 Framework.fx");
		//Allows us to get the assigned type of Shape
		_ShapeType = SHAPE_BOX;
	}
	//The Support Function allows us to find the point on a convex shaoe that is the furthest in a particular direction.
	//For Example- Lets say our cube is moving -x , on the cube , this function will return a point on the cube that is the most furthest in that direction, anypoint on a surface
	//of the cube
	Vec3 Support(const Vec3& direction, const Vec3& position, const Quat* orientation, const float bias) const override;

	void Build(const Vec3* points, const int num);



	float FastestLinearSpeed(const Vec3& angular_velocity, const Vec3& directions) const override;
	
	//Box Has Different InertiaTensor , Remember The Inertia Tensor Is Considered The Mass Matrix , Used TO Distribute Mass Correctly Throughout The OBject

	Mat3 InertiaTensor() const;

	Mat3 _InertiaTensor = InertiaTensor();

	Mat3 _GetInertiaTensor() override { return _InertiaTensor; }
	
	Bounds GetBounds(const Vec3& pos, const Quat& orient)  const override;
	Bounds GetBounds() const override;
	ShapeType GetType() const override;
};
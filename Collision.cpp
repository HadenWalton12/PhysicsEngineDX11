#include "Collision.h"
#include "GJK.h"


bool SphereSphereStatic(ShapeSphere* sphere_a, ShapeSphere* sphere_b, Vec3& pos_a, Vec3& pos_b, Vec3& pt_On_A, Vec3& pt_On_B)
{
	const Vec3 ab = pos_b - pos_a;
	Vec3 normal = ab;
	normal.Normalize();

	pt_On_A = pos_a + normal * sphere_a->_Radius;
	pt_On_B = pos_b - normal * sphere_b->_Radius;
	
	const float radius_ab = sphere_a->_Radius + sphere_b->_Radius;
	const float lengthSquare = ab.GetLengthSqr();

	if (lengthSquare <= (radius_ab * radius_ab))
	{
		return true;
	}
	
	return false;
}



bool Intersect(Body* bodyA, Body* bodyB, Contact& contact)
{
	contact._BodyA = bodyA;
	contact._BodyB = bodyB;
	contact._TimeOfImpact = 0.0f;

	if (bodyA->_Shape->GetType() == Shape::SHAPE_SPHERE && bodyB->_Shape->GetType() == Shape::SHAPE_SPHERE) {
		 ShapeSphere* sphereA = ( ShapeSphere*)bodyA->_Shape;
		 ShapeSphere* sphereB = ( ShapeSphere*)bodyB->_Shape;

		Vec3 posA = bodyA->_Position;
		Vec3 posB = bodyB->_Position;

		if (SphereSphereStatic(sphereA, sphereB, posA, posB, contact.ptOnA_WorldSpace, contact.ptOnB_WorldSpace)) {
			contact.Normal = posA - posB;
			contact.Normal.Normalize();

			contact.ptOnA_LocalSpace = bodyA->WorldSpaceToBodySpace(contact.ptOnA_WorldSpace);
			contact.ptOnB_LocalSpace = bodyB->WorldSpaceToBodySpace(contact.ptOnB_WorldSpace);

			Vec3 ab = bodyB->_Position - bodyA->_Position;
			float r = ab.GetMagnitude() - (sphereA->_Radius + sphereB->_Radius);
			contact._SeperationDistance = r;
			return true;
		}
	}
	else {
		Vec3 ptOnA;
		Vec3 ptOnB;
		const float bias = 0.001f;
		if (GJK_DoesIntersect(bodyA, bodyB, bias, ptOnA, ptOnB)) {
			// There was an intersection, so get the contact data
			Vec3 normal = ptOnB - ptOnA;
			normal.Normalize();

			ptOnA -= normal * bias;
			ptOnB += normal * bias;

			contact.Normal = normal;

			contact.ptOnA_WorldSpace = ptOnA;
			contact.ptOnB_WorldSpace = ptOnB;

			contact.ptOnA_LocalSpace = bodyA->WorldSpaceToBodySpace(contact.ptOnA_WorldSpace);
			contact.ptOnB_LocalSpace = bodyB->WorldSpaceToBodySpace(contact.ptOnB_WorldSpace);

			Vec3 ab = bodyB->_Position - bodyA->_Position;
			float r = (ptOnA - ptOnB).GetMagnitude();
			contact._SeperationDistance = -r;
			return true;
		}

		// There was no collision, but we still want the contact data, so get it
		GJK_ClosestPoints(bodyA, bodyB, ptOnA, ptOnB);
		contact.ptOnA_WorldSpace = ptOnA;
		contact.ptOnB_WorldSpace = ptOnB;

		contact.ptOnA_LocalSpace = bodyA->WorldSpaceToBodySpace(contact.ptOnA_WorldSpace);
		contact.ptOnB_LocalSpace = bodyB->WorldSpaceToBodySpace(contact.ptOnB_WorldSpace);

		Vec3 ab = bodyB->_Position - bodyA->_Position;
		float r = (ptOnA - ptOnB).GetMagnitude();
		contact._SeperationDistance = r;
	}
	return false;
}

bool ConservativeAdvance(Body* body_a, Body* body_b, float dt, Contact& contact)
{
	contact._BodyA = body_a;
	contact._BodyB = body_b;

	float toi = 0.0f;

	int num_intergers = 0;


	//Advance the positions of the bodies until they touch or theres not the time left.
	while (dt > 0.0f)
	{
		//Check for interesection
		bool did_intersect = Intersect(body_a, body_b, contact);

		if (did_intersect)
		{
			contact._TimeOfImpact = toi;
			body_a->Update(-toi);
			body_b->Update(-toi);

			return true;

		}

		++num_intergers;

		if (num_intergers > 10)
		{
			break;
		}

		//Get the vector from the closest point on A to closest point on B
		Vec3 ab = contact.ptOnB_WorldSpace - contact.ptOnA_WorldSpace;
		ab.Normalize();

		//Project the relative velocity onto the ray of shortest distance
		Vec3 relative_velocity = body_a->_LinearVelocity - body_b->_LinearVelocity;
		float ortho_speed = relative_velocity.Dot(ab);

		//Add to the orthospeed the maxium angular speeds of the relative shapes
		float angular_speed_a = body_a->_Shape->FastestLinearSpeed(body_a->_AngularVelocity, ab);
		float angular_speed_b = body_b->_Shape->FastestLinearSpeed(body_b->_AngularVelocity, ab * -1.0f);
		ortho_speed += angular_speed_a + angular_speed_b;

		if (ortho_speed <= 0.0f)
		{
			break;
		}

		float time_to_go = contact._SeperationDistance / ortho_speed;

		if (time_to_go > dt)
		{
			break;
		}

		dt -= time_to_go;
		toi += time_to_go;
		body_a->Update(time_to_go);
		body_b->Update(time_to_go);

	}

	//Unwind the clock
	body_a->Update(-toi);
	body_b->Update(-toi);
	return false;
}

bool RaySphere(Vec3 ray_start, Vec3& ray_direction, Vec3 sphere_centre, float sphere_radius, float& t1, float& t2)
{
	Vec3 m = sphere_centre - ray_start;
	float a = ray_direction.Dot(ray_direction);
	float b = m.Dot(ray_direction);
	float c = m.Dot(m) - sphere_radius * sphere_radius;

	float delta = b * b - a * c;

	float invA = 1.0f / a;

	if (delta < 0)
	{
		return false;
	}

	float delta_root = sqrtf(delta);

	t1 = invA * (b - delta_root);
	t2 = invA * (b + delta_root);

	return true;

}

bool SphereSphereDynamic(ShapeSphere* sphere_a, ShapeSphere* sphere_b, Vec3& pos_a, Vec3& pos_b, Vec3& vel_a, Vec3& vel_b, float dt, Vec3& pt_On_A, Vec3& pt_On_B, float& toi)
{

	Vec3 relative_velocity = vel_a - vel_b;

	Vec3 start_PT_a = pos_a;
	Vec3 end_PT_a = pos_a + relative_velocity * dt;

	Vec3 ray_direction = end_PT_a - start_PT_a;

	float t0 = 0;
	float t1 = 0;

	if (ray_direction.GetLengthSqr() < 0.001f * 0.001f)
	{
		Vec3 ab = pos_b - pos_a;

		float radius = sphere_a->_Radius + sphere_b->_Radius + 0.001f;
		if (ab.GetLengthSqr() > radius * radius)
		{
			return false;
		}
	}

	else if (!RaySphere(pos_a, ray_direction, pos_b, sphere_a->_Radius + sphere_b->_Radius, t0, t1))
	{
		return false;
	}

	t0 *= dt;
	t1 *= dt;

	if (t1 < 0.0f)
	{
		return false;
	}

	toi = (t0 < 0.0f) ? 0.0f : t0;

	if (toi > dt)
	{
		return false;
	}

	Vec3 newPosA = pos_a + vel_a * toi;
	Vec3 newPosB = pos_b + vel_b * toi;
	Vec3 ab = newPosB - newPosA;

	ab.Normalize();

	pt_On_A = newPosA + ab * sphere_a->_Radius;
	pt_On_B = newPosB - ab * sphere_b->_Radius;

	return true;
}


//To Calculate The Distance Between Two Spheres ,We Simply  Check For Overlap Beteen Two Points, these two points will be the positions of the two objects
//We then compare tween the sum of the two radii , if the distance is less than the two radii , there is intersection.
bool Intersect(Body* A, Body* B, float dt, Contact& contact)
{
	//Call Reference To Contact , Since We Will Individually store the contact data of both objects, this will be later used to resolve any collisions detected on the individual object.
	//Hence why we store and refer to this data system within interesection testing.

	contact._BodyA = A;
	contact._BodyB = B;

	contact._TimeOfImpact = 0.0f;

	if (A->_Shape->GetType() == Shape::SHAPE_SPHERE && B->_Shape->GetType() == Shape::SHAPE_SPHERE)
	{
		ShapeSphere* a_sphere = (ShapeSphere*)A->_Shape;
		ShapeSphere* b_sphere = (ShapeSphere*)B->_Shape;

		Vec3 pos_a = A->_Position;
		Vec3 pos_b = B->_Position;


		Vec3 velA = A->_LinearVelocity;
		Vec3 velB = B->_LinearVelocity;

		if (SphereSphereDynamic(a_sphere, b_sphere, pos_a, pos_b, velA, velB, dt, contact.ptOnA_WorldSpace, contact.ptOnB_WorldSpace, contact._TimeOfImpact))
		{
			//Step bodies forward to get local space collision points
			A->Update(contact._TimeOfImpact);
			B->Update(contact._TimeOfImpact);

			//Convert world space contacts to local space
			contact.ptOnA_LocalSpace = A->WorldSpaceToBodySpace(contact.ptOnA_WorldSpace);
			contact.ptOnB_LocalSpace = B->WorldSpaceToBodySpace(contact.ptOnB_WorldSpace);

			contact.Normal = A->_Position - B->_Position;
			contact.Normal.Normalize();

			//Unwind timestep
			A->Update(-contact._TimeOfImpact);
			B->Update(-contact._TimeOfImpact);

			Vec3 ab = B->_Position - A->_Position;
			float r = ab.GetMagnitude() - (a_sphere->_Radius + b_sphere->_Radius);

			contact._SeperationDistance = r;

			return true;
		}

	}
	//use GJK to perform conservative advance
	else
	{
		bool result = ConservativeAdvance(A, B, dt, contact);
		return result;
	}
	return false;
}

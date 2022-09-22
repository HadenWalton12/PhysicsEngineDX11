#include "ShapeConvex.h"

//Build function is simple , passing an array of points and store an array of points
//However with a convex , we should store points on the surface of the convex hull.
//To do this , we have to build up connections of the points , any points are found to be inside the convex hull will be discared.
//We can do this by building  simplx and then expand it out to include all points
//are outside of it.
void ShapeConvex::Build(const Vec3* points, const int num)
{
}

//Same method within the ShapeBox function
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

//Same Method In ShapeBox
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

//To Build A simplex , we do the following steps

//1. Find a point that is furthest in a particular direction (this is deemed the support point)

//2. Find another point that is furthest in the opposite direction (based on the previous point above)

//3. Find a third point that is furthest from the axis of points from previous two points above/ creates a line.

//4. We find the point that is furthest from the plane formed from previous points

//5. With all points,  we connect all these points, this forms a indice list.

/*
	Find Point Furthest In Direction
	//Step One
*/
int FindPointFurthestInDirection(const Vec3* points, const int num, const Vec3& direction)
{
	//Used to find the furthest point during the for loop sequeance
	int max_id = 0;
	float max_distance = direction.Dot(points[0]);
	for (int i = 0; i < num; i++)
	{
		//Using dot product, we get new vector/point , will be used to compare against max distance
		float distance = direction.Dot(points[i]);
		if (distance > max_distance)
		{
			max_distance = distance;
			max_id = i;
		}
	}

	//Returns  ID which element has the highest direction in a group of points.
	return max_id;
}

/*
	Distance From Line
	//Creates a line from two points , then compares a convex hull point , returns a point that gives the point distant from the line calculated
	//This is expressed as the "perpendicular" vec3, since the point returning is the byproduct of 
*/
float DistanceFromLine(const Vec3& a, const Vec3& b, const Vec3& point)
{
	//Difference between two vectors
	Vec3 ab = b - a;
	//Normalized, for assumption using it for its vector direction
	ab.Normalize();

	//Ray difference between point and comparaed line point a
	//Doing this gives us the distance from one point from the line, hence why we then calculate the projection ray , to "project ray onto the line vector
	Vec3 ray = point - a;
	//Projects the ray onto the directional vector
	Vec3 projection = ab * ray.Dot(ab);

	//We create a vector 90 degree/ perpendicular to a vector from the byproduct of the ray and projection.
	Vec3 perpendicular = ray - projection;

	//Returns value that is distance from ab line
	return perpendicular.GetMagnitude();
}

/*
	Find Point Furthest From Line
	//Use function above for utility of this function
	//THis simply iterates using the function above to via this function
*/
Vec3 FindPointFurthestFromLine(const Vec3 * points, const int num , const Vec3 & point_a , const Vec3 & point_b)
{
	int max_id = 0;

	//Initial Max Distance

	float max_distance = DistanceFromLine(point_a, point_b, points[0]);
	
	for (int i = 1; i < num; i++)
	{
	
		float distance = DistanceFromLine(point_a, point_b, points[0]);

		if (distance > max_distance)
		{
			max_distance = distance;
			max_id;
		}

	}

	return points[max_id];
}

/*
	Distance From Triangle
	From 3 given points , to further create a convex hull

*/
float DistanceFromTriangle(const Vec3& a, const Vec3& b, const Vec3& c, const Vec3& point)
{
	//Giving Difference between points
	Vec3 ab = b - a;
	Vec3 ac = c - a;

	//Gives new normal vector , will be directional vector to calculate distance
	Vec3 normal = ab.Cross(ac);
	normal.Normalize();

	//Projects the ray onto the directional vector
	Vec3 ray = point - a;

	//Returns/Calculate distance 
	float distance = ray.Dot(normal);
	return distance;
}

/*
	FindPointFurthestFromTriangle

*/

Vec3 FindPointFurthestFromTriangle(const Vec3* points, const int num, const Vec3& point_a, const Vec3& point_b, const Vec3& point_c)
{
	int max_id = 0;

	//Initial Value
	float max_distance = DistanceFromTriangle(point_a , point_b ,point_c , points[0]);

	for (int i = 1; i < num; i++)
	{
		float distance = DistanceFromTriangle(point_a, point_b, point_c, points[i]);

		if (distance * distance > max_distance * max_distance)
		{
			max_distance = distance;
			max_id = i;

		}
	}
	return points[max_id];
}
struct tri_t
{
	int a;
	int b;
	int c;

};

/*
	BuildTetrahedron

	Utilising all functions above, we use these functions to play out each step.
*/
void BuildTetrahedron(const Vec3 * verticies , const int num , std::vector<Vec3> & hull_points , std::vector<tri_t> & hull_triangles)
{
	hull_points.clear();
	hull_triangles.clear();

	Vec3 points[4];

	//Step 1
	int id = FindPointFurthestInDirection(verticies, num, Vec3(1.0f, 0.0f, 0.0f));
	
	//First point of new tetrahedron
	points[0] = verticies[id];
	
	//Step 2
	id = FindPointFurthestInDirection(verticies, num, points[0] * -1.0f);
	
	//Build Triangle
	points[1] = verticies[id];

	//Step 3
	points[2] = FindPointFurthestFromLine(verticies, num, points[0], points[1]);
	
	//Step 4
	points[3] = FindPointFurthestFromTriangle(verticies, num, points[0], points[1], points[2]);


	//This Is important for making sure the ordering is CCW for all faces.
	float distance = DistanceFromTriangle(points[0], points[1], points[2], points[3]);
	
	if (distance > 0.0f)
	{
		std::swap(points[0], points[1]);

	}

	//step 5
	// 
	//Build the tetrahedron
	hull_points.push_back(points[0]);
	hull_points.push_back(points[1]);
	hull_points.push_back(points[2]);
	hull_points.push_back(points[3]);


	//Creating Draw Order
	tri_t triangle;

	triangle.a = 0;
	triangle.b = 1;
	triangle.c = 2;
	
	hull_triangles.push_back(triangle);

	triangle.a = 0;
	triangle.b = 2;
	triangle.c = 3;

	hull_triangles.push_back(triangle);

	triangle.a = 2;
	triangle.b = 1;
	triangle.c = 3;

	hull_triangles.push_back(triangle);

	triangle.a = 1;
	triangle.b = 0;
	triangle.c = 3;

	hull_triangles.push_back(triangle);

}

//Above are the main functions to build a tetrahadron , we need to make sure each face of the tetraheadron is counter clock wise , 
// hence we need to expand out , to include other points that can then further define the surface of the shape (we have to create face/surface normals)
// The outline of steps to do this will to expand the tetrahedron to the convex hull

//Step 1 Remove any internal points -
// THere may be points isnide the tetrahedron , we no longer need them since we only want the points that define the surface of the convex hull

//Step 2 - Then we choose any point that is still external to the hull , and find the point furthest oi that direction

//Step 3 - Find the dangling edges use them to create 
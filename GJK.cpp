#include "GJK.h"

//Signed Volumes - Prior to learning GJK algorithm , a predeccessor algorithm not utilised anymore is signed volume algorithm.
//In order for GJK to work , it needs the projection of origins to the simplexes we use, The reason for this since GJK needs to know
//the shortest distance/direction to an origin for any given simplex

//Simplex - In geometry a simplex is a generalized term for the notion of a triangle , tetrahedron to arbitrary dimensions , it is named so 
//because it simplex represents the simplest possible polytope made with line segments , meaning the line segment based shapes 
// that are the simplest shapes that can be created are considered simplexes (polytope is polygonal shapes , with flate sides/faces that
//are also 3D)
//The reason we need simplexes anyway is to know the shortest distance and direction to the origin from all simplexes. Hence why we are creating
//signed volume functions , that will allow us to find out these values in accordance to the GJK algorithm.
// 1 - Line Segment 
// 2 - Triangle 
// 3 - Tetragedron
//The signed volume approach is to find which axis-alligned plane maxmizes the projected area or length of a simplex. E.G
//Lets say we have a 1-simplex / line segment , we project this onto the x, y and z axis , we then determine
//which projection on each axis has the greatest length. We choose that axis , then project our point onto it as well
// this given we can calcyulate the barycentric coordinates of the point onto 1-simplex. giving us the project of the point onto the simplex.

Vec2 SignedVolume1D(const Vec3& s1, const Vec3& s2)
{
	//Calculated difference between both points , ray from a - b
	Vec3 ab = s2 - s1;

	//Calculated , ray from a  to origin
	Vec3 ap = Vec3(0.0f) - s1;

	//Projection of origin onto the line
	Vec3 p0 = s1 + ab * ab.Dot(ap) / ab.GetLengthSqr();


	//Choosing the axis with greatest length/distance

	int id = 0;
	float mu_max = 0;
	for (int i = 0; i < 3; i++)
	{
		//Current ray distance , used in condition
		float mu = s2[i] - s1[i];
		if (mu * mu > mu_max * mu_max)
		{
			//Give us line segment / simple with greatest length/distance
			mu_max = mu;
			id = i;

		}
	}

	//Project the simplex points and projected origin onto the "axis" with the greastest length

	const float a = s1[id];
	const float b = s2[id];
	const float p = p0[id];

	//Get the signed distance from a to p and from p to b
	const float C1 = p - a;
	const float C2 = b - p;

	//Checking between if p is between a or b
	if ((p > a && p < b) || (p > b && p < a))
	{
		//lambdas - anomynous function
		Vec2 lambdas;
		lambdas[0] = C2 / mu_max;
		lambdas[1] = C1 / mu_max;
		return lambdas;
	}

	//if p is on the far side of a, we return the following
	if ((a <= b && p <= a) || (a >= b && p >= a))
	{
		return Vec2(1.0f, 0.0f);
	}

	//p must be on far side of b
	return Vec2(0.0f, 1.0f);
}

//For 2-simplex / triangle , we do the same thing , however only we project the triangle onto the xy . yz and zx plane. we then 
//determine which one has the greatest area. then projecting that point onto that plane
//if the point is inside the traingle, we calculate the barycentric coordinates and be done , if not we then project the point onto the edges of
//of the triangle using 1-simplex triangle projection method , and ise the closest projection

int CompareSigns(float a, float b)
{
	if (a > 0.0f && b > 0.0f)
	{
		return 1;
	}
	if (a < 0.0f && b < 0.0f)
	{
		return 1;
	}
	return 0;
}

Vec3 SignedVolume2D(const Vec3& s1, const Vec3& s2, const Vec3& s3)
{
	Vec3 normal = (s2 - s1).Cross(s3 - s1);

	Vec3 p0 = normal * s1.Dot(normal) / normal.GetLengthSqr();

	//Find the axis with the greatest projected area
	int id = 0;
	float max_area = 0;

	for (int i = 0; i < 3; i++)
	{
		int j = (i + 1) % 3;
		int k = (i + 2) % 3;

		Vec2 a = Vec2(s1[j], s1[k]);
		Vec2 b = Vec2(s2[j], s2[k]);
		Vec2 c = Vec2(s3[j], s3[k]);

		Vec2 ab = b - a;
		Vec2 ac = c - a;

		float area = ab.x * ac.y - ab.y * ac.x;
		if (area * area > max_area * max_area)
		{
			id = 0;
			max_area = area;
		}
	}

	//Project onto the appropriate axis
	int x = (id + 1) % 3;
	int y = (id + 2) % 3;

	Vec2 s[3];
	s[0] = Vec2(s1[x], s1[y]);
	s[1] = Vec2(s2[x], s2[y]);
	s[2] = Vec2(s3[x], s3[y]);

	Vec2 p = Vec2(p0[x], p0[y]);

	//Get the sub-areas of the triangles formed from the projected origin and edges
	Vec3 areas;
	for (int i = 0; i < 3; i++)
	{
		int j = (i + 1) % 3;
		int k = (i + 2) % 3;

		Vec2 a = p;
		Vec2 b = s[j];
		Vec2 c = s[k];

		Vec2 ab = b - a;
		Vec2 ac = c - a;

		//Calculating all the areas
		areas[i] = ab.x * ac.y - ab.y * ac.x;
	}

	//If the projected origin is inside the triangle , then returns the barycentric points
	if (CompareSigns(max_area, areas[0]) > 0 && CompareSigns(max_area, areas[1]) > 0 && CompareSigns(max_area, areas[2]) > 0)
	{
		Vec3 lambdas = areas / max_area;
		return lambdas;


	}

	//If previous condition is false , we need to project onto the egdes and determine
	//the closest point to the edges

	float distance = 1e10;
	Vec3 lambdas = Vec3(1.0f, 0.0f, 0.0f);

	for (int i = 0; i < 3; i++)
	{
		int k = (i + 1) % 3;
		int I = (i + 2) % 3;

		Vec3 edges_points[3];
		edges_points[0] = s1;
		edges_points[1] = s2;
		edges_points[2] = s3;

		Vec2 lambda_edge = SignedVolume1D(edges_points[k], edges_points[I]);
		Vec3 point = edges_points[k] * lambda_edge[0] + edges_points[I] * lambda_edge[1];

		if (point.GetLengthSqr() < distance)
		{
			distance = point.GetLengthSqr();
			lambdas[i] = 0;
			lambdas[k] = lambda_edge[0];
			lambdas[I] = lambda_edge[1];
		}
	}
	return lambdas;

}

//Final 3-simplex / tetrahedron function to calculate its barycentric coordinates ,if it isnt
//check each face for which projection is closest

Vec4 SignedVolume3D(const Vec3& s1, const Vec3& s2, const Vec3& s3, const Vec3& s4)
{
	Mat4 matrix;
	matrix.rows[0] = Vec4(s1.x, s2.x, s3.x, s4.x);
	matrix.rows[1] = Vec4(s1.y, s2.y, s3.y, s4.y);
	matrix.rows[2] = Vec4(s1.z, s2.z, s3.z, s4.z);
	matrix.rows[3] = Vec4(1.0f, 1.0f, 1.0f, 1.0f);

	Vec4 C4;
	C4[0] = matrix.Cofactor(3, 0);
	C4[1] = matrix.Cofactor(3, 1);
	C4[2] = matrix.Cofactor(3, 2);
	C4[3] = matrix.Cofactor(3, 3);

	const float determinant_matrix = C4[0] + C4[1] + C4[2] + C4[3];

	//If the barycentric coordinates put the origin inside the simplex, then return them
	if (CompareSigns(determinant_matrix, C4[0]) > 0 && CompareSigns(determinant_matrix, C4[1]) > 0
		&& CompareSigns(determinant_matrix, C4[2]) > 0 && CompareSigns(determinant_matrix, C4[3]) > 0)
	{
		Vec4 lambdas = C4 * (1.0f / determinant_matrix);
		return lambdas;
	}

	//IF we get here , we need to project the origin onto the faces and determine
	// the closest one
	Vec4 lambdas;
	float distance = 1e10;
	for (int i = 0; i < 4; i++)
	{
		int j = (i + 1) % 4;
		int k = (i + 2) % 4;

		Vec3  face_points[4];
		face_points[0] = s1;
		face_points[1] = s2;
		face_points[2] = s3;
		face_points[3] = s4;

		Vec3 lambdas_face = SignedVolume2D(face_points[i], face_points[j], face_points[k]);
		Vec3 point = face_points[i] * lambdas_face[0] + face_points[j] * lambdas_face[1] + face_points[k]
			* lambdas_face[2];

		if (point.GetLengthSqr() < distance)
		{
			distance = point.GetLengthSqr();
			lambdas[i] = lambdas_face[0];
			lambdas[j] = lambdas_face[1];
			lambdas[k] = lambdas_face[2];
		}
	}

	return lambdas;
}

//Test Utility functions will be used to make sure the functions work as expected, projecting a point onto a simplex and return
//barycentric coordinates of this stated projection.
void TestSignedVolumeProjection()
{
	const Vec3 original_points[4] =
	{
		Vec3(0.0f , 0.0f , 0.0f) ,
		Vec3(1.0f , 0.0f , 0.0f) ,
		Vec3(0.0f , 1.0f , 0.0f) ,
		Vec3(0.0f , 0.0f , 1.0f) ,
	};
	Vec3 points[4];
	Vec4 lambdas;
	Vec3 v;

	for (int i = 0; i < 4; i++)
	{
		points[i] = original_points[i] + Vec3(1.0f, 1.0f, 1.0f);
	}

	lambdas = SignedVolume3D(points[0], points[1], points[2], points[3]);
	v.Zero();

	for (int i = 0; i < 4; i++)
	{
		v += points[i] * lambdas[i];
	}

	//print function here


	for (int i = 0; i < 4; i++)
	{
		points[i] = original_points[i] + Vec3(-1.0f, -1.0f, -1.0f) * 0.25f;

	}
	lambdas = SignedVolume3D(points[0], points[1], points[2], points[3]);
	v.Zero();
	for (int i = 0; i < 4; i++)
	{
		v += points[i] * lambdas[i];
	}

	for (int i = 0; i < 4; i++)
	{
		points[i] = original_points[i] + Vec3(-1.0f, -1.0f, -1.0f);

	}
}



/*
	SimplexSignedVolumes

	Projects the orign onto the simplex onto the simplex to acquire the new search direction , also checks if the origin is inside the simplex.
*/
bool SimplexSignedVolumes(point_t* points, const int num, Vec3& new_direction, Vec4 lambdas_out)
{
	const float epsilonf = 0.0001f * 0.0001f;
	lambdas_out.Zero();

	bool does_intersect = false;

	switch (num)
	{
	case 2:
	{
		Vec2 lambdas = SignedVolume1D(points[0].xyz, points[1].xyz);
		Vec3 v(0.0f);

		for (int i = 0; i < 2; i++)
		{
			v += points[i].xyz * lambdas[i];
		}
		new_direction = v * -1.0f;
		does_intersect = (v.GetLengthSqr() < epsilonf);

		lambdas_out[0] = lambdas[0];
		lambdas_out[1] = lambdas[1];
	}break;
	case 3:
	{
		Vec3 lambdas = SignedVolume2D(points[0].xyz, points[1].xyz, points[2].xyz);
		Vec3 v(0.0f);

		for (int i = 0; i < 3; i++)
		{
			v += points[i].xyz * lambdas[i];
		}
		new_direction = v * -1.0f;
		does_intersect = (v.GetLengthSqr() < epsilonf);

		lambdas_out[0] = lambdas[0];
		lambdas_out[1] = lambdas[1];
		lambdas_out[2] = lambdas[2];
	}break;
	case 4:
	{
		Vec4 lambdas = SignedVolume3D(points[0].xyz, points[1].xyz, points[2].xyz, points[3].xyz);
		Vec3 v(0.0f);

		for (int i = 0; i < 4; i++)
		{
			v += points[i].xyz * lambdas[i];
		}
		new_direction = v * -1.0f;
		does_intersect = (v.GetLengthSqr() < epsilonf);

		lambdas_out[0] = lambdas[0];
		lambdas_out[1] = lambdas[1];
		lambdas_out[2] = lambdas[2];
		lambdas_out[3] = lambdas[3];
	}break;


	}
	return does_intersect;

}

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



/*
HasPoint

Checks Whether the new point alreadt exists in the simplex
*/
bool HasPoint(const point_t simplex_points[4], const point_t& new_point)
{
	const float precision = 1e-6f;

	for (int i = 0; i < 4; i++)
	{
		Vec3 delta = simplex_points[i].xyz - new_point.xyz;

		if (delta.GetLengthSqr() < precision * precision)
		{
			return true;
		}
	}
	return false;

}


/*
	SortValids
	Sorts the valid support points to the beggining of an array
*/
void SortValids(point_t simplex_points[4], Vec4& lambdas)
{
	bool valids[4];

	for (int i = 0; i < 4; i++)
	{
		valids[i] = true;

		if (lambdas[i] == 0.0f)
		{
			valids[i] = false;
		}
	}

	Vec4 valid_lambdas(0.0f);
	int valid_count = 0;
	point_t valid_points[4];

	memset(valid_points, 0, sizeof(point_t) * 4);
	for (int i = 0; i < 4; i++)
	{
		if (valids[i])
		{
			valid_points[valid_count] = simplex_points[i];
			valid_lambdas[valid_count] = lambdas[i];
			valid_count++;
		}
	}

	//Copy the valids back into simplex points
	for (int i = 0; i < 4; i++)
	{
		simplex_points[i] = valid_points[i];
		lambdas[i] = valid_lambdas[i];
	}


}


/*
	NumValids
*/

static int NumValids(const Vec4& lambdas)
{
	int num = 0;
	for (int i = 0; i < 4; i++)
	{
		if (0.0f != lambdas[i])
		{
			num++;
		}
	}
	return num;
}

/*
	GJK_DoesIntersect
*/
bool GJK_DoesIntersect(const Body* bodyA, const Body* bodyB)
{
	const Vec3 origin(0.0f);

	int num_points = 1;

	point_t simplex_points[4];

	simplex_points[0] = Support(bodyA, bodyB, Vec3(1.0f, 1.0f, 1.0f), 0.0f);

	float closest_distance = 1e10f;

	bool does_contain_origin = false;

	Vec3 new_direction = simplex_points[0].xyz * -1.0f;

	do
	{
	  //Get The new point check on
		point_t new_point = Support(bodyA , bodyB , Vec3(1.0f , 1.0f , 1.0f) , 0.0f);

		//If the new point is the same as a previous point , then we cant expand any further
		if (HasPoint(simplex_points , new_point))
		{
			break;
		}
		simplex_points[num_points] = new_point;
		num_points++;

		//If this new point hasnt been moved passed the porigin , then the origin cannot be i the set, therefore there is no collision
		float dotdot = new_direction.Dot(new_point.xyz - origin);
		if (dotdot < 0.0f)
		{
			break;
		}

		Vec4 lambdas;

		does_contain_origin = SimplexSignedVolumes(simplex_points, num_points, new_direction, lambdas);
		if (does_contain_origin)
		{
			break;
		}


		//Check that the new projection of the prigin onto the simplex is closer than the previous
		float distance = new_direction.GetLengthSqr();

		if (distance >= closest_distance)
		{
			break;
		}
		closest_distance = distance;

		//Use the lambdas that support new search direction , and invalidate anuy points that dont support it
		SortValids(simplex_points, lambdas);
		num_points = NumValids(lambdas);
		does_contain_origin = (4 == num_points);
	} while (!does_contain_origin);

	return does_contain_origin;
}





/*
	Barycentric Coordinates
	This borrows our signed volume code to perform the barycentric coordinates
*/
Vec3 BarycentricCoordinates(Vec3 s1, Vec3 s2, Vec3 s3, const Vec3& point)
{
	s1 = s1 - point;
	s2 = s2 - point;
	s3 = s3 - point;

	Vec3 normal = (s2 - s1).Cross(s3 - s1);
	Vec3 point0 = normal * s1.Dot(normal) / normal.GetLengthSqr();

	//Find The Axis With The Greatests Projected Area
	int idx = 0;
	float area_max = 0;

	for (int i = 0; i < 3 ; i++)
	{
		int j = (i + 1) % 3;
		int k = (i + 2) % 3;

		Vec2 a = Vec2(s1[j], s1[k]);
		Vec2 b = Vec2(s2[j], s2[k]);
		Vec2 c = Vec2(s3[j], s3[k]);

		Vec2 ab = b - a;
		Vec2 ac = c - a;

		float area = ab.x * ac.y - ab.y * ac.x;

		if (area * area > area_max * area_max)
		{
			idx = i;
			area_max = area;
		}
	}

	//Project onto the appropriate axis
	int x = (idx + 1) % 3;
	int y = (idx + 2) % 3;

	Vec2 s[3];

	s[0] = Vec2(s1[x], s1[y]);
	s[1] = Vec2(s2[x], s2[y]);
	s[2] = Vec2(s3[x], s3[y]);
	Vec2 p = Vec2(point0[x], point0[y]);

	//Get the sub-areas of the triangle formed from the project origin and the edges
	Vec3 areas;

	for (int i = 0; i < 3; i++)
	{
		int j = (i + 1) % 3;
		int k = (i + 1) % 3;
	
		Vec2 a = p;
		Vec2 b = s[j];
		Vec2 c = s[k];

		Vec2 ab = b - a;
		Vec2 ac = c - a;
	
		areas[i] = ab.x * ac.y - ab.y * ac.x;

	}

	Vec3 lambdas = areas / area_max;

	if (!lambdas.IsValid())
	{
		lambdas = Vec3(1.0f, 0.0f, 0.0f);
	}

	return lambdas;
}

Vec3 NormalDirection(const tri_t tri, const std::vector<point_t> & points)
{
	const Vec3& a = points[tri.a].xyz;
	const Vec3& b = points[tri.b].xyz;
	const Vec3& c = points[tri.c].xyz;

	Vec3 ab = b - a;
	Vec3 ac = c - a;

	Vec3 normal = ab.Cross(ac);
	normal.Normalize();

	return normal;
}

/*
	Signed Distance To Triangle

*/
float SignedDistanceToTriangle(const tri_t & triangle , const Vec3 & point , const std::vector<point_t>& points)
{
	const Vec3 normal = NormalDirection(triangle, points);
	const Vec3& a = points[triangle.a].xyz;
	const Vec3 a2_point = point - a;
	const float distance = normal.Dot(a2_point);

	return distance;
}


/*
	Closest Triangle
*/
int ClosestTriangle(const std::vector<tri_t>& triangles, const std::vector<point_t>& points)
{
	float min_distance_square = 1e10;

	int idx = -1;

	for (int i = 0; i < triangles.size(); i++)
	{
		const tri_t triangle = triangles[i];

		float distance = SignedDistanceToTriangle(triangle, Vec3(0.0f), points);
		float distance_square = distance * distance;
		if (distance_square < min_distance_square )
		{
			idx = i;
			min_distance_square = distance_square;
		}
	}
	return idx;
}

/*
	HasPoint
*/

bool HasPoint(const Vec3 & w , const std::vector<tri_t> triangles , const std::vector<point_t> & points)
{
	const float epsilons = 0.001f * 0.001f;
	Vec3 delta;

	for (int i = 0; i < triangles.size(); i++)
	{
		const tri_t& triangle = triangles[i];

		delta = w - points[triangle.a].xyz;
		if (delta.GetLengthSqr() < epsilons)
		{
			return true;
		}

		delta = w - points[triangle.b].xyz;
		if (delta.GetLengthSqr() < epsilons)
		{
			return true;
		}
	
		delta = w - points[triangle.c].xyz;
		if (delta.GetLengthSqr() < epsilons)
		{
			return true;
		}
	
	}
	
	return false;
}

/*
	Remove TrianglesFacingPoint
*/


int RemoveTrianglesFacingPoint(const Vec3& point, std::vector<tri_t>& triangles, const std::vector<point_t>& points)
{

	int num_removed = 0;

	for (int i = 0; i < triangles.size(); i++)
	{
		const tri_t& triangle = triangles[i];

		float distance = SignedDistanceToTriangle(triangle, point, points);

		if (distance > 0.0f)
		{
			//This triangle faces the point , remove it
			triangles.erase(triangles.begin() + i);
			i--;
			num_removed++;
		}
	}

	return num_removed;
}

/*
	FindDanglingEdges
*/
void FindDanglingEdges(std::vector<edge_t>& dangling_edges , const std::vector<tri_t> & triangles)
{
	dangling_edges.clear();

	for (int i = 0; i < triangles.size(); i++)
	{
		const tri_t& triangle = triangles[i];

		edge_t edges[3];
		edges[0].a = triangle.a;
		edges[0].b = triangle.b;

		edges[1].a = triangle.a;
		edges[1].b = triangle.b;

		edges[2].a = triangle.a;
		edges[2].b = triangle.b;

		int counts[3];

		counts[0] = 0;
		counts[1] = 0;
		counts[2] = 0;

		for (int j = 0; j < triangles.size(); j++)
		{
			if (j == i)
			{
				continue;
			}

			const tri_t & triangle_2 = triangles[j];

			edge_t edges_2[3];
			edges_2[0].a = triangle_2.a;
			edges_2[0].b = triangle_2.b;

			edges_2[1].a = triangle_2.a;
			edges_2[1].b = triangle_2.b;

			edges_2[2].a = triangle_2.a;
			edges_2[2].b = triangle_2.b;

			for (int k = 0; k < 3; k++)
			{
				if(edges[k] == edges_2[0])
				{ 
					counts[k]++;
				}
				if (edges[k] == edges_2[1])
				{
					counts[k]++;
				}
				if (edges[k] == edges_2[2])
				{
					counts[k]++;
				}
			}
		}

		//AN Edge That Isnt shared, it is dangling
		for (int k = 0; k < 3; k++)
		{
			if (0 == counts[k])
			{
				dangling_edges.push_back(edges[k]);
			}
		}
	}


}

/*
	EPA_Expand
*/
float EPA_Expand(const Body* body_a , const Body* body_b , const float bias , const point_t simplex_points[4] , Vec3 & point_on_a , Vec3 & point_on_b)
{
	std::vector<point_t> points;
	std::vector<tri_t> triangles;
	std::vector<edge_t> dangling_edges;

	Vec3 centre(0.0f);

	for (int i = 0; i < 4; i++)
	{
		points.push_back(simplex_points[i]);
		centre += simplex_points[i].xyz;
	}
	
	centre *= 0.25f;

	//Build The Triangles
	for (int i = 0; i < 4; i++)
	{
		int j = (i + 1) % 4;
		int k = (i + 2) % 4;

		tri_t triangle;
		
		triangle.a = i;
		triangle.b = i;
		triangle.c = i;

		int unused_point = (i + 3) % 4;

		float distance = SignedDistanceToTriangle(triangle, points[unused_point].xyz, points);

		//The unused points is always on the negative/inside of the triangle
		//make sure the normal points away
		if (distance > 0.0f)
		{
			std::swap(triangle.a, triangle.b);
		}

		triangles.push_back(triangle);
 	}

	//
	//Expand the simplex to find the closest face of the CSO to the origin
	//
	while (1)
	{
		const int idx = ClosestTriangle(triangles, points);
		Vec3 normal = NormalDirection(triangles[idx], points);

		const point_t new_point = Support(body_a, body_b, normal, bias);
		
		//If w already exists, we just stop, we dont have to any further
		if ()
		{

		}
	};
}
#include "Model.h"
#include "Shape.h"
#include "ShapeBox.h"
#include "ShapeSphere.h"
#include "ShapeConvex.h"
Model::Model()
{
}
Model::~Model()
{
}


/*
====================================================
FloatToByte
// Assumes a float between [-1,1]
====================================================
*/
unsigned char FloatToByte_n11(const float f) {
	int i = (int)(f * 127 + 128);
	return (unsigned char)i;
}
/*
====================================================
FloatToByte
// Assumes a float between [0,1]
====================================================
*/
unsigned char FloatToByte_01(const float f) {
	int i = (int)(f * 255);
	return (unsigned char)i;
}

Vec3 Byte4ToVec3(const unsigned char* data) {
	Vec3 pt;
	pt.x = float(data[0]) / 255.0f;	// 0,1
	pt.y = float(data[1]) / 255.0f;	// 0,1
	pt.z = float(data[2]) / 255.0f;	// 0,1

	pt.x = 2.0f * (pt.x - 0.5f); //-1,1
	pt.y = 2.0f * (pt.y - 0.5f); //-1,1
	pt.z = 2.0f * (pt.z - 0.5f); //-1,1
	return pt;
}

void Vec3ToFloat3(const Vec3& v, float* f) {
	f[0] = v.x;
	f[1] = v.y;
	f[2] = v.z;
}
void Vec2ToFloat2(const Vec2& v, float* f) {
	f[0] = v.x;
	f[1] = v.y;
}
void Vec3ToByte4(const Vec3& v, unsigned char* b) {
	Vec3 tmp = v;
	tmp.Normalize();
	b[0] = FloatToByte_n11(tmp.x);
	b[1] = FloatToByte_n11(tmp.y);
	b[2] = FloatToByte_n11(tmp.z);
	b[3] = 0;
}

/*
====================================================
FillFullScreenQuad
====================================================
*/
void FillFullScreenQuad(Model& model) {
	const int numVerts = 4;
	const int numIdxs = 6;
	NewVertex	screenVerts[numVerts];
	int		screenIndices[numIdxs];

	memset(screenVerts, 0, sizeof(NewVertex) * 4);

	screenVerts[0].Pos[0] = -1.0f;
	screenVerts[0].Pos[1] = -1.0f;
	screenVerts[0].Pos[2] = 0.0f;

	screenVerts[1].Pos[0] = 1.0f;
	screenVerts[1].Pos[1] = -1.0f;
	screenVerts[1].Pos[2] = 0.0f;

	screenVerts[2].Pos[0] = 1.0f;
	screenVerts[2].Pos[1] = 1.0f;
	screenVerts[2].Pos[2] = 0.0f;

	screenVerts[3].Pos[0] = -1.0f;
	screenVerts[3].Pos[1] = 1.0f;
	screenVerts[3].Pos[2] = 0.0f;


	screenVerts[0].st[0] = 0.0f;
	screenVerts[0].st[1] = 1.0f;

	screenVerts[1].st[0] = 1.0f;
	screenVerts[1].st[1] = 1.0f;

	screenVerts[2].st[0] = 1.0f;
	screenVerts[2].st[1] = 0.0f;

	screenVerts[3].st[0] = 0.0f;
	screenVerts[3].st[1] = 0.0f;

	screenVerts[0].buff[0] = 255;
	screenVerts[1].buff[0] = 255;
	screenVerts[2].buff[0] = 255;
	screenVerts[3].buff[0] = 255;


	screenIndices[0] = 0;
	screenIndices[1] = 1;
	screenIndices[2] = 2;

	screenIndices[3] = 0;
	screenIndices[4] = 2;
	screenIndices[5] = 3;

	for (int i = 0; i < numVerts; i++) {
		model._Vertices.push_back(screenVerts[i]);
	}

	for (int i = 0; i < numIdxs; i++) {
		model._Indices.push_back(screenIndices[i]);
	}
}

/*
====================================================
FillCube
====================================================
*/
void FillCube(Model& model) {
	const int numIdxs = 3 * 2 * 6;
	const int numVerts = 4 * 6;
	NewVertex	cubeVerts[numVerts];
	int		cubeIdxs[numIdxs];

	memset(cubeVerts, 0, sizeof(NewVertex) * 4 * 6);

	for (int face = 0; face < 6; face++) {
		const int dim0 = face / 2;
		const int dim1 = (dim0 + 1) % 3;
		const int dim2 = (dim0 + 2) % 3;
		const float val = ((face & 1) == 0) ? -1.0f : 1.0f;

		cubeVerts[face * 4 + 0].Pos[dim0] = val;
		cubeVerts[face * 4 + 0].Pos[dim1] = val;
		cubeVerts[face * 4 + 0].Pos[dim2] = val;

		cubeVerts[face * 4 + 1].Pos[dim0] = val;
		cubeVerts[face * 4 + 1].Pos[dim1] = -val;
		cubeVerts[face * 4 + 1].Pos[dim2] = val;

		cubeVerts[face * 4 + 2].Pos[dim0] = val;
		cubeVerts[face * 4 + 2].Pos[dim1] = -val;
		cubeVerts[face * 4 + 2].Pos[dim2] = -val;

		cubeVerts[face * 4 + 3].Pos[dim0] = val;
		cubeVerts[face * 4 + 3].Pos[dim1] = val;
		cubeVerts[face * 4 + 3].Pos[dim2] = -val;


		cubeVerts[face * 4 + 0].st[0] = 0.0f;
		cubeVerts[face * 4 + 0].st[1] = 1.0f;

		cubeVerts[face * 4 + 1].st[0] = 1.0f;
		cubeVerts[face * 4 + 1].st[1] = 1.0f;

		cubeVerts[face * 4 + 2].st[0] = 1.0f;
		cubeVerts[face * 4 + 2].st[1] = 0.0f;

		cubeVerts[face * 4 + 3].st[0] = 0.0f;
		cubeVerts[face * 4 + 3].st[1] = 0.0f;


		cubeVerts[face * 4 + 0].norm[dim0] = FloatToByte_n11(val);
		cubeVerts[face * 4 + 1].norm[dim0] = FloatToByte_n11(val);
		cubeVerts[face * 4 + 2].norm[dim0] = FloatToByte_n11(val);
		cubeVerts[face * 4 + 3].norm[dim0] = FloatToByte_n11(val);


		cubeVerts[face * 4 + 0].tang[dim1] = FloatToByte_n11(val);
		cubeVerts[face * 4 + 1].tang[dim1] = FloatToByte_n11(val);
		cubeVerts[face * 4 + 2].tang[dim1] = FloatToByte_n11(val);
		cubeVerts[face * 4 + 3].tang[dim1] = FloatToByte_n11(val);


		cubeIdxs[face * 6 + 0] = face * 4 + 0;
		cubeIdxs[face * 6 + 1] = face * 4 + 1;
		cubeIdxs[face * 6 + 2] = face * 4 + 2;

		cubeIdxs[face * 6 + 3] = face * 4 + 0;
		cubeIdxs[face * 6 + 4] = face * 4 + 2;
		cubeIdxs[face * 6 + 5] = face * 4 + 3;
	}

	for (int i = 0; i < numVerts; i++) {
		model._Vertices.push_back(cubeVerts[i]);
	}

	for (int i = 0; i < numIdxs; i++) {
		model._Indices.push_back(cubeIdxs[i]);
	}
}
void FillCubeTessellated(Model& model, int numDivisions) {
	if (numDivisions < 1) {
		numDivisions = 1;
	}

	const int numnum = (numDivisions + 1) * (numDivisions + 1);
	Vec3* v = (Vec3*)malloc(numnum * sizeof(Vec3));
	Vec2* st = (Vec2*)malloc(numnum * sizeof(Vec2));
	for (int y = 0; y < numDivisions + 1; y++) {
		for (int x = 0; x < numDivisions + 1; x++) {
			float xf = (((float)x / (float)numDivisions) - 0.5f) * 2.0f;
			float yf = (((float)y / (float)numDivisions) - 0.5f) * 2.0f;
			v[y * (numDivisions + 1) + x] = Vec3(xf, yf, 1.0f);

			float sf = (float)x / (float)numDivisions;
			float tf = (float)y / (float)numDivisions;
			st[y * (numDivisions + 1) + x] = Vec2(sf, tf);
		}
	}

	const int numFaces = numDivisions * numDivisions;

	int faceIdx = 0;
	int* faceIdxs = (int*)malloc(3 * 2 * numFaces * sizeof(int));
	for (int y = 0; y < numDivisions; y++) {
		for (int x = 0; x < numDivisions; x++) {
			int y0 = y;
			int y1 = y + 1;
			int x0 = x;
			int x1 = x + 1;

			faceIdxs[faceIdx * 6 + 1] = y0 * (numDivisions + 1) + x0;
			faceIdxs[faceIdx * 6 + 0] = y1 * (numDivisions + 1) + x0;
			faceIdxs[faceIdx * 6 + 2] = y1 * (numDivisions + 1) + x1;

			faceIdxs[faceIdx * 6 + 4] = y0 * (numDivisions + 1) + x0;
			faceIdxs[faceIdx * 6 + 3] = y1 * (numDivisions + 1) + x1;
			faceIdxs[faceIdx * 6 + 5] = y0 * (numDivisions + 1) + x1;

			faceIdx++;
		}
	}

	Mat3 matOrients[6];
	for (int i = 0; i < 6; i++) {
		matOrients[i].Identity();
	}
	// px
	matOrients[0].rows[0] = Vec3(0.0f, 0.0f, 1.0f);
	matOrients[0].rows[1] = Vec3(1.0f, 0.0f, 0.0f);
	matOrients[0].rows[2] = Vec3(0.0f, 1.0f, 0.0f);
	// nx
	matOrients[1].rows[0] = Vec3(0.0f, 0.0f, -1.0f);
	matOrients[1].rows[1] = Vec3(-1.0f, 0.0f, 0.0f);
	matOrients[1].rows[2] = Vec3(0.0f, 1.0f, 0.0f);

	// py
	matOrients[2].rows[0] = Vec3(1.0f, 0.0f, 0.0f);
	matOrients[2].rows[1] = Vec3(0.0f, 0.0f, 1.0f);
	matOrients[2].rows[2] = Vec3(0.0f, -1.0f, 0.0f);
	// ny
	matOrients[3].rows[0] = Vec3(1.0f, 0.0f, 0.0f);
	matOrients[3].rows[1] = Vec3(0.0f, 0.0f, -1.0f);
	matOrients[3].rows[2] = Vec3(0.0f, 1.0f, 0.0f);

	// pz
	matOrients[4].rows[0] = Vec3(1.0f, 0.0f, 0.0f);
	matOrients[4].rows[1] = Vec3(0.0f, 1.0f, 0.0f);
	matOrients[4].rows[2] = Vec3(0.0f, 0.0f, 1.0f);
	// nz
	matOrients[5].rows[0] = Vec3(-1.0f, 0.0f, 0.0f);
	matOrients[5].rows[1] = Vec3(0.0f, 1.0f, 0.0f);
	matOrients[5].rows[2] = Vec3(0.0f, 0.0f, -1.0f);

	const int numIdxs = 3 * 2 * 6 * numFaces;
	const int numVerts = 4 * 6 * numFaces;
	NewVertex* cubeVerts = (NewVertex*)malloc(numVerts * sizeof(NewVertex));
	int* cubeIdxs = (int*)malloc(numIdxs * sizeof(int));

	memset(cubeVerts, 0, sizeof(NewVertex) * numVerts);

	for (int side = 0; side < 6; side++) {
		const Mat3& mat = matOrients[side];

		const Vec3 tang = mat * Vec3(1.0f, 0.0f, 0.0f);
		const Vec3 norm = mat * Vec3(0.0f, 0.0f, 1.0f);

		for (int vid = 0; vid < numnum; vid++) {
			const Vec3 Pos = mat * v[vid];
			const Vec2 uv = st[vid];

			cubeVerts[side * numnum + vid].Pos[0] = Pos[0];
			cubeVerts[side * numnum + vid].Pos[1] = Pos[1];
			cubeVerts[side * numnum + vid].Pos[2] = Pos[2];

			cubeVerts[side * numnum + vid].st[0] = uv[0];
			cubeVerts[side * numnum + vid].st[1] = uv[1];

			cubeVerts[side * numnum + vid].Normal[0] = FloatToByte_n11(norm[0]);
			cubeVerts[side * numnum + vid].Normal[1] = FloatToByte_n11(norm[1]);
			cubeVerts[side * numnum + vid].norm[2] = FloatToByte_n11(norm[2]);
			cubeVerts[side * numnum + vid].norm[3] = FloatToByte_n11(0.0f);

			cubeVerts[side * numnum + vid].tang[0] = FloatToByte_n11(tang[0]);
			cubeVerts[side * numnum + vid].tang[1] = FloatToByte_n11(tang[1]);
			cubeVerts[side * numnum + vid].tang[2] = FloatToByte_n11(tang[2]);
			cubeVerts[side * numnum + vid].tang[3] = FloatToByte_n11(0.0f);
		}

		for (int idx = 0; idx < 3 * 2 * numFaces; idx++) {
			const int offset = 3 * 2 * numFaces * side;
			cubeIdxs[idx + offset] = faceIdxs[idx] + numnum * side;
		}
	}

	for (int i = 0; i < numVerts; i++) {
		model._Vertices.push_back(cubeVerts[i]);
	}

	for (int i = 0; i < numIdxs; i++) {
		model._Indices.push_back(cubeIdxs[i]);
	}

	free(v);
	free(st);
	free(faceIdxs);
	free(cubeVerts);
	free(cubeIdxs);
}
/*
====================================================
FillSphere
====================================================
*/
void FillSphere(Model& model, const float radius) {
	float t = radius;
	if (t < 0.0f) {
		t = 0.0f;
	}
	if (t > 100.0f) {
		t = 100.0f;
	}
	t /= 100.0f;
	float min = 5;
	float max = 30;
	float s = min * (1.0f - t) + max * t;
	FillCubeTessellated(model, (int)s);

	// Project the tessellated cube onto a sphere
	for (int i = 0; i < model._Vertices.size(); i++) {
		Vec3 Pos = model._Vertices[i].Pos;
		Pos.Normalize();

		model._Vertices[i].Pos[0] = Pos[0];
		model._Vertices[i].Pos[1] = Pos[1];
		model._Vertices[i].Pos[2] = Pos[2];

		model._Vertices[i].norm[0] = FloatToByte_n11(Pos[0]);
		model._Vertices[i].norm[1] = FloatToByte_n11(Pos[1]);
		model._Vertices[i].norm[2] = FloatToByte_n11(Pos[2]);
		model._Vertices[i].norm[3] = FloatToByte_n11(0.0f);

		Vec3 tang = Byte4ToVec3(model._Vertices[i].norm);
		Vec3 bitang = Pos.Cross(tang);
		bitang.Normalize();
		tang = bitang.Cross(Pos);

		model._Vertices[i].tang[0] = FloatToByte_n11(tang[0]);
		model._Vertices[i].tang[1] = FloatToByte_n11(tang[1]);
		model._Vertices[i].tang[2] = FloatToByte_n11(tang[2]);
		model._Vertices[i].tang[3] = FloatToByte_n11(0.0f);
	}
}



bool Model::BuildFromShape(const Shape* shape)
{

	if (shape == nullptr)
	{
		return false;
	}
	if (shape->GetType() == Shape::SHAPE_BOX)
	{
		const ShapeBox* shape_box = (const ShapeBox*)shape;

		_Vertices.clear();
		_Indices.clear();
		FillCubeTessellated(*this, 0);
		Vec3 half_dim = (shape_box->_BoxBounds.maxs - shape_box->_BoxBounds.mins) * 0.5f;
		Vec3 centre = (shape_box->_BoxBounds.maxs + shape_box->_BoxBounds.mins) * 0.5f;
		for (int v = 0; v < _Vertices.size(); v++) {
			for (int i = 0; i < 3; i++) {
				_Vertices[v].Pos[i] *= half_dim[i];
				_Vertices[v].Pos[i] += centre[i];
			}
		}
	}
	else if (shape->GetType() == Shape::SHAPE_SPHERE)
	{
		const ShapeSphere* shape_sphere = (const ShapeSphere*)shape;

		_Vertices.clear();
		_Indices.clear();
		FillSphere(*this, shape_sphere->_Radius);
		for (int v = 0; v < _Vertices.size(); v++) {
			for (int i = 0; i < 3; i++) {
				_Vertices[v].Pos[i] *= shape_sphere->_Radius;
			}
		}
	}
	else if (shape->GetType() == Shape::SHAPE_CONVEX)
	{
		 ShapeConvex* shape_convex = ( ShapeConvex*)shape;

		_Vertices.clear();
		_Indices.clear();

		// Build the connected convex hull from the points
		std::vector< Vec3 > hull_points;
		std::vector< tri_t > hull_triangles;
		
		shape_convex->BuildConvexHull(shape_convex->_BoxPoints , hull_points , hull_triangles);

		//Calculate Smoothed Normals
		std::vector<Vec3> normals;
		normals.reserve(hull_points.size());

		for (int i = 0; i < hull_points.size(); i++)
		{
			Vec3 normal(0.0f);
			for (int t = 0; t < hull_triangles.size(); t++) {
				const tri_t& tri = hull_triangles[t];
				if (i != tri.a && i != tri.b && i != tri.c) {
					continue;
				}

				const Vec3& a = hull_points[tri.a];
				const Vec3& b = hull_points[tri.b];
				const Vec3& c = hull_points[tri.c];

				Vec3 ab = b - a;
				Vec3 ac = c - a;
				normal += ab.Cross(ac);
			}

			normal.Normalize();
			normals.push_back(normal);
		
		}

		_Vertices.reserve(hull_points.size());

		for (int i = 0; i < hull_points.size(); i++)
		{
			NewVertex vert;
			memset(&vert, 0, sizeof(vert));
			vert.Pos[0] = hull_points[i].x;
			vert.Pos[1] = hull_points[i].y;
			vert.Pos[2] = hull_points[i].z;
		
			Vec3 norm = normals[i];
			norm.Normalize();
	
			//Need To Add Normals

			_Vertices.push_back(vert);
		}
	
		_Indices.reserve(hull_triangles.size() * 3);
		for (int i = 0; i < hull_triangles.size(); i++) {
			_Indices.push_back(hull_triangles[i].a);
			_Indices.push_back(hull_triangles[i].b);
			_Indices.push_back(hull_triangles[i].c);
		}
	}

	return true;

}
void Model::MakeVertexBuffer(ID3D11Device* device)
{
	HRESULT hr;

	D3D11_BUFFER_DESC vertex_buffer;
	ZeroMemory(&vertex_buffer, sizeof(vertex_buffer));
	//Describing Trianlge Vertex Data 
	vertex_buffer.Usage = D3D11_USAGE_DEFAULT;
	vertex_buffer.ByteWidth = (int)(sizeof(_Vertices[0]) * _Vertices.size());//Size of data contained inside buffer
	vertex_buffer.BindFlags = D3D11_BIND_VERTEX_BUFFER;
	vertex_buffer.CPUAccessFlags = 0;

	D3D11_SUBRESOURCE_DATA InitTriangleData;
	ZeroMemory(&InitTriangleData, sizeof(InitTriangleData));
	InitTriangleData.pSysMem = _Vertices.data();

	hr = device->CreateBuffer(&vertex_buffer, &InitTriangleData, &_MeshData.VertexBuffer);

}

 void Model::Cleanup(ID3D11DeviceContext& deviceContext)
{
}

 void Model::DrawIndexed(ID3D11Device* device)
{
	 HRESULT hr;

	 D3D11_BUFFER_DESC index_buffer;
	 ZeroMemory(&index_buffer, sizeof(index_buffer));
	 //Describing Trianlge Vertex Data 
	 index_buffer.Usage = D3D11_USAGE_DEFAULT;
	 index_buffer.ByteWidth = (int)(sizeof(_Indices[0]) * _Indices.size());//Size of data contained inside buffer
	 index_buffer.BindFlags = D3D11_BIND_INDEX_BUFFER;
	 index_buffer.CPUAccessFlags = 0;

	 D3D11_SUBRESOURCE_DATA InitTriangleData;
	 ZeroMemory(&InitTriangleData, sizeof(InitTriangleData));
	 InitTriangleData.pSysMem = _Indices.data();

	 hr = device->CreateBuffer(& index_buffer, & InitTriangleData, & _MeshData.IndexBuffer);
}

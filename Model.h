#pragma once
#include "Vector.h"
#include "Quanternion.h"
#include "VertexShader.h"
#include "PixelShader.h"
#include "Mesh.h"
#include <vector>
#include <array>
#include "DirectX.h"

struct NewVertex
{
	float Pos[3];
	unsigned char Normal[4];
	XMFLOAT2 TexC;
	float			st[2];	// 8 bytes
	unsigned char	norm[4];	// 4 bytes
	unsigned char	tang[4];	// 4 bytes
	unsigned char	buff[4];	// 4 bytes
	bool operator<(const NewVertex other) const
	{
		return memcmp((void*)this, (void*)&other, sizeof(NewVertex)) > 0;
	};
};

class Shape;

class Model
{
public:
	Model();
	~Model();

	MeshData _MeshData;
	std::vector< NewVertex > _Vertices;
	std::vector< unsigned int > _Indices;

	bool BuildFromShape(const Shape* shape);
	void MakeVertexBuffer(ID3D11Device* device);

	DX* _Dx11;
	void Cleanup(ID3D11DeviceContext& deviceContext);

	void DrawIndexed(ID3D11Device* device);
private:

};

struct RenderModel {
	Model* model;			// The vao buffer to draw
	uint32_t uboByteOffset;	// The byte offset into the uniform buffer
	uint32_t uboByteSize;	// how much space we consume in the uniform buffer

	Vec3 pos;
	Quat orient;
};
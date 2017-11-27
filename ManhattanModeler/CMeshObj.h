#pragma once

#include <vector>

#include "CVector3D.h" 
#include "math_types.h"

enum elemType 
{
	T_WALL=0,
	T_ROOF=1,
};
 
class CMeshObj
{
public:
	CMeshObj(void);
	~CMeshObj(void);

public:
	struct MeshVertex 
	{
		CVector3D v;
		MeshVertex( const CVector3D & vv ) : v( vv ) { }
		MeshVertex() {}
	};
	struct MeshTriangle 
	{
		int i[ 3 ];
		MeshTriangle( int i0, int i1, int i2 ) 
		{
			i[ 0 ] = i0;
			i[ 1 ] = i1;
			i[ 2 ] = i2;
		}
		MeshTriangle() {}
	}; 

public:
	std::vector< MeshVertex >	m_vecVertex; 
	
	std::vector< MeshTriangle >	m_vecTriangle;  // Here, the index starts from 0. But attention. In obj formate file, vertex index starts from 1.
	std::vector<std::vector<int>>  m_vecTriRoofIdx; // distinguish each roof patch. 

	std::vector<CVector3D>		m_vecTriNrl;
	std::vector<elemType>		m_vecElemType;
	std::vector<CVector3D>		m_vecMeshMidPnt;
	std::vector<int> m_contour;
	 
	cgBbox3f bbox();

public:
	void pushVecVertice(std::vector<CVector3D> & pnts);
	void pushOneVertex(CVector3D & pnt);
	void PushOneFace(int i, int j, int k );

	static CMeshObj * loadFromObj(const std::string& file_name);

	void SaveToObj( const char * filename );
	void drawMesh(); // draw fit box model & Poisson model & model loaded from obj file
	void drawMeshObj(bool showNrl = false, bool rdmColor = false);// draw MRF polyhedron model

	void static combineTwoMeshObj( CMeshObj *&meshDst, CMeshObj *meshSrc );


private: 
	bool IsDegenrateFace( CMeshObj::MeshTriangle & f );
};

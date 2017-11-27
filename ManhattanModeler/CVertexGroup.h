#pragma once
 
#include <vector> 
#include "math_types.h"
#include "CScanVertex.h"
#include "BoundingBox.h"

class CVertexGroup: public std::vector<CScanVertex *>
{
public:
	CVertexGroup();
	CVertexGroup(std::vector<CScanVertex *> * vVetexs);
	~CVertexGroup();
public:
	CVector3D	m_midPnt;		// planar center point 
	cgPlane3f	m_cgPlane;		// plane
	Colorf		m_color;
	bool		m_toshow;
	CBoundingBox m_bbox;	

	std::vector<CScanVertex *> m_pnts; /// this vector storages the original points.

public: 
	void resetBBox();
	void drawGroup(unsigned int fast =1);

	void mergeGroup(CVertexGroup * anotherGroup);
	void mergeGroup(std::vector<CScanVertex *> * anotherGroup);
	void updateMidPnt();
	void clearProjection();

	// to fit the plane from the vector "m_pnts", e.g. refresh the plane model parameters.
	int refitPln_least_squares();

	// if the plane model has been modified, all projection vertices should be calculated again.
	void refreshProjectionVtx();

	// if a vertex is close to a plane, add it to the vector "m_pnts" and add its projection to this vertexGroup. 
	// errorThreshold is the maximum distance from the point to the plane. 
	bool check_add( CScanVertex * vtx, double suqareDistThresh, double nrlThreshold);
};

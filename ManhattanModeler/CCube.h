
//===========================================================================
// This code implements the FittingBox method described in:
//
// Minglei Li, Peter Wonka, Liangliang Nan 
// "Manhattan-world Urban Reconstruction from Point Clouds"
// ECCV'2016.
//
// E-Mail: minglei_li@126.com (Minglei Li)
//	       pwonka@gmail.com (Peter Wonka)
//         liangliang.nan@gmail.com (Liangliang Nan)
//
// All rights reserved
//===========================================================================


#pragma once

#include "math_types.h" 
#include "CPointCloud.h" 
#include "CPolyHedron.h"
#include "draw_model.h"

#include <QString> 

class CVertexGroup;  
class CPointCloud;
class CCube 
{
public: 
	CCube (CPolyHedron* cp);
	~CCube(void); 

public:
	CPointCloud * m_pc;

	CPolyHedron * 				m_CPolyHedron;
	typedef std::vector<CVector3D*> abox8Pnts;
	abox8Pnts					m_v8Pnts;	// 8 corners of a cube.	
	std::vector<CVertexGroup*>	m_v6Plns;	// 6 planes of a cube.
	std::vector<std::vector<CVector3D*>> m_6fc4corner; // 6 x 4 corner

	std::vector<std::vector<CScanVertex*>>	m_v6facetVtxes;	// store the points which located on 6 facets of each box.
  
 
	int	 m_numPntOnCube; // the number of points located on the cube. = num(m_v6facetVtxes)
	std::vector<double> 	m_6CoverRate;
	int m_validGridNum;
	double m_score;

	int m_id; // when building a cube, give it an integer id. 
	std::vector<int> m_6neighborCubeId; //  record the ids of neighbor cubes of this cube.
 

public:
	void dist_pnts_on6facet();   // distribute points to cube square facets.
	void cal_6coverRate();
	void draw_facetPnts();

private:  
	void dist_6FacetCorner();	// distribute 8 corners to 6 facets.

	bool pntInFacet(std::vector<CVector3D*> & facet4corner, CVector3D & pnt); 
};


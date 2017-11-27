
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


// box structure used after fitting boxes. 
#pragma  once

#include "math_types.h" 
#include "CPointCloud.h" 
#include "CPolyHedron.h"
#include "draw_model.h" 

#include "CFittingBox.h"

#include <QString> 
#include "CCube.h"

class CVertexGroup; 
class CBoundingBox; 
class CCubeBoxes: public std::vector<CCube*>
{
public:
	CCubeBoxes();  
	CCubeBoxes(CPointCloud * pc,CFittingBox * fb);
	~CCubeBoxes();
public: 
	CPointCloud		* m_pc;
	CBoundingBox	m_bbox;  

	CPolyHedron *						m_finalPhModel; // the final merged model. using bool option to merger m_vCPolyhedron.
	std::vector<bool>					m_boxOverlap;	// denote if each pair boxes are overlapped. if there are n boxes, this vector contains n*n elements.

private:
	std::vector<_pCoord> m_vecBoxCoord;
	int m_numX,m_numY,m_numZ; // denote how many rows, columns, roofs of boxes

private:
	std::vector<std::vector<CVector3D*>>	m_vecBox8pnts;		// its size is box number, a vector contains all detected boxes, each has 8 points.

	std::vector<std::vector<CVertexGroup*>>	m_vecBox6Plns;// its size is equal to box number,each item has 6 pointGroup.
	 
public:  
	void drawFitBoxes(DrawMode drawmodel);
	void drawViewScore();
	    
	void graphcutClassify();  

private:  
	void buildCubeFrom8Pnt();
	void identifyCubeNeighbor();
	void cal_cubeScore(); // for each cube, calculate its facet cover rate.
	  
	QString genIthName(int i, const char * str); // generate the variable name, e.g. x0, x1, x2, ...
	 
	void mergeUnionPolyhedron(); 
	void convert_to_exact_polyhedron(std::vector<CVector3D*> & v8Pnts, cgNef_polyhedron_3 &outNefPH);
};

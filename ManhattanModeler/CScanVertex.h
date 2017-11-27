#pragma once

#include "CVector3D.h"
#include "color.h"  

#include "math_types.h"

class CScanVertex
{
public:
	CScanVertex(void);
	CScanVertex(CVector3D & pnt,CVector3D & nrl, Colorf & clr);
	CScanVertex(CScanVertex & vtx);
	~CScanVertex(void);
public:
	CVector3D	point_ ;
	CVector3D   normal_;
	Colorf      color_ ;
	Colorf	*	color_error;
	Colorf  *   color_height;

	double		error; //  indicate the error from the point to the reconstructed surface, querying by AABB tree.
	bool        selected_ ;
	int			_index; // this is used while building minimum spanning tree. 
public: 
	void setColor(double r,double g,double b,double a =1) ;
	 
};

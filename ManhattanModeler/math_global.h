// geometry_global.h
#ifndef _MATH_GLOBAL_H_
#define _MATH_GLOBAL_H_


#include "math_types.h"
#include "color.h"

#define MATH_FLOAT_PI 3.14159265359

enum CoordTranferMode 
{
	TR_XY, // around Z axis
	TR_YZ, // around X axis
	TR_ZX  // around y axis
};

namespace Global {

	extern   	Colorf	   facet_color; 


	//////////////////////////////////////////////////////////////////////////


	float       normalize(cgVector3f& v) ; 
	float       length_of(const cgVector3f& v);

	cgBbox3f      bbox_of(const std::vector<cgPoint3f>& points);

} ;


#endif




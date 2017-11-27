#include "math_global.h"
#include <cgal/bounding_box.h>
#include <cmath>



namespace Global {

	Colorf	   facet_color	= Colorf(1.0f, 0.67f, 0.5f, 1.0f); 


	float normalize(cgVector3f& v) {
		float len = std::sqrt(v.squared_length());
		if (len != 0.0f)
			v = v / len;
		return len;
	} 

	float length_of(const cgVector3f& v) {
		return std::sqrt(v.squared_length());
	}

	cgBbox3f bbox_of(const std::vector<cgPoint3f>& points) {
		// axis-aligned bounding box of 3D points

		return CGAL::bounding_box(points.begin(), points.end()).bbox(); 
	}

}
#include "math_types.h"

#include <CGAL/bounding_box.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

namespace Geom 
{

	bool point_in_polygon( const cgPoint2f& p, const cgPolygon2f& polygon )
	{
		bool bIsInside = false;
		size_t n = (int)polygon.size();

		for (size_t i=0, j=n-1; i<n; j=i, ++i) 
		{
			const cgPoint2f& u0 = polygon[i];
			const cgPoint2f& u1 = polygon[j];  // current edge
			if (((u0.y() <= p.y()) && (p.y() < u1.y())) ||  // U1 is above the ray, U0 is on or below the ray
				((u1.y() <= p.y()) && (p.y() < u0.y())))    // U0 is above the ray, U1 is on or below the ray
			{
				// find x-intersection of current edge with the ray. 
				// Only consider edge crossings on the ray to the right of P.
				float x = u0.x() + (p.y() - u0.y()) * (u1.x() - u0.x()) / (u1.y() - u0.y());
				if (x > p.x())
					bIsInside = !bIsInside;
			}
		}
		return bIsInside;
	}

	bool point_in_polygon( const cgPoint2f& p, const std::vector<cgPoint2f>& polygon )
	{
		bool bIsInside = false;
		size_t n = (int)polygon.size();

		for (size_t i=0, j=n-1; i<n; j=i, ++i) {
			const cgPoint2f& u0 = polygon[i];
			const cgPoint2f& u1 = polygon[j];  // current edge
			if (((u0.y() <= p.y()) && (p.y() < u1.y())) ||  // U1 is above the ray, U0 is on or below the ray
				((u1.y() <= p.y()) && (p.y() < u0.y())))    // U0 is above the ray, U1 is on or below the ray
			{
				// find x-intersection of current edge with the ray. 
				// Only consider edge crossings on the ray to the right of P.
				float x = u0.x() + (p.y() - u0.y()) * (u1.x() - u0.x()) / (u1.y() - u0.y());
				if (x > p.x())
					bIsInside = !bIsInside;
			}
		}
		return bIsInside;
	}

	cgBbox2f bbox_of( const std::vector<cgPoint2f>& points )
	{
		return CGAL::bounding_box(points.begin(), points.end()).bbox();
	}

}

#ifndef _MATH_TYPES_H_
#define _MATH_TYPES_H_

#include "basic_types.h" 

#include <CGAL/Cartesian.h> 
#include <CGAL/Bbox_2.h>
#include <CGAL/Bbox_3.h>

#include <CGAL/Polygon_2.h>
#include <CGAL/Polyhedron_3.h>

#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/AABB_triangle_primitive.h>
#include <CGAL/HalfedgeDS_face_base.h>
#include <CGAL/HalfedgeDS_vertex_base.h>

#include <vector>
#include <tuple>        // std::tuple, std::get, std::tie, std::ignore

//#include "Vector3D.h"

//#include "rply.h"
#define nil 0 


typedef float				FT;				// number type float
typedef CGAL::Cartesian<FT> FloatKernel;				// kernel
typedef FloatKernel::Point_2			cgPoint2f;  
typedef FloatKernel::Point_3			cgPoint3f;		// point
typedef FloatKernel::Vector_3			cgVector3f;		// vector	
typedef FloatKernel::Line_3			cgLine3f;		// line
typedef FloatKernel::Segment_2		cgSegment2f;	// segment
typedef FloatKernel::Segment_3		cgSegment3f;	// segment
typedef FloatKernel::Plane_3			cgPlane3f;		// plane
typedef CGAL::Bbox_3        cgBbox3f;		// bounding box
typedef CGAL::Bbox_2		cgBbox2f;		// bounding box

typedef CGAL::Polygon_2<FloatKernel>	cgPolygon2f;

typedef FloatKernel::Triangle_3		cgTriangle;		// Triangle, used for error calculation in AABB tree

typedef CGAL::AABB_triangle_primitive<FloatKernel, std::vector<cgTriangle>::iterator> AABB_triangle_Primitive;
typedef CGAL::AABB_traits<FloatKernel, AABB_triangle_Primitive> AABB_triangle_traits;
typedef CGAL::AABB_tree<AABB_triangle_traits> cgAABBTree;



// a refined facet with a normal and a tag
template <class	Refs,	class	T, class P,	class	Norm>
class	Enriched_facet : public	CGAL::HalfedgeDS_face_base<Refs, T>
{
	// tag
	int	m_tag; 

	// normal
	Norm m_normal;

public:

	// life	cycle
	// no	constructors to	repeat,	since	only
	// default constructor mandatory

	Enriched_facet()
	{
	}

	// tag
	const int&	tag()	{	return m_tag;	}
	void tag(const int& t)	{	m_tag	=	t; }

	// normal
	typedef	Norm Normal_3;
	Normal_3&	normal() { return	m_normal;	}
	const	Normal_3&	normal() const { return	m_normal;	}
};

// a refined halfedge with a general tag and 
// a binary tag to indicate wether it belongs 
// to the control mesh or not
template <class	Refs,	class	Tprev, class Tvertex,	class	Tface, class Norm>
class	Enriched_halfedge	:	public CGAL::HalfedgeDS_halfedge_base<Refs,Tprev,Tvertex,Tface>
{
private:

	// tag
	int	m_tag; 

	// option	for	edge superimposing
	bool m_control_edge; 

public:

	// life	cycle
	Enriched_halfedge()
	{
		m_control_edge = true;
	}

	// tag
	const int& tag() const { return m_tag;	}
	int& tag() { return m_tag;	}
	void tag(const int& t)	{	m_tag	=	t; }

	// control edge	
	bool& control_edge()	{ return m_control_edge; }
	const bool& control_edge()	const { return m_control_edge; }
	void control_edge(const bool& flag) { m_control_edge	=	flag;	}
};



// a refined vertex with a normal and a tag
template <class	Refs,	class	T, class P,	class N, class	Kernel>
class	Enriched_vertex	:	public CGAL::HalfedgeDS_vertex_base<Refs,	T, P>
{
public:
	typedef	typename N Normal;
	//typedef typename CCurvature<Kernel> Curvature;

private:
	int	m_tag; 
	Normal m_normal;
	//Curvature m_curvature;

public:
	// life	cycle
	Enriched_vertex()	 {}
	// repeat	mandatory	constructors
	Enriched_vertex(const	P& pt)
		:	CGAL::HalfedgeDS_vertex_base<Refs, T,	P>(pt)
	{
	}

	// normal
	Normal&	normal() { return	m_normal;	}
	const	Normal&	normal() const { return	m_normal;	}

	//// curvature
	//Curvature& curvature() { return m_curvature; }
	//const Curvature& curvature() const { return m_curvature; }

	// tag
	int& tag() {	return m_tag;	}
	const int& tag() const {	return m_tag;	}
	void tag(const int& t)	{	m_tag	=	t; }
};

// A redefined items class for the Polyhedron_3	
// with	a	refined	vertex class that	contains a 
// member	for	the	normal vector	and	a	refined
// facet with	a	normal vector	instead	of the 
// plane equation	(this	is an	alternative	
// solution	instead	of using 
// Polyhedron_traits_with_normals_3).

struct Enriched_items	:	public CGAL::Polyhedron_items_3
{
		// wrap	vertex
		template <class	Refs,	class	Traits>
		struct Vertex_wrapper
		{
				typedef	typename Traits::Point_3 Point;
				typedef	typename Traits::Vector_3	Normal;
				typedef	Enriched_vertex<Refs,
													CGAL::Tag_true,
													Point,
													Normal,
				                  Traits>	Vertex;
		};

		// wrap	face
		template <class	Refs,	class	Traits>
		struct Face_wrapper
		{
				typedef	typename Traits::Point_3	Point;
				typedef	typename Traits::Vector_3	Normal;
				typedef	Enriched_facet<Refs,
												 CGAL::Tag_true,
												 Point,
												 Normal> Face;
		};

		// wrap	halfedge
		template <class	Refs,	class	Traits>
		struct Halfedge_wrapper
		{
				typedef	typename Traits::Vector_3	Normal;
				typedef	Enriched_halfedge<Refs,
														CGAL::Tag_true,
														CGAL::Tag_true,
														CGAL::Tag_true,
														Normal>	Halfedge;
		};
};
 
class	cgPolyhedron :	public CGAL::Polyhedron_3<FloatKernel,Enriched_items>
{ 
public :
	 
	cgPolyhedron()	
	{ 
	}
	virtual	~cgPolyhedron() 
	{
	}
};


//typedef CGAL::Polyhedron_3<FloatKernel>  cgPolyhedron;
typedef cgPolyhedron::Point_3 cgPolyPoint_3;
// This namespace gathers some global geometric utilities.
namespace Geom 
{
	cgBbox2f      bbox_of(const std::vector<cgPoint2f>& points);
	bool point_in_polygon(const cgPoint2f& p, const cgPolygon2f& polygon) ;

	bool point_in_polygon(const cgPoint2f& p, const std::vector<cgPoint2f>& polygon) ;


}

#endif


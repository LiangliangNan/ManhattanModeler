
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

#include "CPolyHedron.h"
#include <gl/glut.h>
//#include <CGAL/triangulate_polyhedron.h>
#include <iostream>
#include <fstream> 
#include "./CVector3D.h"
#include "rply.h"
#include <CGAL/Polyhedron_incremental_builder_3.h>
#include <CGAL/IO/Polyhedron_iostream.h>
#include "CPointCloud.h"
#include "logger.h"

#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/AABB_face_graph_triangle_primitive.h>

#include "Parameters.h"

typedef cgPolyhedron::Halfedge_handle Halfedge_handle; 
typedef cgPolyhedron::Facet_iterator Facet_iterator; 
typedef cgPolyhedron::Vertex_iterator Vertex_iteror;
typedef cgPolyhedron::Halfedge_around_facet_circulator phHalfedge_facet_circulator;

typedef cgPolyhedron::HalfedgeDS             HalfedgeDS;

typedef CGAL::AABB_face_graph_triangle_primitive<cgPolyhedron> AABB_Primitive;
typedef CGAL::AABB_traits<FloatKernel, AABB_Primitive> AABB_Traits;
typedef CGAL::AABB_tree<AABB_Traits> aabbTree; 

// A modifier creating a triangle with the incremental builder.
template<class HDS>
class polyhedron_builder : public CGAL::Modifier_base<HDS> 
{
public:
	std::vector<double> &coords;
	std::vector<int>    &tris;
	polyhedron_builder( std::vector<double> &_coords, std::vector<int> &_tris ) : coords(_coords), tris(_tris) 
	{}
	void operator()( HDS& hds) 
	{
		typedef typename HDS::Vertex   Vertex;
		typedef typename Vertex::Point Point;

		// create a cgal incremental builder
		CGAL::Polyhedron_incremental_builder_3<HDS> B( hds, true);
		
		B.begin_surface( coords.size()/3, tris.size()/3 );

		// add the polyhedron vertices
		for( int i=0; i<(int)coords.size(); i+=3 )
		{
			B.add_vertex( Point( coords[i+0], coords[i+1], coords[i+2] ) );
		}

		// add the polyhedron triangles
		for( int i=0; i<(int)tris.size(); i+=3 )
		{
			B.begin_facet();
			B.add_vertex_to_facet( tris[i+0] );
			B.add_vertex_to_facet( tris[i+1] );
			B.add_vertex_to_facet( tris[i+2] );
			B.end_facet();
		}

		// finish up the surface
		B.end_surface();
	}
};

CPolyHedron::CPolyHedron( cgPolyhedron& ph ):cgPolyhedron(ph),m_bbox(NULL)
{  
	m_hasNormal = false;
	onModel = false; 
	onModelTotally = false;
	onModelMaybe = false;	
	toShow =true;
	//m_nbPntOn = 0;
	Colorf colour(1,0,0); 
	if (ph.is_pure_quad() || ph.is_pure_triangle())
	{
		calculateVolume();
	}
}
CPolyHedron::CPolyHedron():m_bbox(NULL)
{  
	m_hasNormal = false;
	onModel = false; 
	toShow =true;
	//m_nbPntOn = 0;
	Colorf colour(1,0,0);
	m_volume = 0;
}

CPolyHedron::CPolyHedron( const CPolyHedron & cPH )
{
	m_hasNormal = false;
	this->toShow = cPH.toShow;
	this->onModel = cPH.onModel;
	this->colour = cPH.colour; 

	if (cPH.m_bbox!=NULL) 
		this->m_bbox = cPH.m_bbox; 
	else
		this->m_bbox = NULL;

	this->m_nefPolyHedron = cPH.m_nefPolyHedron; 
	this->m_volume = cPH.m_volume; 
}
 

float comp_pure_triangle_PolyHedronVolume(cgPolyhedron & ph)
{
	float pVolume = 0;
	if (!ph.is_closed() || !ph.is_pure_triangle())
	{
		return 0;
	}

	for ( Facet_iterator i = ph.facets_begin(); i != ph.facets_end(); ++i) 
	{
		phHalfedge_facet_circulator j = i->facet_begin(); 

		cgPolyPoint_3 v0 =	j->vertex()->point();
		cgPolyPoint_3 v1 = (++j)->vertex()->point();
		cgPolyPoint_3 v2 = (++j)->vertex()->point();

		CVector3D vec0 (v0.x(),v0.y(),v0.z());

		CVector3D vec10(v1.x()-v0.x(),v1.y()-v0.y(),v1.z()-v0.z());
		CVector3D vec20(v2.x()-v0.x(),v2.y()-v0.y(),v2.z()-v0.z());

		CVector3D vCross = vec10^vec20;
		float tempVolum = vCross * vec0;
		//????????? (float)CGAL::to_double ???????????????????????????????????
		//float tempVolum = (float)CGAL::to_double(CGAL::cross_product(vec10 , vec20) * vec0); 
		pVolume+=tempVolum; 
	}

	pVolume = pVolume /6;
	return pVolume;

}

void CPolyHedron::calculateVolume()
{ 
	//typedef cgPolyhedron::Facet_iterator Facet_iterator; 
	//typedef cgPolyhedron::Halfedge_around_facet_circulator Halfedge_facet_circulator;
	float pVolume = 0; 
	if (!this->is_closed() )
	{ 
		Logger::output("This polyhedron cannot compute volume!\nThis polyhedron is not closed.\n");
		return;
	}
	if (this->empty() || this->size_of_vertices ()<8 )
	{ 
		Logger::output("This polyhedron cannot compute volume!\n");
		return;
	}

	else if ( this->is_pure_triangle ())
	{
		pVolume = comp_pure_triangle_PolyHedronVolume((cgPolyhedron)(*this));
	}
	else if ( this->is_pure_quad ()) // if it is pure quad, split every facet to 2 triangles.
	{
		cgPolyhedron tempPH = (cgPolyhedron)(*this);

		for ( Facet_iterator i = tempPH.facets_begin(); i != tempPH.facets_end(); ++i) 
		{
			// the final volume may be negative. ?????????????????????????????
			// there are some problems, because the tempPH's structure has changed in the loop.
			cgPolyhedron::Halfedge_around_facet_circulator ci = i->facet_begin();
			if( CGAL::circulator_size(ci) != 4)
			{ 
				continue;
			}
			Halfedge_handle j1 = i->halfedge();
			Halfedge_handle j2 = j1->next()->next();

			// use CGAL function to split the facet.
			tempPH.split_facet(j1,j2); 
		}
		if ( tempPH.is_pure_triangle ())
		{
			pVolume = comp_pure_triangle_PolyHedronVolume(tempPH); 
			tempPH.clear();
		}
	} 
	m_volume = pVolume; 
}
  

void CPolyHedron::drawPolyhedron( float transparentRatio/*=1.0f*/ )
{
	if (!toShow)
	{
		return;
	}
	if (lml_param::showPHWirframe) 
	{
		GLboolean on = glIsEnabled(GL_LIGHTING);
		if(on)
			glDisable(GL_LIGHTING); 

		glEnable(GL_LINE_SMOOTH);
		glHint(GL_LINE_SMOOTH_HINT,GL_NICEST);  

		glPolygonMode(GL_FRONT_AND_BACK,GL_LINE);
		glColor4f(0.1f, 0.1f, 0.1f,1.0f);
		//glColor4f(colour.r(),colour.g(),colour.b(),transparent);
		glLineWidth(1.0f);
		glBegin(GL_LINES);
		for(cgPolyhedron::Edge_iterator h = this->edges_begin();h != edges_end();h++)
		{  
			// assembly	and	draw line	segment
			const cgPolyPoint_3& p1 = h->prev()->vertex()->point();
			const cgPolyPoint_3& p2 = h->vertex()->point();
			::glVertex3d(p1[0],p1[1],p1[2]);
			::glVertex3d(p2[0],p2[1],p2[2]);
		}
		::glEnd();
		if(on)
			glEnable(GL_LIGHTING); 
	} 

	if (!lml_param::rdmColor)
	{
		glColor4f(0.9f,0.9f,0.9f,1.0f);
	}
	else
		glColor4f(colour.r(),colour.g(),colour.b(),transparentRatio);

	// begin to draw polyhedron facets.
	glPolygonMode(GL_FRONT, GL_FILL); 
	for ( Facet_iterator pFacet = this->facets_begin(); pFacet != this->facets_end(); ++pFacet) 
	{ 
		glBegin(GL_POLYGON);
		// revolve around current face to get vertices
		phHalfedge_facet_circulator pHalfedge = pFacet->facet_begin();
		do
		{
			if (pHalfedge->vertex()->normal() != CGAL::NULL_VECTOR)
			{
				const Facet::Normal_3& normal	=	pHalfedge->vertex()->normal();
				::glNormal3d(normal[0],normal[1],normal[2]);
			} 

			// polygon assembly	is performed per vertex
			const cgPolyPoint_3& point	=	pHalfedge->vertex()->point();
			::glVertex3d(point[0],point[1],point[2]);
		}
		while(++pHalfedge	!= pFacet->facet_begin());
		glEnd();
	}  

	//if (lml_param::showNormal)
	//{ 
	//	GLboolean on = glIsEnabled(GL_LIGHTING);
	//	if(on)
	//		glDisable(GL_LIGHTING); 
	//	
	//	glBegin(GL_LINES);
	//	for(cgPolyhedron::Vertex_iterator iterV = this->vertices_begin();iterV != this->vertices_end();iterV++)
	//	{  
	//		const cgPolyPoint_3& p1 = iterV->point();
	//		cgPolyPoint_3 p2 = iterV->normal();
	//		::glVertex3d(p1[0],p1[1],p1[2]);
	//		::glVertex3d(p2[0],p2[1],p2[2]);
	//	}
	//	::glEnd();

	//	if (on)
	//		glEnable(GL_LIGHTING);
	//}
}
   
void CPolyHedron::make_cube_3( cgPolyhedron & P, 
							  cgPolyPoint_3 &p0,cgPolyPoint_3 &p1,cgPolyPoint_3 &p2,cgPolyPoint_3 &p3, 
							  cgPolyPoint_3 &p4,cgPolyPoint_3 &p5,cgPolyPoint_3 &p6,cgPolyPoint_3 &p7 )
{

	cgVector3f v0 = p0  - CGAL::ORIGIN;
	cgVector3f v1 = p1  - CGAL::ORIGIN;
	cgVector3f v2 = p2  - CGAL::ORIGIN;
	cgVector3f v3 = p3  - CGAL::ORIGIN;
	cgVector3f v4 = p4  - CGAL::ORIGIN;
	cgVector3f v5 = p5  - CGAL::ORIGIN;
	cgVector3f v6 = p6  - CGAL::ORIGIN;
	cgVector3f v7 = p7  - CGAL::ORIGIN;

	cgVector3f v76 = v7-v6;
	cgVector3f v26 = v2-v6;
	cgVector3f v46 = v4-v6;

	// appends a cube of size [0,1]^3 to the polyhedron P.
	CGAL_precondition( P.is_valid());

	Halfedge_handle h;
	if (CGAL::to_double(CGAL::cross_product(v76,v26)*v46)>0)
	{
		// satisfy the order of planes(xi,xi+1,yi,yi+1,zi,zi+1)

		//http://doc.cgal.org/latest/Polyhedron/index.html 
		h = P.make_tetrahedron( p7, p4, p6, p2);
		Halfedge_handle g = h->next()->opposite()->next(); // Fig. (a)
		P.split_edge( h->next());
		P.split_edge( g->next());
		P.split_edge( g); // Fig. (b)
		h->next()->vertex()->point() = p5;
		g->next()->vertex()->point() = p0;
		g->opposite()->vertex()->point() = p3; // Fig. (c)
		Halfedge_handle f = P.split_facet( g->next(),
			g->next()->next()->next()); // Fig. (d)
		Halfedge_handle e = P.split_edge( f);
		e->vertex()->point() = p1; // Fig. (e)
		P.split_facet( e, f->next()->next()); // Fig. (f)
	}
	else
	{
		h = P.make_tetrahedron( p4, p1, p0, p2);
		Halfedge_handle g = h->next()->opposite()->next(); // Fig. (a)
		P.split_edge( h->next());
		P.split_edge( g->next());
		P.split_edge( g); // Fig. (b)
		h->next()->vertex()->point() = p5;
		g->next()->vertex()->point() = p3;
		g->opposite()->vertex()->point() = p6; // Fig. (c)
		Halfedge_handle f = P.split_facet( g->next(),
			g->next()->next()->next()); // Fig. (d)
		Halfedge_handle e = P.split_edge( f);
		e->vertex()->point() = p7; // Fig. (e)
		P.split_facet( e, f->next()->next()); // Fig. (f)
	}
	//{
	//	h = P.make_tetrahedron( p3, p0, p2, p6);
	//	Halfedge_handle g = h->next()->opposite()->next(); // Fig. (a)
	//	P.split_edge( h->next());
	//	P.split_edge( g->next());
	//	P.split_edge( g); // Fig. (b)
	//	h->next()->vertex()->point() = p1;
	//	g->next()->vertex()->point() = p4;
	//	g->opposite()->vertex()->point() = p7; // Fig. (c)
	//	Halfedge_handle f = P.split_facet( g->next(),
	//		g->next()->next()->next()); // Fig. (d)
	//	Halfedge_handle e = P.split_edge( f);
	//	e->vertex()->point() = p5; // Fig. (e)
	//	P.split_facet( e, f->next()->next()); // Fig. (f)
	//}
	CGAL_postcondition( P.is_valid());

	return ;
}

void CPolyHedron::make_cube_3( cgPolyhedron & P, const std::vector<cgPoint3f>& cube8Pnts )
{
	if (cube8Pnts.size()!=8)
	{
		std::cout<<"\nerror: make cube. point number error!\n"<<std::endl;
	}
	cgPolyPoint_3 p0 = cube8Pnts[0];cgVector3f v0 = p0  - CGAL::ORIGIN;
	cgPolyPoint_3 p1 = cube8Pnts[1];cgVector3f v1 = p1  - CGAL::ORIGIN;
	cgPolyPoint_3 p2 = cube8Pnts[2];cgVector3f v2 = p2  - CGAL::ORIGIN;
	cgPolyPoint_3 p3 = cube8Pnts[3];cgVector3f v3 = p3  - CGAL::ORIGIN;
	cgPolyPoint_3 p4 = cube8Pnts[4];cgVector3f v4 = p4  - CGAL::ORIGIN;
	cgPolyPoint_3 p5 = cube8Pnts[5];cgVector3f v5 = p5  - CGAL::ORIGIN;
	cgPolyPoint_3 p6 = cube8Pnts[6];cgVector3f v6 = p6  - CGAL::ORIGIN;
	cgPolyPoint_3 p7 = cube8Pnts[7];cgVector3f v7 = p7  - CGAL::ORIGIN;

	cgVector3f v76 = v7-v6;
	cgVector3f v26 = v2-v6;
	cgVector3f v46 = v4-v6;

	// appends a cube of size [0,1]^3 to the polyhedron P.
	CGAL_precondition( P.is_valid());

	Halfedge_handle h;
	if (CGAL::to_double(CGAL::cross_product(v76,v26)*v46)>0)
	{
		//http://doc.cgal.org/latest/Polyhedron/index.html 
		h = P.make_tetrahedron( p7, p4, p6, p2);
		Halfedge_handle g = h->next()->opposite()->next(); // Fig. (a)
		P.split_edge( h->next());
		P.split_edge( g->next());
		P.split_edge( g); // Fig. (b)
		h->next()->vertex()->point() = p5;
		g->next()->vertex()->point() = p0;
		g->opposite()->vertex()->point() = p3; // Fig. (c)
		Halfedge_handle f = P.split_facet( g->next(),
			g->next()->next()->next()); // Fig. (d)
		Halfedge_handle e = P.split_edge( f);
		e->vertex()->point() = p1; // Fig. (e)
		P.split_facet( e, f->next()->next()); // Fig. (f)
	}
	else
	{
		h = P.make_tetrahedron( p3, p0, p2, p6);
		Halfedge_handle g = h->next()->opposite()->next(); // Fig. (a)
		P.split_edge( h->next());
		P.split_edge( g->next());
		P.split_edge( g); // Fig. (b)
		h->next()->vertex()->point() = p1;
		g->next()->vertex()->point() = p4;
		g->opposite()->vertex()->point() = p7; // Fig. (c)
		Halfedge_handle f = P.split_facet( g->next(),
			g->next()->next()->next()); // Fig. (d)
		Halfedge_handle e = P.split_edge( f);
		e->vertex()->point() = p5; // Fig. (e)
		P.split_facet( e, f->next()->next()); // Fig. (f)
	}
	CGAL_postcondition( P.is_valid());

	return ;

}

cgNewPolyhedron_3::Halfedge_handle CPolyHedron::make_cube_3( cgNewPolyhedron_3 & P, cgNewPolyhedron_3::Point_3 &p0,cgNewPolyhedron_3::Point_3 &p1,cgNewPolyhedron_3::Point_3 &p2,cgNewPolyhedron_3::Point_3 &p3, cgNewPolyhedron_3::Point_3 &p4,cgNewPolyhedron_3::Point_3 &p5,cgNewPolyhedron_3::Point_3 &p6,cgNewPolyhedron_3::Point_3 &p7 )
{
	newVector3f v0 = p0  - CGAL::ORIGIN;
	newVector3f v1 = p1  - CGAL::ORIGIN;
	newVector3f v2 = p2  - CGAL::ORIGIN;
	newVector3f v3 = p3  - CGAL::ORIGIN;
	newVector3f v4 = p4  - CGAL::ORIGIN;
	newVector3f v5 = p5  - CGAL::ORIGIN;
	newVector3f v6 = p6  - CGAL::ORIGIN;
	newVector3f v7 = p7  - CGAL::ORIGIN;

	newVector3f v76 = v7-v6;
	newVector3f v26 = v2-v6;
	newVector3f v46 = v4-v6;

	// appends a cube of size [0,1]^3 to the polyhedron P.
	CGAL_precondition( P.is_valid());
	cgNewPolyhedron_3::Halfedge_handle h;

	if (CGAL::to_double(CGAL::cross_product(v76,v26)*v46)>0)
	{ 
		//typedef typename Poly::Point_3 Point;
		h = P.make_tetrahedron( p7, p4, p6, p2);
		cgNewPolyhedron_3::Halfedge_handle g = h->next()->opposite()->next(); // Fig. (a)
		P.split_edge( h->next());
		P.split_edge( g->next());
		P.split_edge( g); // Fig. (b)
		h->next()->vertex()->point() = p5;
		g->next()->vertex()->point() = p0;
		g->opposite()->vertex()->point() = p3; // Fig. (c)
		cgNewPolyhedron_3::Halfedge_handle f = P.split_facet( g->next(),
			g->next()->next()->next()); // Fig. (d)
		cgNewPolyhedron_3::Halfedge_handle e = P.split_edge( f);
		e->vertex()->point() = p1; // Fig. (e)
		P.split_facet( e, f->next()->next()); // Fig. (f)
		CGAL_postcondition( P.is_valid());
	}
	else
	{
		//typedef typename Poly::Point_3 Point;
		h = P.make_tetrahedron( p3, p0, p2, p6);
		cgNewPolyhedron_3::Halfedge_handle g = h->next()->opposite()->next(); // Fig. (a)
		P.split_edge( h->next());
		P.split_edge( g->next());
		P.split_edge( g); // Fig. (b)
		h->next()->vertex()->point() = p1;
		g->next()->vertex()->point() = p4;
		g->opposite()->vertex()->point() = p7; // Fig. (c)
		cgNewPolyhedron_3::Halfedge_handle f = P.split_facet( g->next(),
			g->next()->next()->next()); // Fig. (d)
		cgNewPolyhedron_3::Halfedge_handle e = P.split_edge( f);
		e->vertex()->point() = p5; // Fig. (e)
		P.split_facet( e, f->next()->next()); // Fig. (f)
		CGAL_postcondition( P.is_valid());

	}
	return h;
}


void CPolyHedron::savePolyHedron_to_OFF(std::ofstream & out, cgPolyhedron & P )
{

}

bool CPolyHedron::read_Polyhedron_From_OFF( const char * file_name )
{
	std::ifstream stream(file_name);
	if(!stream)
	{
		Logger::output( "Cannot open file!\n");
		return false;
	} 
	stream >> *this;
	if(!stream)
	{
		Logger::output("It is not a polyhedron\n"); 
		return false;
	} 
	  
	calculateVolume();

	buildBBoxForPH();

	compute_normals();

	Colorf c(214.0f/255.0f,214.0f/255.0f,214.0f/255.0f,1.0f);
	set_color(c);
	toShow = true;
	onModel = false;

	return true;
}


// throws away texture coordinates, normals, etc.
// stores results in input coords array, packed [x0,y0,z0,x1,y1,z1,...] and
// tris array packed [T0a,T0b,T0c,T1a,T1b,T1c,...]
bool CPolyHedron::read_Polyhedron_From_obj( const char *filename )
{ 
	std::vector<double> coords;
	std::vector<int>	tris;
	double x, y, z;
	char line[126];

	// open the file, return if open fails
	FILE *fp = fopen(filename, "r" );
	if( !fp ) return false;

	// read lines from the file, if the first character of the
	// line is 'v', we are reading a vertex, otherwise, if the
	// first character is a 'f' we are reading a facet
	while( fgets( line, 126, fp ) )
	{
		if( line[0] == 'v' )
		{
			sscanf( line, "%*s %lf %lf %lf%", &x, &y, &z );
			coords.push_back( x );
			coords.push_back( y );
			coords.push_back( z );
		} else if( line[0] == 'f' )
		{
			int i0,i1,i2;
			sscanf( line, "%*s %d %d %d", &i0, &i1, &i2 );
			tris.push_back( i0-1 );
			tris.push_back( i1-1 );
			tris.push_back( i2-1 );
		}
	}
	fclose(fp);

	if( coords.size() == 0 )
		return false;

	CPolyHedron * Ph = this;

	Logger::output(Logger::convInt2Str(tris.size()) + "\n");
	polyhedron_builder<HalfedgeDS> builder( coords, tris );
	Ph->delegate( builder );
	 
	Logger::output(Logger::convInt2Str(Ph->size_of_vertices()) + "\n");
	Logger::output(Logger::convInt2Str(Ph->size_of_facets()) + "\n");

	calculateVolume();

	buildBBoxForPH();

	compute_normals();



	Colorf c(1.0,0,0,1.0);
	set_color(c);
	toShow = true;
	onModel = false;
	return true;
}

void CPolyHedron::buildBBoxForPH()
{
	float minx=1e10,miny=1e10,minz=1e10,maxx=-1e10,maxy=-1e10,maxz=-1e10;
	for ( cgPolyhedron::Point_iterator i = this->points_begin(); i != this->points_end(); ++i) 
	{

		cgPoint3f pnti = *i;
		float xi = pnti.x();
		float yi = pnti.y();
		float zi = pnti.z();
		if (xi<minx) { minx = xi; }
		if (xi>maxx) { maxx = xi; }
		if (yi<miny) { miny = yi; }
		if (yi>maxy) { maxy = yi; }
		if (zi<minz) { minz = zi; }
		if (zi>maxz) { maxz = zi; }
	}
	m_bbox= new CBoundingBox(minx,miny,minz,maxx,maxy,maxz);
}


bool CPolyHedron::save_PHGroup_Ply( std::vector<CPolyHedron*> & obj, const char * filename ) 
{
	std::vector<CVector3D> vPnt;
	std::vector<CVector3D> vNrl;
	std::vector<CVector3D> vClr;
	std::vector<std::vector<int>> vvIndx;
	int pntNum=0;
	int fctNum=0;
	for (int i=0;i<obj.size();i++)
	{
		CPolyHedron & ph = *obj[i]; 
		if ( ph.is_pure_quad ()) // if it is pure quad, split every facet to 2 triangles.
		{
			for ( Facet_iterator i = ph.facets_begin(); i != ph.facets_end(); ++i) 
			{
				phHalfedge_facet_circulator j = i->facet_begin();
				// Facets in polyhedral surfaces are at least triangles.
				if( CGAL::circulator_size(j) != 4)
				{ 
					continue;
				}
				cgPolyPoint_3 v0 =	j->vertex()->point();
				cgPolyPoint_3 v1 = (++j)->vertex()->point();
				cgPolyPoint_3 v2 = (++j)->vertex()->point();
				cgPolyPoint_3 v3 = (++j)->vertex()->point();

				CVector3D vec0 (CGAL::to_double(v0.x()),CGAL::to_double(v0.y()),CGAL::to_double(v0.z())); 
				CVector3D vec1 (CGAL::to_double(v1.x()),CGAL::to_double(v1.y()),CGAL::to_double(v1.z())); 
				CVector3D vec2 (CGAL::to_double(v2.x()),CGAL::to_double(v2.y()),CGAL::to_double(v2.z())); 
				CVector3D vec3 (CGAL::to_double(v3.x()),CGAL::to_double(v3.y()),CGAL::to_double(v3.z())); 
				vPnt.push_back(vec0);
				vPnt.push_back(vec1);
				vPnt.push_back(vec2);
				vPnt.push_back(vec3);
				CVector3D nrl = (vec0-vec1)^(vec2-vec1);
				nrl.normalize();
				vNrl.push_back(nrl);vNrl.push_back(nrl);vNrl.push_back(nrl);vNrl.push_back(nrl);
				CVector3D clor (ph.colour.r(),ph.colour.g(),ph.colour.b());
				vClr.push_back(clor);vClr.push_back(clor);vClr.push_back(clor);vClr.push_back(clor);
				std::vector<int> vidx;
				vidx.push_back(pntNum++);
				vidx.push_back(pntNum++);
				vidx.push_back(pntNum++);
				vidx.push_back(pntNum++);

				vvIndx.push_back(vidx);
				fctNum++;

			}
		}
		if ( ph.is_pure_triangle()) // if it is pure quad, split every facet to 2 triangles.
		{
			for ( Facet_iterator i = ph.facets_begin(); i != ph.facets_end(); ++i) 
			{
				phHalfedge_facet_circulator j = i->facet_begin();
				// Facets in polyhedral surfaces are at least triangles.
				if( CGAL::circulator_size(j) != 3)
				{ 
					continue;
				}
				cgPolyPoint_3 v0 =	j->vertex()->point();
				cgPolyPoint_3 v1 = (++j)->vertex()->point();
				cgPolyPoint_3 v2 = (++j)->vertex()->point(); 

				CVector3D vec0 (CGAL::to_double(v0.x()),CGAL::to_double(v0.y()),CGAL::to_double(v0.z())); 
				CVector3D vec1 (CGAL::to_double(v1.x()),CGAL::to_double(v1.y()),CGAL::to_double(v1.z())); 
				CVector3D vec2 (CGAL::to_double(v2.x()),CGAL::to_double(v2.y()),CGAL::to_double(v2.z()));  
				vPnt.push_back(vec0);
				vPnt.push_back(vec1);
				vPnt.push_back(vec2);
				CVector3D nrl = (vec0-vec1)^(vec2-vec1);
				nrl.normalize();
				vNrl.push_back(nrl);vNrl.push_back(nrl);vNrl.push_back(nrl); 
				CVector3D clor (ph.colour.r(),ph.colour.g(),ph.colour.b());
				vClr.push_back(clor);vClr.push_back(clor);vClr.push_back(clor); 
				std::vector<int> vidx;
				vidx.push_back(pntNum++);
				vidx.push_back(pntNum++);
				vidx.push_back(pntNum++); 

				vvIndx.push_back(vidx);
				fctNum++;

			}
		}
	} 
	p_ply ply = ply_create(filename, PLY_LITTLE_ENDIAN, nil, 0, nil) ;

	if(ply == nil) 
	{
		return false ;
	}

	if (!ply_add_element(ply, "vertex", pntNum)) 
	{
		ply_close(ply) ;
		return false ;
	}

	e_ply_type length_type, value_type;
	length_type = value_type = static_cast<e_ply_type>(-1);
	std::string pos[3] = { "x", "y", "z" };
	for (unsigned int i=0; i<3; ++i) 
	{
		if (!ply_add_property(ply, pos[i].c_str(), PLY_FLOAT, length_type, value_type)) 
		{
			ply_close(ply) ;
			return false ;
		}
	}

	std::string normal[3] = { "nx", "ny", "nz" };
	for (unsigned int i=0; i<3; ++i) 
	{
		if (!ply_add_property(ply, normal[i].c_str(), PLY_FLOAT, length_type, value_type)) 
		{
			ply_close(ply) ;
			return false ;
		}
	}

	std::string color[3] = { "red", "green", "blue" };
	for (unsigned int i=0; i<3; ++i) 
	{
		if (!ply_add_property(ply, color[i].c_str(), PLY_UCHAR, length_type, value_type)) 
		{
			ply_close(ply) ;
			return false ;
		}
	}


	if (!ply_add_element(ply, "face", fctNum)) 
	{ 
		ply_close(ply) ;
		return false ;
	}
	if (!ply_add_property(ply, "vertex_indices", PLY_LIST, PLY_UCHAR, PLY_INT)) 
	{ 
		ply_close(ply) ;
		return false ;
	} 

	if(!ply_write_header(ply)) 
	{
		ply_close(ply) ;
		return false ;
	}

	for (int i=0; i<pntNum; ++i) { 
		ply_write(ply, vPnt[i].pVec[0]);
		ply_write(ply, vPnt[i].pVec[1]);
		ply_write(ply, vPnt[i].pVec[2]);

		ply_write(ply, vNrl[i].pVec[0]);
		ply_write(ply, vNrl[i].pVec[1]);
		ply_write(ply, vNrl[i].pVec[2]);
		//int clr = (int)(vClr[i].pVec[0]*255)%256;
		//int clg = (int)(vClr[i].pVec[1]*255)%256;
		//int clb = (int)(vClr[i].pVec[2]*255)%256;

		int clr = 150;
		int clg = 150;
		int clb = 150;

		ply_write(ply, clr);
		ply_write(ply, clg);
		ply_write(ply, clb); 
	}

	for (unsigned int i=0; i<fctNum; ++i) 
	{ 
		const std::vector<int> &  vts = vvIndx[i] ;
		ply_write(ply, vts.size());
		for (unsigned int j=0; j<vts.size(); ++j) 
		{
			ply_write(ply, vts[j]);
		}
	}

	ply_close(ply);
	return true ;
}

bool CPolyHedron::save_PHGroup_Ply( std::vector<CPolyHedron*> & obj, const char * filename, double bigTheta )
{
	std::vector<CVector3D> vPnt;
	std::vector<CVector3D> vNrl;
	std::vector<CVector3D> vClr;
	std::vector<std::vector<int>> vvIndx;
	int pntNum=0;
	int fctNum=0;
	for (int i=0;i<obj.size();i++)
	{
		CPolyHedron & ph = *obj[i]; 
		if ( ph.is_pure_quad ()) // if it is pure quad, split every facet to 2 triangles.
		{
			for ( Facet_iterator i = ph.facets_begin(); i != ph.facets_end(); ++i) 
			{
				phHalfedge_facet_circulator j = i->facet_begin();
				// Facets in polyhedral surfaces are at least triangles.
				if( CGAL::circulator_size(j) != 4)
				{ 
					continue;
				}
				cgPolyPoint_3 v0 =	j->vertex()->point();
				cgPolyPoint_3 v1 = (++j)->vertex()->point();
				cgPolyPoint_3 v2 = (++j)->vertex()->point();
				cgPolyPoint_3 v3 = (++j)->vertex()->point();

				CVector3D vec0 (v0.x(),v0.y(),v0.z()); vPnt.push_back(vec0);
				CVector3D vec1 (v1.x(),v1.y(),v1.z()); vPnt.push_back(vec1);
				CVector3D vec2 (v2.x(),v2.y(),v2.z()); vPnt.push_back(vec2);
				CVector3D vec3 (v3.x(),v3.y(),v3.z()); vPnt.push_back(vec3);
				CVector3D nrl = (vec0-vec1)^(vec2-vec1);
				nrl.normalize();
				vNrl.push_back(nrl);vNrl.push_back(nrl);vNrl.push_back(nrl);vNrl.push_back(nrl);
				CVector3D clor (ph.colour.r(),ph.colour.g(),ph.colour.b());
				vClr.push_back(clor);vClr.push_back(clor);vClr.push_back(clor);vClr.push_back(clor);
				std::vector<int> vidx;
				vidx.push_back(pntNum++);
				vidx.push_back(pntNum++);
				vidx.push_back(pntNum++);
				vidx.push_back(pntNum++);

				vvIndx.push_back(vidx);
				fctNum++;

			}
		}
		if ( ph.is_pure_triangle() ) // if it is pure quad, split every facet to 2 triangles.
		{
			for ( Facet_iterator i = ph.facets_begin(); i != ph.facets_end(); ++i) 
			{
				phHalfedge_facet_circulator j = i->facet_begin(); 
				cgPolyPoint_3 v0 =	j->vertex()->point();
				cgPolyPoint_3 v1 = (++j)->vertex()->point();
				cgPolyPoint_3 v2 = (++j)->vertex()->point(); 

				CVector3D vec0 (v0.x(),v0.y(),v0.z()); vPnt.push_back(vec0);
				CVector3D vec1 (v1.x(),v1.y(),v1.z()); vPnt.push_back(vec1);
				CVector3D vec2 (v2.x(),v2.y(),v2.z()); vPnt.push_back(vec2); 
				CVector3D nrl = (vec0-vec1)^(vec2-vec1);
				nrl.normalize();
				vNrl.push_back(nrl);vNrl.push_back(nrl);vNrl.push_back(nrl);
				CVector3D clor (ph.colour.r(),ph.colour.g(),ph.colour.b());
				vClr.push_back(clor);vClr.push_back(clor);vClr.push_back(clor);
				std::vector<int> vidx;
				vidx.push_back(pntNum++);
				vidx.push_back(pntNum++);
				vidx.push_back(pntNum++); 

				vvIndx.push_back(vidx);
				fctNum++;

			}
		}
	}

	for (int i=0;i<vPnt.size();i++)
	{
		//double cosTheta = cos(bigTheta);
		//double sinTheta = sin(bigTheta);
		CVector3D & pnt = vPnt[i];
		CPointCloud::trBackSingPnt(pnt,bigTheta,TR_XY);
		CVector3D & nrl = vNrl[i];
		CPointCloud::trBackSingPnt(nrl,bigTheta,TR_XY);
	}

	p_ply ply = ply_create(filename, PLY_LITTLE_ENDIAN, nil, 0, nil) ;

	if(ply == nil) 
	{
		return false ;
	}

	if (!ply_add_element(ply, "vertex", pntNum)) 
	{
		ply_close(ply) ;
		return false ;
	}

	e_ply_type length_type, value_type;
	length_type = value_type = static_cast<e_ply_type>(-1);
	std::string pos[3] = { "x", "y", "z" };
	for (unsigned int i=0; i<3; ++i) 
	{
		if (!ply_add_property(ply, pos[i].c_str(), PLY_FLOAT, length_type, value_type)) 
		{
			ply_close(ply) ;
			return false ;
		}
	}

	std::string normal[3] = { "nx", "ny", "nz" };
	for (unsigned int i=0; i<3; ++i) 
	{
		if (!ply_add_property(ply, normal[i].c_str(), PLY_FLOAT, length_type, value_type)) 
		{
			ply_close(ply) ;
			return false ;
		}
	}

	std::string color[3] = { "red", "green", "blue" };
	for (unsigned int i=0; i<3; ++i) 
	{
		if (!ply_add_property(ply, color[i].c_str(), PLY_UCHAR, length_type, value_type)) 
		{
			ply_close(ply) ;
			return false ;
		}
	}


	if (!ply_add_element(ply, "face", fctNum)) 
	{ 
		ply_close(ply) ;
		return false ;
	}
	if (!ply_add_property(ply, "vertex_indices", PLY_LIST, PLY_UCHAR, PLY_INT)) 
	{ 
		ply_close(ply) ;
		return false ;
	} 

	if(!ply_write_header(ply)) 
	{
		ply_close(ply) ;
		return false ;
	}

	for (int i=0; i<pntNum; ++i) { 
		ply_write(ply, vPnt[i].pVec[0]);
		ply_write(ply, vPnt[i].pVec[1]);
		ply_write(ply, vPnt[i].pVec[2]);

		ply_write(ply, vNrl[i].pVec[0]);
		ply_write(ply, vNrl[i].pVec[1]);
		ply_write(ply, vNrl[i].pVec[2]);
		//int clr = (int)(vClr[i].pVec[0]*255)%256;
		//int clg = (int)(vClr[i].pVec[1]*255)%256;
		//int clb = (int)(vClr[i].pVec[2]*255)%256;

		int clr = 150;
		int clg = 150;
		int clb = 150;

		ply_write(ply, clr);
		ply_write(ply, clg);
		ply_write(ply, clb); 
	}

	for (unsigned int i=0; i<fctNum; ++i) 
	{ 
		const std::vector<int> &  vts = vvIndx[i] ;
		ply_write(ply, vts.size());
		for (unsigned int j=0; j<vts.size(); ++j) 
		{
			ply_write(ply, vts[j]);
		}
	}

	ply_close(ply);
	return true ;
}

bool pntInside(const CVector3D & point, cgPolyhedron & polyhedrn) 
{  
	if (!polyhedrn.is_pure_triangle())
	{
		return false;
	}
	for (Facet_iterator i = polyhedrn.facets_begin(); i != polyhedrn.facets_end(); ++i) 
	{ 

		phHalfedge_facet_circulator j = i->facet_begin(); 

		cgPolyPoint_3 v0 =	j->vertex()->point();
		CVector3D pv0(v0.x(),v0.y(),v0.z());
		cgPolyPoint_3 v1 = (++j)->vertex()->point();
		cgPolyPoint_3 v2 = (++j)->vertex()->point(); 

		CVector3D vec10(v1.x()-v0.x(),v1.y()-v0.y(),v1.z()-v0.z());
		CVector3D vec20(v2.x()-v0.x(),v2.y()-v0.y(),v2.z()-v0.z());

		CVector3D normal =  vec10^vec20;  // all normals should face outside of the polyhedron.

		CVector3D pv = point-pv0;
		float D = pv*normal;
		if (D>0)
		{
			return false;
		}
	}
	return true;
}
  
void CPolyHedron::convert_PH_kernel( cgNewPolyhedron_3 & in_PH, cgPolyhedron & out_PH ) // unfinished code......
{
	if (in_PH.empty() || !in_PH.is_pure_triangle() || !in_PH.is_pure_quad())
	{
		Logger::output("The polyhedron is not pure triangle or pure quad, ...\n");
		//return;
	}
	
	std::vector<double> coords;
	std::vector<int>	vidx;

	int pntNum=0;

	if ( in_PH.is_pure_quad ()) // if it is pure quad, split every facet to 2 triangles.
	{ 
		for ( cgNewPolyhedron_3::Facet_iterator i = in_PH.facets_begin(); i != in_PH.facets_end(); ++i) 
		{ 
			cgNewPolyhedron_3::Halfedge_around_facet_circulator ci = i->facet_begin(); 

			cgNewPolyhedron_3::Halfedge_handle j1 = i->halfedge();
			cgNewPolyhedron_3::Halfedge_handle j2 = j1->next()->next();

			// use CGAL function to split the facet.
			in_PH.split_facet(j1,j2); 
		} 
	} 

	//if ( in_PH.is_pure_triangle())
	//{ 
		for ( cgNewPolyhedron_3::Facet_iterator i = in_PH.facets_begin(); i != in_PH.facets_end(); ++i) 
		{
			cgNewPolyhedron_3::Halfedge_around_facet_circulator j = i->facet_begin();
			// Facets in polyhedral surfaces should be triangles.
			if( CGAL::circulator_size(j) != 3)
			{ 
				continue;
			}
			cgNewPolyhedron_3::Point_3 v0 =	j->vertex()->point();
			cgNewPolyhedron_3::Point_3 v1 = (++j)->vertex()->point();
			cgNewPolyhedron_3::Point_3 v2 = (++j)->vertex()->point(); 

			coords.push_back(CGAL::to_double(v0.x()));coords.push_back(CGAL::to_double(v0.y()));coords.push_back(CGAL::to_double(v0.z()));  
			coords.push_back(CGAL::to_double(v1.x()));coords.push_back(CGAL::to_double(v1.y()));coords.push_back(CGAL::to_double(v1.z())); 
			coords.push_back(CGAL::to_double(v2.x()));coords.push_back(CGAL::to_double(v2.y()));coords.push_back(CGAL::to_double(v2.z()));    

			vidx.push_back(pntNum++);
			vidx.push_back(pntNum++);
			vidx.push_back(pntNum++);  
		}
 

		polyhedron_builder<HalfedgeDS> builder( coords, vidx );
		out_PH.delegate( builder );

		if (out_PH.empty() || !out_PH.is_closed())
		{
			std::cout<<"error: undesired problem.\n ";
			return;
		}

	//}  
}

void CPolyHedron::compute_normals_per_facet()
{
	for (Facet_iterator f = this->facets_begin(); f != this->facets_end(); ++f)
	{ 
		cgPolyhedron::Facet::Normal_3 sum = CGAL::NULL_VECTOR;
		cgPolyhedron::Facet::Halfedge_around_facet_circulator h = f->facet_begin();
		do
		{
			cgPolyhedron::Facet::Normal_3 normal	=	CGAL::cross_product(
				h->next()->vertex()->point() - h->vertex()->point(),
				h->next()->next()->vertex()->point() - h->next()->vertex()->point());
			double sqnorm	=	normal * normal;
			if(sqnorm	!= 0)
				normal = normal	/	(float)std::sqrt(sqnorm);
			sum	=	sum	+	normal;
		}
		while(++h	!= f->facet_begin());
		double sqnorm = sum * sum;
		if(sqnorm	!= 0.0)
			f->normal() = sum / std::sqrt(sqnorm);
		else
		{
			f->normal() = CGAL::NULL_VECTOR;
			Logger::output("degenerate face\n");
		} 
	}
	
}

void CPolyHedron::compute_normals_per_vertex()
{
	for (Vertex_iteror v = this->vertices_begin();v!=this->vertices_end(); ++v )
	{ 
		cgPolyhedron::Vertex::Normal	normal = CGAL::NULL_VECTOR;
		Vertex::Halfedge_around_vertex_const_circulator	pHalfedge	=	v->vertex_begin();
		Vertex::Halfedge_around_vertex_const_circulator	begin	=	pHalfedge;
		CGAL_For_all(pHalfedge,begin)	
			if(!pHalfedge->is_border())
				normal = normal	+	pHalfedge->facet()->normal();
		double	sqnorm = normal * normal;
		if(sqnorm != 0.0f)
			v->normal() = normal	/	(float)std::sqrt(sqnorm);
		else
			v->normal() = CGAL::NULL_VECTOR;
	}
}

CPolyHedron * CPolyHedron::buildPolyhedron( std::vector<double> & vVtx, std::vector<int> & vIdx )
{
	CPolyHedron * Ph = new CPolyHedron();
	polyhedron_builder<HalfedgeDS> builder( vVtx, vIdx );
	Ph->delegate( builder );

	Ph->calculateVolume();
	Ph->compute_normals();
	Ph->buildBBoxForPH();

	Colorf c(1.0,0,0,1.0);
	Ph->set_color(c);
	Ph->toShow = true;
	Ph->onModel = false; 
	return Ph;
}

bool CPolyHedron::save_PHGroup_Obj( std::vector<CPolyHedron*> & obj, const char * filename, double bigTheta /*= 0*/ )
{
	std::vector<CVector3D> vPnt;
	std::vector<CVector3D> vNrl;
	std::vector<CVector3D> vClr;
	std::vector<std::vector<int>> vvIndx;
	int pntNum=0;
	int fctNum=0;
	for (int i=0;i<obj.size();i++)
	{
		CPolyHedron & ph = *obj[i]; 
		if ( ph.is_pure_quad ()) // if it is pure quad, split every facet to 2 triangles.
		{
			for ( Facet_iterator i = ph.facets_begin(); i != ph.facets_end(); ++i) 
			{
				phHalfedge_facet_circulator j = i->facet_begin();
				// Facets in polyhedral surfaces are at least triangles.
				if( CGAL::circulator_size(j) != 4)
				{ 
					continue;
				}
				cgPolyPoint_3 v0 =	j->vertex()->point();
				cgPolyPoint_3 v1 = (++j)->vertex()->point();
				cgPolyPoint_3 v2 = (++j)->vertex()->point();
				cgPolyPoint_3 v3 = (++j)->vertex()->point();

				CVector3D vec0 (CGAL::to_double(v0.x()),CGAL::to_double(v0.y()),CGAL::to_double(v0.z())); 
				CVector3D vec1 (CGAL::to_double(v1.x()),CGAL::to_double(v1.y()),CGAL::to_double(v1.z())); 
				CVector3D vec2 (CGAL::to_double(v2.x()),CGAL::to_double(v2.y()),CGAL::to_double(v2.z())); 
				CVector3D vec3 (CGAL::to_double(v3.x()),CGAL::to_double(v3.y()),CGAL::to_double(v3.z())); 
				vPnt.push_back(vec0);
				vPnt.push_back(vec1);
				vPnt.push_back(vec2);
				vPnt.push_back(vec3);
				CVector3D nrl = (vec0-vec1)^(vec2-vec1);
				nrl.normalize();
				vNrl.push_back(nrl);vNrl.push_back(nrl);vNrl.push_back(nrl);vNrl.push_back(nrl);
				CVector3D clor (ph.colour.r(),ph.colour.g(),ph.colour.b());
				vClr.push_back(clor);vClr.push_back(clor);vClr.push_back(clor);vClr.push_back(clor);
				std::vector<int> vidx;
				vidx.push_back(pntNum++);
				vidx.push_back(pntNum++);
				vidx.push_back(pntNum++);
				vidx.push_back(pntNum++);

				vvIndx.push_back(vidx);
				fctNum++;

			}
		}
		if ( ph.is_pure_triangle()) // if it is pure quad, split every facet to 2 triangles.
		{
			for ( Facet_iterator i = ph.facets_begin(); i != ph.facets_end(); ++i) 
			{
				phHalfedge_facet_circulator j = i->facet_begin();
				// Facets in polyhedral surfaces are at least triangles.
				if( CGAL::circulator_size(j) != 3)
				{ 
					continue;
				}
				cgPolyPoint_3 v0 =	j->vertex()->point();
				cgPolyPoint_3 v1 = (++j)->vertex()->point();
				cgPolyPoint_3 v2 = (++j)->vertex()->point(); 

				CVector3D vec0 (CGAL::to_double(v0.x()),CGAL::to_double(v0.y()),CGAL::to_double(v0.z())); 
				CVector3D vec1 (CGAL::to_double(v1.x()),CGAL::to_double(v1.y()),CGAL::to_double(v1.z())); 
				CVector3D vec2 (CGAL::to_double(v2.x()),CGAL::to_double(v2.y()),CGAL::to_double(v2.z()));  
				vPnt.push_back(vec0);
				vPnt.push_back(vec1);
				vPnt.push_back(vec2);
				CVector3D nrl = (vec0-vec1)^(vec2-vec1);
				nrl.normalize();
				vNrl.push_back(nrl);vNrl.push_back(nrl);vNrl.push_back(nrl); 
				CVector3D clor (ph.colour.r(),ph.colour.g(),ph.colour.b());
				vClr.push_back(clor);vClr.push_back(clor);vClr.push_back(clor); 
				std::vector<int> vidx;
				vidx.push_back(pntNum++);
				vidx.push_back(pntNum++);
				vidx.push_back(pntNum++); 

				vvIndx.push_back(vidx);
				fctNum++;

			}
		}
	}  
	if (bigTheta != 0 )
	{
		for (int i=0;i<vPnt.size();i++)
		{ 
			CVector3D & pnt = vPnt[i];
			CPointCloud::trBackSingPnt(pnt,bigTheta,TR_XY);
			CVector3D & nrl = vNrl[i];
			CPointCloud::trBackSingPnt(nrl,bigTheta,TR_XY);
		}
	}

	FILE * m_pFile;
	fopen_s( & m_pFile, filename, "w" );
	fprintf_s( m_pFile, "#\n" );

	for ( int i = 0; i < ( int )vPnt.size(); i++ ) 
	{
		fprintf_s( m_pFile, "v %.8f %.8f %.8f\n", vPnt[ i ].x_(), vPnt[ i ].y_(), vPnt[ i ].z_() );
	}

	for ( int i = 0; i < ( int )vvIndx.size(); i++ ) 
	{ 
		std::vector<int> & vIdx = vvIndx[i];
		fprintf_s( m_pFile, "f ");
		for (int fi =0;fi<vIdx.size();fi++)
		{
		// In obj formate file, vertex index starts from 1. 
			fprintf_s( m_pFile, "%d ",vIdx[fi]+1);
		}
		fprintf_s( m_pFile, "\n");
	} 

	fclose( m_pFile );
	return true;
} 



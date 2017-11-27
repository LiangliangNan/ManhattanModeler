
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

#include "math_global.h"
#include "math_types.h"
#include "color.h" 
#include ".\boundingbox.h"

#include <CGAL/Exact_predicates_exact_constructions_kernel.h> 
#include <CGAL/Nef_polyhedron_3.h> 
typedef CGAL::Exact_predicates_exact_constructions_kernel	nefKernel; 
typedef CGAL::Polyhedron_3<nefKernel>						cgNewPolyhedron_3;
typedef CGAL::Nef_polyhedron_3<nefKernel>					cgNef_polyhedron_3;
typedef nefKernel::Vector_3									newVector3f;



class CPolyHedron : public cgPolyhedron  //CGAL::Polyhedron_3<FloatKernel,Enriched_items>
{
public: 
	CPolyHedron(cgPolyhedron & ph);
	CPolyHedron(const CPolyHedron & cPH);
	CPolyHedron();


	~CPolyHedron(){};

	CBoundingBox *m_bbox; 
	cgNef_polyhedron_3 m_nefPolyHedron;
	Colorf colour;
	bool toShow;
	bool onModelTotally;	// this bool indicates the cube is inside a closed mesh.
	bool onModelMaybe;		// this bool indicates the cube may be on the model, depending on the facet points.
	bool onModel;// indicate whether this polyhedron is on the final model result.
	   
private:
	float m_volume;
public: 

	bool m_hasNormal;
	// construct a polyhedron using vertices and facet indexes. 
	// vVtx = [x0,y0,z0,...xn,yn,zn] vIdx = [Tri01,Tri02,Tri03,...,Trin1,Trin2,Trin3]
	static CPolyHedron * buildPolyhedron(std::vector<double> & vVtx, std::vector<int> & vIdx); 

	// normals (per	facet, then	per	vertex)
	void compute_normals()
	{
		compute_normals_per_facet();
		compute_normals_per_vertex();
	}

	void set_color(Colorf clr){colour.set(clr.r(),clr.g(),clr.b());}
	void set_show(bool show_or_not){toShow=show_or_not;}
	float getVolum()
	{
		return m_volume;
	};
	 
	void drawPolyhedron(float transparentRatio=1.0f);
	bool read_Polyhedron_From_OFF (const char * file_name);
	bool read_Polyhedron_From_obj( const char *filename );
	//static bool checkIntersect(CPolyHedron & ph1, CPolyHedron & ph2);

	//typedef typename cgPolyhedron::Halfedge_handle Halfedge_handle;
	static cgNewPolyhedron_3::Halfedge_handle CPolyHedron::make_cube_3( cgNewPolyhedron_3 & P, cgNewPolyhedron_3::Point_3 &p0,cgNewPolyhedron_3::Point_3 &p1,cgNewPolyhedron_3::Point_3 &p2,cgNewPolyhedron_3::Point_3 &p3, cgNewPolyhedron_3::Point_3 &p4,cgNewPolyhedron_3::Point_3 &p5,cgNewPolyhedron_3::Point_3 &p6,cgNewPolyhedron_3::Point_3 &p7 );
	static void make_cube_3(cgPolyhedron & P, const std::vector<cgPoint3f>& cube8Pnts) ;
	static void make_cube_3( cgPolyhedron & P, cgPolyhedron::Point_3 &p0,cgPolyhedron::Point_3 &p1,cgPolyhedron::Point_3 &p2,cgPolyhedron::Point_3 &p3, cgPolyhedron::Point_3 &p4,cgPolyhedron::Point_3 &p5,cgPolyhedron::Point_3 &p6,cgPolyhedron::Point_3 &p7 );

	static void convert_PH_kernel(cgNewPolyhedron_3 & in_PH, cgPolyhedron & out_PH);

	static bool save_PHGroup_Ply( std::vector<CPolyHedron*> & obj, const char * filename );
	static bool save_PHGroup_Ply( std::vector<CPolyHedron*> & obj, const char * filename, double bigTheta ); 
	static bool save_PHGroup_Obj( std::vector<CPolyHedron*> & obj, const char * filename, double bigTheta = 0);

private:

	void calculateVolume();
	void savePolyHedron_to_OFF(std::ofstream & out, cgPolyhedron & P );
	void buildBBoxForPH(); 


	void compute_normals_per_facet();
	void compute_normals_per_vertex();
};

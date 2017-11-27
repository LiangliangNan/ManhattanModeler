
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


#include "CPointCloud.h"
#include <iostream>
#include <string>
#include <sstream>
#include <cassert>
#include <gl/gl.h>

#include "logger.h"
#include "math_global.h"
#include "PlyLoad.h"
#include "rply.h"
#include "Parameters.h" 

#include "CMeshObj.h"  
#include "ProgressBar.h"

#include <time.h>
 



#include <CGAL/linear_least_squares_fitting_3.h> 
 
//template <typename T>
//void delete_pointed_to(T* const ptr)
//{
//	delete ptr;
//} 
CPointCloud::CPointCloud(void):m_ptrKDTree(nil),
								m_cBoundingBox(nil), 
								m_aabbTree(nil), 
								m_MeshObj(nil)/*,labeling(nil)*/
								//m_OctNodePC(nil), 
{
	this->toShow = true; 
	this->m_bSelect= false;
	this->showHeightColor = false;
	this->hasMW = false;
}

CPointCloud::~CPointCloud(void)
{
	clearAll();
	//// free memory
	//std::for_each(c.begin(), c.end(), delete_pointed_to<base>);

}
CPointCloud & CPointCloud::operator=( const CPointCloud & PC_ )
{
	clearAll();
	if (PC_.m_vtxPC.size()>0)
	{
		this->m_vtxPC.insert(m_vtxPC.end(),PC_.m_vtxPC.begin(),PC_.m_vtxPC.end()); 
	}  
	this->build_boundingBox();
	return (*this);
}

void CPointCloud::clearAll()
{ 

	for (std::vector<CScanVertex *>::iterator it=this->m_vtxPC.begin(); it!=this->m_vtxPC.end(); it++) 
	{  
		delete *it; 
	}
	m_vtxPC.clear(); 

	deletKDtree(); 
	if (m_cBoundingBox!=nil)
	{
		delete m_cBoundingBox;
		m_cBoundingBox = nil;
	}  
	if (m_MeshObj!=NULL)
	{
		delete m_MeshObj;
		m_MeshObj = NULL;
	}  
}

void CPointCloud::deletKDtree()
{
	if (m_ptrKDTree) 
	{
		delete m_ptrKDTree;
		m_ptrKDTree = nil;
	} 
} 
bool CPointCloud::save_ply(const char * file_name) 
{ 
	p_ply ply = ply_create(file_name, PLY_LITTLE_ENDIAN, nil, 0, nil) ;

	if(ply == nil) 
	{
		return false ;
	}


	int num_v = m_vtxPC.size(); 
	if (!ply_add_element(ply, "vertex", num_v)) 
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

	// 		std::vector<Facet*> faces;
	// 		if (object_->object_type() == OBJ_MESH) {
	// 			const PolyMesh* mesh = dynamic_cast<const PolyMesh*>(object_);
	// 			faces = mesh->faces();
	// 			unsigned int num_f = faces.size();
	// 			if (!ply_add_element(ply, "face", num_f)) {
	// 				Logger::output("PlySave") << "unable to add element \'face\'" << Logger::endl();
	// 				ply_close(ply) ;
	// 				return false ;
	// 			}
	// 			if (!ply_add_property(ply, "vertex_indices", PLY_LIST, PLY_UCHAR, PLY_INT)) {
	// 				Logger::output("PlySave") << "unable to add property \'vertex_indices\'" << Logger::endl();
	// 				ply_close(ply) ;
	// 				return false ;
	// 			}
	// 		}

	if(!ply_write_header(ply)) 
	{
		ply_close(ply) ;
		return false ;
	}

	//////////////////////////////////////////////////////////////////////////
	ProgressLogger progress(num_v);  
	for (int i=0; i<num_v; ++i) 
	{  
		progress.notify(i);
		if (progress.is_canceled())
			break;
		ply_write(ply, m_vtxPC[i]->point_.x_());
		ply_write(ply, m_vtxPC[i]->point_.y_());
		ply_write(ply, m_vtxPC[i]->point_.z_());

		ply_write(ply, m_vtxPC[i]->normal_.x_());
		ply_write(ply, m_vtxPC[i]->normal_.y_());
		ply_write(ply, m_vtxPC[i]->normal_.z_());
		if (m_vtxPC[i]->color_error!=NULL && lml_param::hidePoisson)
		{
			int cr = (int)(m_vtxPC[i]->color_error->r()*255)%256;
			int cg = (int)(m_vtxPC[i]->color_error->g()*255)%256;
			int cb = (int)(m_vtxPC[i]->color_error->b()*255)%256;
			ply_write(ply,cr);
			ply_write(ply,cg);
			ply_write(ply,cb); 
		}
		else{ 
			int cr = (int)(m_vtxPC[i]->color_.r()*255)%256;
			int cg = (int)(m_vtxPC[i]->color_.g()*255)%256;
			int cb = (int)(m_vtxPC[i]->color_.b()*255)%256;
			ply_write(ply,cr);
			ply_write(ply,cg);
			ply_write(ply,cb); 
		}
		
	}

	// 		for (unsigned int i=0; i<faces.size(); ++i) {
	// 			const Facet* f = faces[i];
	// 			const std::vector<ScanVertex*>&  vts = f->vertices();
	// 			ply_write(ply, vts.size());
	// 			for (unsigned int j=0; j<vts.size(); ++j) {
	// 				ply_write(ply, vts[j]->idx());
	// 			}
	// 		}

	ply_close(ply);
	return true ;
}


void CPointCloud::load_ply( const char * filename, CPointCloud * ptrPC )
{
	CPlyLoad plyload_;
	plyload_.loadPly(filename,ptrPC);
	
	//if (ptrPC->save_ply(filename))
	//{
	//	delete ptrPC;
	//}
	
	return;
} 

void CPointCloud::build_boundingBox()
{
	if (m_cBoundingBox) 
		m_cBoundingBox->Reset(); 
	else
		m_cBoundingBox = new CBoundingBox();

	for (int i=0;i<m_vtxPC.size();i++)
	{
		this->m_cBoundingBox->Push(m_vtxPC[i]->point_);
	}

	/*std::cout<<"bounding box corner:\t"<<m_cBoundingBox.m_vMax.x_()<<" "<< m_cBoundingBox.m_vMax.y_()<<" "<< m_cBoundingBox.m_vMax.z_()
		<<"\n\t\t"<<m_cBoundingBox.m_vMin.x_()<<" "<<m_cBoundingBox.m_vMin.y_()<<" "<< m_cBoundingBox.m_vMin.z_()<<"\n";*/
}

void CPointCloud::build_kd_tree( int nMaxBucketSize /* = 16 */ )
{
	unsigned int num = this->m_vtxPC.size();
	if (num == 0)
	{
		m_ptrKDTree = nil;
		return;
	}

	kdtree::Vector3D* points = new kdtree::Vector3D[num];

	for (int i=0; i<num; ++i) 
	{ 
		points[i].x = m_vtxPC[i]->point_.x_();
		points[i].y = m_vtxPC[i]->point_.y_();
		points[i].z = m_vtxPC[i]->point_.z_();
	}

	m_ptrKDTree = new kdtree::KdTree( points, num, nMaxBucketSize );
	delete points;
}
void  CPointCloud::get_1_nearest_CVector3D( CVector3D & vQuery, CVector3D & result)  
{
	if (!m_ptrKDTree)
		build_kd_tree();

	kdtree::Vector3D v3d( vQuery.x_(), vQuery.y_(), vQuery.z_() );
	m_ptrKDTree->setNOfNeighbours( 1 );
	m_ptrKDTree->queryPosition( v3d ); 

	unsigned int numK = m_ptrKDTree->getNOfFoundNeighbours();

	if (numK != 1)
	{
		Logger::output("error: not found 1 nearest point.\n");
		return;
	}
	for( unsigned int i=0; i<numK; ++i)
	{
		result = m_vtxPC[ m_ptrKDTree->getNeighbourPositionIndex(i) ]->point_; //  also get the distance by calling GetNearestPointDistance( i ); // or the square of the distance by calling GetNearestPointDistanceSq( i );
	} 
} 
 
unsigned int CPointCloud::get_K_nearest_points( CScanVertex * vQuery, std::vector<CScanVertex*>& neighbors, int K /*=30*/ )  
{
	if (!m_ptrKDTree)
		build_kd_tree();

	kdtree::Vector3D v3d( vQuery->point_.x_(), vQuery->point_.y_(), vQuery->point_.z_() );
	m_ptrKDTree->setNOfNeighbours( K );
	m_ptrKDTree->queryPosition( v3d ); 

	unsigned int numK = m_ptrKDTree->getNOfFoundNeighbours();

	neighbors.resize(numK);
	for( unsigned int i=0; i<numK; ++i)
	{
		neighbors[i] = m_vtxPC[ m_ptrKDTree->getNeighbourPositionIndex(i) ]; //  also get the distance by calling GetNearestPointDistance( i ); // or the square of the distance by calling GetNearestPointDistanceSq( i );
	}
	return numK;
}

unsigned int CPointCloud::get_K_nearest_points(CScanVertex * vQuery, std::vector<CScanVertex*> & result,  int K /*= 30*/, double fRange /*= 0.5*/) 
{
	if (!m_ptrKDTree)
		build_kd_tree();

	kdtree::Vector3D v3d(vQuery->point_.x_(), vQuery->point_.y_(), vQuery->point_.z_());
	m_ptrKDTree->setNOfNeighbours( K );
	m_ptrKDTree->queryRange( v3d, fRange*fRange );

	unsigned int numK = m_ptrKDTree->getNOfFoundNeighbours();

	result.resize(numK);
	for( unsigned int i=0; i<numK; ++i)
	{
		result[i] = m_vtxPC[ m_ptrKDTree->getNeighbourPositionIndex(i) ];
		// you may also get the distance by calling GetNearestPointDistance( i );
		// or the square of the distance by calling GetNearestPointDistanceSq( i );
	}
	return numK;
}

unsigned int CPointCloud::get_K_nearest_pntIdx( CVector3D & vQuery, std::vector<int>& vPositionIndex, int K /*=30*/ )  
{
	if (!m_ptrKDTree)
		build_kd_tree();

	kdtree::Vector3D v3d( vQuery.x_(), vQuery.y_(), vQuery.z_() );
	m_ptrKDTree->setNOfNeighbours( K );
	m_ptrKDTree->queryPosition( v3d ); 

	unsigned int numK = m_ptrKDTree->getNOfFoundNeighbours();

	vPositionIndex.resize(numK);
	for( unsigned int i=0; i<numK; ++i)
	{
		vPositionIndex[i] = m_ptrKDTree->getNeighbourPositionIndex(i); //  also get the distance by calling GetNearestPointDistance( i ); // or the square of the distance by calling GetNearestPointDistanceSq( i );
	}
	return numK;
}
unsigned int CPointCloud::get_K_nearest_points( CVector3D &vQuery, std::vector<CScanVertex*> & neighbors, int K /*= 10*/, double fRange/* = -1*/)
{
	if (!m_ptrKDTree)
		build_kd_tree();

	kdtree::Vector3D v3d( vQuery.x_(), vQuery.y_(), vQuery.z_() );
	m_ptrKDTree->setNOfNeighbours( K );
	if (fRange == -1)
	{
		m_ptrKDTree->queryPosition( v3d ); 
	}
	else
	{ 
		m_ptrKDTree->queryRange( v3d, fRange*fRange );
	}
	

	unsigned int numK = m_ptrKDTree->getNOfFoundNeighbours();

	neighbors.resize(numK);
	for( unsigned int i=0; i<numK; ++i)
	{
		neighbors[i] = m_vtxPC[ m_ptrKDTree->getNeighbourPositionIndex(i) ]; //  also get the distance by calling GetNearestPointDistance( i ); // or the square of the distance by calling GetNearestPointDistanceSq( i );
	}
	return numK;
}

 

void CPointCloud::drawPC( float pntSize /*,int interval*/ /*= 1*/ )
{
	if (!toShow)
	{
		return;
	} 

	unsigned int nb_vertices =m_vtxPC.size(); 

	glPointSize(pntSize); 

	glBegin(GL_POINTS);
	int interval = 1 ;
	for (int i=0; i<nb_vertices; i+=interval) 
	{ 
		if (m_vtxPC[i]->color_error!=NULL && lml_param::hidePoisson)
		{
			glColor3d(m_vtxPC[i]->color_error->r(), m_vtxPC[i]->color_error->g(), m_vtxPC[i]->color_error->b());
		}
		else if(m_vtxPC[i]->color_height!=NULL && this->showHeightColor)
		{
			glColor3d(m_vtxPC[i]->color_height->r(),m_vtxPC[i]->color_height->g(),m_vtxPC[i]->color_height->b());
		}
		else if (m_vtxPC[i]->selected_)
		{
			glColor3d(1,0, 0);
		}
		else
		{
			glColor3d(m_vtxPC[i]->color_.r(), m_vtxPC[i]->color_.g(), m_vtxPC[i]->color_.b());
		} 
		glNormal3d(m_vtxPC[i]->normal_.x_(), m_vtxPC[i]->normal_.y_(), m_vtxPC[i]->normal_.z_());
		glVertex3d(m_vtxPC[i]->point_.x_(), m_vtxPC[i]->point_.y_(), m_vtxPC[i]->point_.z_());
	}
	glEnd();  
}

void CPointCloud::drawLabels()
{
	if (this->m_vecLabels.size()>0)
	{ 
		glPointSize(lml_param::showPointSize); 

		glBegin(GL_POINTS);
		int interval = 1 ;
		for (int i=0; i<this->m_vtxPC.size(); i+=interval) 
		{  
			int idx = m_vecLabels[i];
			glColor3d(m_colorTab[idx].r(), m_colorTab[idx].g(), m_colorTab[idx].b());

			glNormal3d(m_vtxPC[i]->normal_.x_(), m_vtxPC[i]->normal_.y_(), m_vtxPC[i]->normal_.z_());
			glVertex3d(m_vtxPC[i]->point_.x_(), m_vtxPC[i]->point_.y_(), m_vtxPC[i]->point_.z_());
		}
		glEnd();   
	}
}
 

void CPointCloud::trSinglePnt( CVector3D & m_vecPoint, double cos_theta, double sin_theta, CoordTranferMode mode)
{ 
	unsigned int id0, id1;
	switch (mode)
	{
	case TR_XY:
		id0=0;
		id1=1;
		break;
	case TR_YZ:
		id0=1;
		id1=2;
		break;
	case  TR_ZX:
		id0=2;
		id1=0;
		break;
	}
	double xsrc=m_vecPoint.pVec[id0];
	double ysrc=m_vecPoint.pVec[id1];
	m_vecPoint.pVec[id0]=cos_theta*xsrc+sin_theta*ysrc;
	m_vecPoint.pVec[id1]=-sin_theta*xsrc+cos_theta*ysrc; 
}
void CPointCloud::trSinglePnt( CVector3D & m_vecPoint, double theta,CoordTranferMode mode)
{ 
	double cos_theta, sin_theta;

	cos_theta=cos(theta);//-3.14159265359/2);
	sin_theta=sin(theta);//-3.14159265359/2);
	unsigned int id0, id1;
	switch (mode)
	{
	case TR_XY:
		id0=0;
		id1=1;
		break;
	case TR_YZ:
		id0=1;
		id1=2;
		break;
	case  TR_ZX:
		id0=2;
		id1=0;
		break;
	}
	double xsrc=m_vecPoint.pVec[id0];
	double ysrc=m_vecPoint.pVec[id1];
	m_vecPoint.pVec[id0]=cos_theta*xsrc+sin_theta*ysrc;
	m_vecPoint.pVec[id1]=-sin_theta*xsrc+cos_theta*ysrc; 
}
void CPointCloud::trBackSingPnt( CVector3D & m_vecPoint, double cos_theta, double sin_theta, CoordTranferMode mode)
{  
	unsigned int id0, id1;
	switch (mode)
	{
	case TR_XY:
		id0=0;
		id1=1;
		break;
	case TR_YZ:
		id0=1;
		id1=2;
		break;
	case  TR_ZX:
		id0=2;
		id1=0;
		break;
	}
	double xsrc=m_vecPoint.pVec[id0];
	double ysrc=m_vecPoint.pVec[id1];
	m_vecPoint.pVec[id0]=cos_theta*xsrc-sin_theta*ysrc;
	m_vecPoint.pVec[id1]=sin_theta*xsrc+cos_theta*ysrc;  
}
void CPointCloud::trBackSingPnt( CVector3D & m_vecPoint, double  theta,  CoordTranferMode mode)
{  
	double cos_theta, sin_theta;

	cos_theta=cos(theta);//-3.14159265359/2);
	sin_theta=sin(theta);//-3.14159265359/2);
	unsigned int id0, id1;
	switch (mode)
	{
	case TR_XY:
		id0=0;
		id1=1;
		break;
	case TR_YZ:
		id0=1;
		id1=2;
		break;
	case  TR_ZX:
		id0=2;
		id1=0;
		break;
	}
	double xsrc=m_vecPoint.pVec[id0];
	double ysrc=m_vecPoint.pVec[id1];
	m_vecPoint.pVec[id0]=cos_theta*xsrc-sin_theta*ysrc;
	m_vecPoint.pVec[id1]=sin_theta*xsrc+cos_theta*ysrc;  
}
void CPointCloud::transPntCloud( CPointCloud & pc, double theta, CoordTranferMode mode )
{
	double cos_theta, sin_theta;

	cos_theta=cos(theta);//-3.14159265359/2);
	sin_theta=sin(theta);//-3.14159265359/2);
	if (pc.m_cBoundingBox) 
		pc.m_cBoundingBox->Reset(); 
	else
		pc.m_cBoundingBox = new CBoundingBox();
	
	int pntNb=pc.m_vtxPC.size();
	for (int i=0;i<pntNb;i++)
	{
		CVector3D & pnt=pc.m_vtxPC[i]->point_;
		trSinglePnt(pnt, cos_theta, sin_theta,mode); 
		pc.m_cBoundingBox->Push(pnt);

		CVector3D & nrl=pc.m_vtxPC[i]->normal_;
		trSinglePnt(nrl,cos_theta, sin_theta,mode);
	}

}

void CPointCloud::transBackPntCloud( CPointCloud & pc, double theta, CoordTranferMode mode  )
{
	double cos_theta, sin_theta;

	cos_theta=cos(theta);//-3.14159265359/2);
	sin_theta=sin(theta);//-3.14159265359/2);

	if (pc.m_cBoundingBox) 
		pc.m_cBoundingBox->Reset(); 
	else
		pc.m_cBoundingBox = new CBoundingBox();

	int pntNb=pc.m_vtxPC.size();
	for (int i=0;i<pntNb;i++)
	{
		CVector3D & pp=pc.m_vtxPC[i]->point_;
		trBackSingPnt(pp, cos_theta,sin_theta,mode); 
		pc.m_cBoundingBox->Push(pp);
		CVector3D & pn=pc.m_vtxPC[i]->normal_;
		trBackSingPnt(pn,cos_theta,sin_theta,mode);
	}
}


void inline det2(double a,double b,double c,double d,double& dReturn)
{
	dReturn=(a*d)-(c*b);
	return;
}
double inline det3(double a1,double b1,double c1,
				   double a2,double b2,double c2,
				   double a3,double b3,double c3 )
{
	//		| a1 b1 c1 |
	//		| a2 b2 c2 |
	//		| a3 b3 c3 |
	double dAone, dAfour, dAseven;
	det2(b2,c2,b3,c3,dAone); 
	det2(a2,c2,a3,c3,dAfour); 
	det2(a2,b2,a3,b3,dAseven); 

	double result=(a1*dAone)-(b1*dAfour)+(c1*dAseven);
	return result;
}
void CPointCloud::calculateXYZ(cgPlane3f & abcd1, cgPlane3f &abcd2, cgPlane3f &abcd3, CVector3D & interscPnt)
{
	// CGAL plane: a*x+b*y+c*z+d=0

	//	a1 b1 c1    x   d1 
	//	a2 b2 c2  * y = d2 
	//	a3 b3 c3    z   d3 

	//		| d1 b1 c1 |
	//		| d2 b2 c2 |
	//		| d3 b3 c3 |
	//	x = --------------
	//			d

	//		| a1 d1 c1 |
	//		| a2 d2 c2 |
	//		| a3 d3 c3 |
	//	y = --------------
	//			d

	//		| a1 b1 d1 |
	//		| a2 b2 d2 |
	//		| a3 b3 d3 |
	//	z = --------------
	//			d

	//		| a1 b1 c1 |
	//	d = | a2 b2 c2 |
	//		| a3 b3 c3 |

	// plane: a*x+b*y+c*z+d=0 -> a*x+b*y+c*z=-d

	double a1=abcd1.a();double b1=abcd1.b();double c1=abcd1.c();double d1=-abcd1.d();
	double a2=abcd2.a();double b2=abcd2.b();double c2=abcd2.c();double d2=-abcd2.d();
	double a3=abcd3.a();double b3=abcd3.b();double c3=abcd3.c();double d3=-abcd3.d();

	double x,y,z;
	double d=det3(a1,b1,c1,a2,b2,c2,a3,b3,c3);
	x=det3(d1,b1,c1,d2,b2,c2,d3,b3,c3)/d;
	y=det3(a1,d1,c1,a2,d2,c2,a3,d3,c3)/d;
	z=det3(a1,b1,d1,a2,b2,d2,a3,b3,d3)/d;
	interscPnt.pVec[0]=x;
	interscPnt.pVec[1]=y;
	interscPnt.pVec[2]=z;
}

 

void CPointCloud::smooth_PC_By_Range( const double pRange, unsigned int k /*= 8*/ )
{
	for (int i =0; i<this->m_vtxPC.size(); i++)
	{
		//std::cout<<i<<" ";
		CScanVertex * pnt = this->m_vtxPC[i];
		std::vector<CScanVertex * > result;
		int sz = get_K_nearest_points(pnt,result,k,pRange);
		if (sz<3)
		{
			//for (int si =0;si<result.size();si++)
			//{
			// delete result[si]; 
			//}
			result.clear();
			continue;
		}
		else
		{
			CVector3D pos (0,0,0);
			for (int j=0;j<result.size();j++)
			{
				pos += result[j]->point_;
			}
			pos = pos / result.size();
			pnt->point_ = pos;

			//for (int si =0;si<result.size();si++)
			//{
			// delete result[si]; 
			//}
			result.clear();
		}
	}
	Logger::output("Accomplish Smooth pc.\n");
} 
 
CMeshObj * CPointCloud::gen_meshFromXYZ_idx(std::vector<double> &pntxyz, std::vector<int> &faceIdx012)
{
	CMeshObj * mesh = new CMeshObj();
	for (int i=0;i<pntxyz.size();i+=3)
	{
		CVector3D tempPt (pntxyz[i+0],pntxyz[i+1],pntxyz[i+2]);
		mesh->pushOneVertex(tempPt);
	}
	 for (int i=0; i<faceIdx012.size();i+=3)
	 {
		 mesh->PushOneFace(faceIdx012[i],faceIdx012[i+1],faceIdx012[i+2]);
		 CVector3D & p0 = mesh->m_vecVertex[faceIdx012[i+0]].v;
		 CVector3D & p1 = mesh->m_vecVertex[faceIdx012[i+1]].v;
		 CVector3D & p2 = mesh->m_vecVertex[faceIdx012[i+2]].v;
		 CVector3D nrl = (p2-p1)^(p0-p1);
		 nrl.normalize();
		 mesh->m_vecTriNrl.push_back(nrl);
		 CVector3D midPt = 0.333*(p0+p1+p2);
		 mesh->m_vecMeshMidPnt.push_back(midPt);
		 if (nrl.z_()<0.05) 
			 mesh->m_vecElemType.push_back(T_WALL); 
		 else
			 mesh->m_vecElemType.push_back(T_ROOF);
	 }
	 return mesh;
}

 
#include "CVertexGroup.h"
#include "math_types.h"

#include <CGAL/linear_least_squares_fitting_3.h> 

#include <gl/gl.h>


CVertexGroup::CVertexGroup():m_toshow(true) 
{
	m_color = Colorf(0,0,0,1);
}
CVertexGroup::CVertexGroup(std::vector<CScanVertex *> * vVetexs):m_toshow(true) 
{
	m_color = Colorf(0,0,0,1);
	mergeGroup(vVetexs);
} 
CVertexGroup::~CVertexGroup() 
{
	//if (this->size()>0)
	//{
	//	for (std::vector<CScanVertex*>::iterator it = this->begin(); it!=this->end(); it++)
	//	{
	//		delete * it;
	//	}
	//	this->clear();
	//}
	//if (this->m_pnts.size()>0)
	//{
	//	//for (std::vector<CScanVertex*>::iterator it = this->m_pnts.begin(); it!=this->m_pnts.end(); it++)
	//	//{
	//	//	delete * it;
	//	//}
	//	this->m_pnts.clear();
	//}
}

void CVertexGroup::resetBBox()
{
	for (int i =0;i<this->size();i++)
	{
		m_bbox.Push(this->at(i)->point_);
	}
}

void CVertexGroup::drawGroup(unsigned int fast)
{ 
	if (!m_toshow)
		return;
 
	glColor4fv(m_color.data());

	glBegin(GL_POINTS);	
	for (unsigned int j=0; j<size(); j+= fast) 
	{
		CVector3D & n = at(j)->normal_;
		glNormal3d(n.x_(), n.y_(), n.z_());
		CVector3D & p = at(j)->point_;
		glVertex3d(p.x_(), p.y_(), p.z_());			
	}
	glEnd(); 
}

void CVertexGroup::mergeGroup(CVertexGroup * anotherGroup)
{
	this->insert(this->end(),anotherGroup->begin(),anotherGroup->end());
	this->m_pnts.insert(this->m_pnts.end(), anotherGroup->m_pnts.begin(),anotherGroup->m_pnts.end());
	updateMidPnt();
}

void CVertexGroup::mergeGroup(std::vector<CScanVertex *> * anotherGroup)
{
	this->insert(this->end(),anotherGroup->begin(),anotherGroup->end()); 
	updateMidPnt();
}

// to fit the plane from the vector "m_pnts", e.g. refresh the plane model parameters.
int CVertexGroup::refitPln_least_squares()
{
	if (this->m_pnts.size()<3)
	{
		std::cout<<"error: not enough points for plan fitting!\n";
		return 0;
	}

	std::vector<cgPoint3f> pnts;
	for (std::vector<CScanVertex*>::iterator it = this->m_pnts.begin(); it != this->m_pnts.end(); it++)
	{
		CVector3D & pt = (*it)->point_;
		cgPoint3f cgPnt = cgPoint3f(pt.x_(),pt.y_(),pt.z_());
		pnts.push_back(cgPnt);
	}

	// least square fit plane and detect the centroid point.
	cgPoint3f centrPnt;
	cgPlane3f pln;
	int qualityScore = linear_least_squares_fitting_3(pnts.begin(),pnts.end(),pln,centrPnt,CGAL::Dimension_tag<0>());
	pnts.clear();

	// modify the plane model of current group and centroid point.
	//double a = pln.a(), b = pln.b(), c = pln.c(), d = pln.d();
	//cgPlane3f newPln = cgPlane3f(a,b,c,d);

	if (this->m_cgPlane.orthogonal_vector()*pln.orthogonal_vector() >0) 
		this->m_cgPlane = pln; 
	else
		this->m_cgPlane = pln.opposite();

	this->m_midPnt = CVector3D(centrPnt.x(),centrPnt.y(),centrPnt.z());

	refreshProjectionVtx();

	return qualityScore;
}

void CVertexGroup::refreshProjectionVtx()
{
	this->clearProjection(); // clear current projection points.

	// replace the projection points with new projection points.
	int num = this->m_pnts.size();
	for (int i=0; i< num; i++)
	{
		CScanVertex & vtx = *(this->m_pnts[i]);
		cgPoint3f pnt = cgPoint3f(vtx.point_.x_(),vtx.point_.y_(),vtx.point_.z_());

		cgPoint3f cgP_dst=this->m_cgPlane.projection(pnt);
		CVector3D tmpPnt(cgP_dst.x(),cgP_dst.y(),cgP_dst.z()); 
		CVector3D tempNrl(this->m_cgPlane.a(),this->m_cgPlane.b(),this->m_cgPlane.c());
		CScanVertex * tempV = new CScanVertex(tmpPnt,tempNrl,this->m_color);

		this->push_back(tempV);
	}
}

void CVertexGroup::clearProjection()
{
	if (this->size()>0)
	{
		this->clear();
	}
}

bool CVertexGroup::check_add( CScanVertex * vtx, double suqareDistThresh, double nrlThreshold)
{
	CVector3D nrl = CVector3D(this->m_cgPlane.a(),this->m_cgPlane.b(),this->m_cgPlane.c());
	if (vtx->normal_*nrl < nrlThreshold)
	{
		return false;
	}

	cgPoint3f pnt = cgPoint3f(vtx->point_.x_(),vtx->point_.y_(),vtx->point_.z_());
	cgPoint3f cgP_dst=this->m_cgPlane.projection(pnt);
	float distance = CGAL::squared_distance(pnt, cgP_dst);
	if (distance < suqareDistThresh)
	{
		this->m_pnts.push_back(vtx);
		CVector3D tmpPnt(cgP_dst.x(),cgP_dst.y(),cgP_dst.z()); 
		CVector3D tempNrl(this->m_cgPlane.a(),this->m_cgPlane.b(),this->m_cgPlane.c());
		CScanVertex * tempV = new CScanVertex(tmpPnt,tempNrl,this->m_color);

		this->push_back(tempV);
		return true;
	}
	else
		return false;
}

void CVertexGroup::updateMidPnt()
{
	if (this->size()<1)
		return;
	CVector3D midPnt (0,0,0);
	int sz = 0;
	for (int i=0;i<this->size();i++)
	{
		CVector3D & aPt = this->at(i)->point_;
		midPnt+=aPt;
		sz++;
	}
	m_midPnt = midPnt/sz;
	CVector3D & nor = this->at(0)->normal_; // in a plane all points has the same normal.
	m_cgPlane = cgPlane3f(cgPoint3f(m_midPnt.x_(), m_midPnt.y_(),  m_midPnt.z_()),
		cgVector3f(nor.x_(), nor.y_(), nor.z_()));  
}

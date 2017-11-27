
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


#include "CCube.h"
#include "Parameters.h"
#include <gl/gl.h>



CCube::CCube( CPolyHedron* cp ): m_CPolyHedron(cp) ,m_pc(NULL)
{
	m_6CoverRate.resize(6,0);
	m_numPntOnCube = 0;
	m_validGridNum = 0;
	m_score = 0;
}


CCube::~CCube(void)
{
	if ( this->m_CPolyHedron != NULL)
	{
		this->m_CPolyHedron = NULL;
	}
}

void CCube::dist_pnts_on6facet()
{
	dist_6FacetCorner(); 

	std::vector<std::vector<CVector3D*>> & coner6x4 = this->m_6fc4corner; // six facets, each facet consists of 4 vertexes.

	for (unsigned int j=0; j<6; j++) // loop plane
	{ 
		int aFacePntNum = 0;   
		std::vector<CScanVertex *> tempGroup;

		CVertexGroup * pcGroup_ij = m_v6Plns[j];  

		CVector3D & v0 = *(coner6x4[j][0]); 
		CVector3D & v1 = *(coner6x4[j][1]); 
		CVector3D & v2 = *(coner6x4[j][2]); 
		CVector3D & v3 = *(coner6x4[j][3]);  

		// the direction of a facet.
		CVector3D facetNrl = (v0-v1)^(v2-v1); 
		// calculate the facet area.
		double facetArea = facetNrl.length(); 
		 
		double minx = 1e6, miny = 1e6, minz = 1e6;
		double maxx = -1e6, maxy = -1e6, maxz = -1e6;

		for(int vi = 0;vi<4;vi++)
		{
			if (coner6x4[j][vi]->x_()>maxx)
			{maxx = coner6x4[j][vi]->x_();}
			if (coner6x4[j][vi]->x_()<minx)
			{minx = coner6x4[j][vi]->x_();}
			if (coner6x4[j][vi]->y_()>maxy)
			{maxy = coner6x4[j][vi]->y_();}
			if (coner6x4[j][vi]->y_()<miny)
			{miny = coner6x4[j][vi]->y_();}
			if (coner6x4[j][vi]->z_()>maxz)
			{maxz = coner6x4[j][vi]->z_();}
			if (coner6x4[j][vi]->z_()<minz)
			{minz = coner6x4[j][vi]->z_();}
		}

		// if a point is outside a bbox, don't count it.  else check if this point is on the facet.
		int sz = pcGroup_ij->size();
		for (int gi=0; gi<sz; gi++)
		{  
			CVector3D & pnt= pcGroup_ij->at(gi)->point_;
			if (pnt.x_()>maxx || pnt.y_()>maxy || pnt.z_()>maxz)
				continue;
			if (pnt.x_()<minx || pnt.y_()<miny || pnt.z_()<minz)
				continue;
			CVector3D vp0 =v0 - pnt;
			CVector3D vp1 =v1 - pnt;
			CVector3D vp2 =v2 - pnt;
			CVector3D vp3 =v3 - pnt;

			CVector3D vp10 = vp1^vp0;
			CVector3D vp21 = vp2^vp1;
			CVector3D vp32 = vp3^vp2;
			CVector3D vp03 = vp0^vp3;

			if (vp10*vp21<0||vp21*vp32<0||vp32*vp03<0||vp10*vp03<0) 
				continue;
			else 
				tempGroup.push_back(pcGroup_ij->m_pnts[gi]);  
		} 
		this->m_numPntOnCube += tempGroup.size();
		this->m_v6facetVtxes.push_back(tempGroup);
	}

}

void CCube::dist_6FacetCorner()
{ 
	abox8Pnts & box8pnts = m_v8Pnts; 
	std::vector<CVector3D*> pp0,pp1,pp2,pp3,pp4,pp5;
	CVector3D * pnt0 = box8pnts[0]; CVector3D * pnt1 = box8pnts[1];
	CVector3D * pnt2 = box8pnts[2]; CVector3D * pnt3 = box8pnts[3];
	CVector3D * pnt4 = box8pnts[4]; CVector3D * pnt5 = box8pnts[5];
	CVector3D * pnt6 = box8pnts[6]; CVector3D * pnt7 = box8pnts[7];
	// clock wise 
	pp0.push_back(pnt0); pp0.push_back(pnt1); pp0.push_back(pnt3); pp0.push_back(pnt2);
	pp1.push_back(pnt4); pp1.push_back(pnt6); pp1.push_back(pnt7); pp1.push_back(pnt5);
	pp2.push_back(pnt0); pp2.push_back(pnt4); pp2.push_back(pnt5); pp2.push_back(pnt1);
	pp3.push_back(pnt2); pp3.push_back(pnt3); pp3.push_back(pnt7); pp3.push_back(pnt6);
	pp4.push_back(pnt0); pp4.push_back(pnt2); pp4.push_back(pnt6); pp4.push_back(pnt4);
	pp5.push_back(pnt1); pp5.push_back(pnt5); pp5.push_back(pnt7); pp5.push_back(pnt3); 

	this->m_6fc4corner.push_back(pp0);
	this->m_6fc4corner.push_back(pp1);
	this->m_6fc4corner.push_back(pp2);
	this->m_6fc4corner.push_back(pp3);
	this->m_6fc4corner.push_back(pp4);
	this->m_6fc4corner.push_back(pp5);
}

bool CCube::pntInFacet( std::vector<CVector3D*> & facet4corner, CVector3D & pnt )
{ 
	CVector3D * v0 = facet4corner[0]; 
	CVector3D * v1 = facet4corner[1]; 
	CVector3D * v2 = facet4corner[2]; 
	CVector3D * v3 = facet4corner[3]; 

	CVector3D vp0 =*v0 -pnt;
	CVector3D vp1 =*v1 -pnt;
	CVector3D vp2 =*v2 -pnt;
	CVector3D vp3 =*v3 -pnt;

	if ((vp1^vp0)*(vp2^vp1)<0||
		(vp2^vp1)*(vp3^vp2)<0||
		(vp3^vp2)*(vp0^vp3)<0||
		(vp1^vp0)*(vp0^vp3)<0) 
	{
		return false;
	}
	else
	{
		return true;
	}
}

void CCube::cal_6coverRate()
{ 
	double p_gridSize = lml_param::ransac_bitmap_resolution/2; //
	int sz = 0; // the whole number of valid grid on a box.
	for (int fi =0; fi<6;fi++)
	{  
		CVertexGroup * tempGroup = this->m_v6Plns[fi];
		std::vector<CVector3D*> & aFacetCorners = this->m_6fc4corner[fi];
		double minx = 1e6,miny = 1e6, minz = 1e6, maxx = -1e6,maxy = -1e6, maxz = -1e6;
		for (int vi=0;vi<4;vi++)
		{
			CVector3D * v = aFacetCorners[vi];
			if (v->x_()<minx)
			{
				minx = v->x_();
			}
			else if (v->x_()>maxx)
			{
				maxx = v->x_();
			}
			if (v->y_()<miny)
			{
				miny = v->y_();
			}
			else if (v->y_()>maxy)
			{
				maxy = v->y_();
			}
			if (v->z_()<minz)
			{
				minz = v->z_();
			}
			else if (v->z_()>maxz)
			{
				maxz = v->z_();
			} 
		} 
		
		double _d = tempGroup->m_cgPlane.d();
		CVector3D _nrl (tempGroup->m_cgPlane.a(),tempGroup->m_cgPlane.b(),tempGroup->m_cgPlane.c());
		_nrl.normalize();
			
		// if the facet is a YZ plane.
		if (abs(tempGroup->m_cgPlane.a()) > 0.8)
		{ 
			if (fi == 0)
				_nrl.pVec[0]= -abs(_nrl.pVec[0]);
			else if(fi == 1)
				_nrl.pVec[0]=  abs(_nrl.pVec[0]);

			double x = 0.5 * (minx + maxx); 
			double y,z;
			for ( y = miny; y<=maxy; y+=p_gridSize)
			{
				for ( z = minz; z<=maxz; z+= p_gridSize)
				{
					CVector3D pntI (x,y,z);

					std::vector<CScanVertex *> neighbor;

					int num = this->m_pc->get_K_nearest_points(pntI,neighbor,10,p_gridSize);
					double bigW = 0;
					double effect = 0;
					for (int i=0; i<neighbor.size();i++)
					{
						CVector3D & _p = neighbor[i]->point_;
						CVector3D & _N = neighbor[i]->normal_; 
							
						double smallW = 1/(1+abs(_p.x_() - _d)); 
						effect += smallW * (_N * _nrl);  
						bigW +=smallW; 
					}
					effect = effect/bigW;
					if (bigW != 0)
					{
						 m_6CoverRate[fi]+= effect;
						 sz++;
					}
					
				}
			}
		}

		// if the facet is a XZ plane.
		if (abs(tempGroup->m_cgPlane.b()) > 0.8)
		{
			if (fi == 2)
				_nrl.pVec[1]= -abs(_nrl.pVec[1]);
			else if(fi == 3)
				_nrl.pVec[1]=  abs(_nrl.pVec[1]);

			double y = 0.5 * (miny + maxy); 
			double x,z;
			for ( x = minx; x<=maxx; x+= p_gridSize)
			{
				for ( z = minz; z<=maxz; z+= p_gridSize)
				{
					CVector3D pntI (x,y,z);

					std::vector<CScanVertex *> neighbor;

					int num = this->m_pc->get_K_nearest_points(pntI,neighbor,10,p_gridSize);
					double bigW = 0;
					double effect = 0;
					for (int i=0; i<neighbor.size();i++)
					{
						CVector3D & _p = neighbor[i]->point_;
						CVector3D & _N = neighbor[i]->normal_;
						double smallW = 1/(1+abs(_p.y_() - _d));
						effect += smallW * (_N * _nrl);  
						bigW +=smallW; 
					}
					effect = effect/bigW;
					if (bigW != 0)
					{
						m_6CoverRate[fi]+= effect;
						sz++;
					} 
				}
			}
		}
		 
		// if the facet is a XY plane.
		if (abs(tempGroup->m_cgPlane.c()) > 0.8)
		{
			if (fi == 4)
				_nrl.pVec[2]= -abs(_nrl.pVec[2]);
			else if(fi == 5)
				_nrl.pVec[2]=  abs(_nrl.pVec[2]);

			double z = 0.5 * (minz + maxz); 
			double x,y;
			for ( y = miny; y<=maxy; y+=p_gridSize)
			{
				for ( x = minx; x<=maxx; x+= p_gridSize)
				{
					CVector3D pntI (x,y,z);

					std::vector<CScanVertex *> neighbor;

					int num = this->m_pc->get_K_nearest_points(pntI,neighbor,10,p_gridSize);
					double bigW = 0;
					double effect = 0;
					for (int i=0; i<neighbor.size();i++)
					{
						CVector3D & _p = neighbor[i]->point_;
						CVector3D & _N = neighbor[i]->normal_;
						double smallW = 1/(1+abs(_p.z_() - _d));
						effect += smallW * (_N * _nrl);  
						bigW +=smallW; 
					}
					effect = effect/bigW;
					if (bigW != 0)
					{
						m_6CoverRate[fi]+= effect;
						sz++;
					}
				}
			}
		} // if the facet is a XY plane.

	}// end of "for (int fi =0; fi<6;fi++)"
	if (sz ==0)
	{
		return;
	}
	else
	{ 
		m_validGridNum = sz;
		for (int fi=0;fi<6;fi++)
		{
			m_score += m_6CoverRate[fi];
		}
		//m_score = m_score/ m_validGridNum;
		return ;
	}
}

void CCube::draw_facetPnts()
{
	if (!this->m_CPolyHedron->toShow || this->m_v6facetVtxes.size()==0)
		return;
	else
		for (int i=0;i<this->m_v6facetVtxes.size();i++)
		{
			std::vector<CScanVertex*> & grp = this->m_v6facetVtxes[i];
			
			glBegin(GL_POINTS);
			for (std::vector<CScanVertex*>::iterator it = grp.begin(); it != grp.end();it++)
			{
				glColor3d((*it)->color_.r(),  (*it)->color_.g(), (*it)->color_.b());
				glNormal3d((*it)->normal_.x_(),(*it)->normal_.y_(),(*it)->normal_.z_());
				glVertex3d((*it)->point_.x_(), (*it)->point_.y_(), (*it)->point_.z_()); 
			}
			glEnd();
		}
}

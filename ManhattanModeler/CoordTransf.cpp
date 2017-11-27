
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


#include "CoordTransf.h"

#include "CPointCloud.h"
#include "CVector3D.h"
#include "CVertexGroup.h"

CCoordTransf::CCoordTransf(void)
{
}

CCoordTransf::~CCoordTransf(void)
{
}

void CCoordTransf::rotateSinglePnt( CVector3D & m_vecPoint, double cos_theta, double sin_theta, CoordTranferMode mode)
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
void CCoordTransf::rotateSinglePnt( CVector3D & m_vecPoint, double theta,CoordTranferMode mode)
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
void CCoordTransf::transBackSingPnt( CVector3D & m_vecPoint, double cos_theta, double sin_theta, CoordTranferMode mode)
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
void CCoordTransf::transBackSingPnt( CVector3D & m_vecPoint, double  theta,  CoordTranferMode mode)
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
void CCoordTransf::rotatePntCloud( CPointCloud & pc, double radian_theta, CoordTranferMode mode )
{
	double cos_theta, sin_theta;

	cos_theta=cos(radian_theta);//-3.14159265359/2);
	sin_theta=sin(radian_theta);//-3.14159265359/2);

	pc.m_cBoundingBox->Reset();
	int pntNb=pc.m_vtxPC.size();
	for (int i=0;i<pntNb;i++)
	{
		CVector3D & pnt=pc.m_vtxPC[i]->point_;
		rotateSinglePnt(pnt, cos_theta, sin_theta,mode); 
		pc.m_cBoundingBox->Push(pnt);

		CVector3D & nrl=pc.m_vtxPC[i]->normal_;
		rotateSinglePnt(nrl,cos_theta, sin_theta,mode);
	}
	pc.build_boundingBox();
}

void CCoordTransf::transBackPntCloud( CPointCloud & pc, double radian_theta, CoordTranferMode mode  )
{
	double cos_theta, sin_theta;

	cos_theta=cos(radian_theta);//-3.14159265359/2);
	sin_theta=sin(radian_theta);//-3.14159265359/2);

	pc.m_cBoundingBox->Reset();
	int pntNb=pc.m_vtxPC.size();
	for (int i=0;i<pntNb;i++)
	{
		CVector3D & pp=pc.m_vtxPC[i]->point_;
		transBackSingPnt(pp, cos_theta,sin_theta,mode); 
		pc.m_cBoundingBox->Push(pp);
		CVector3D & pn=pc.m_vtxPC[i]->normal_;
		transBackSingPnt(pn,cos_theta,sin_theta,mode);
	}
}

void CCoordTransf::transGroupPC( CVertexGroup & pcGroup, double radian_theta, CoordTranferMode mode )
{
	double cos_theta, sin_theta;

	cos_theta=cos(radian_theta);//-3.14159265359/2);
	sin_theta=sin(radian_theta);//-3.14159265359/2); 

	int pntNb=pcGroup.size();
	for (int i=0;i<pntNb;i++)
	{
		//CScanVertex *
		CScanVertex * vtx = pcGroup[i];
		CVector3D & pnt=vtx->point_;
		rotateSinglePnt(pnt, cos_theta, sin_theta,mode);  

		CVector3D & nrl=vtx->normal_;
		rotateSinglePnt(nrl,cos_theta, sin_theta,mode);
	} 
	pcGroup.updateMidPnt(); 
}

void CCoordTransf::transBackGroupPC( CVertexGroup & pcGroup, double radian_theta, CoordTranferMode mode  )
{
	double cos_theta, sin_theta;

	cos_theta=cos(radian_theta);//-3.14159265359/2);
	sin_theta=sin(radian_theta);//-3.14159265359/2);

	int pntNb=pcGroup.size();
	for (int i=0;i<pntNb;i++)
	{
		CScanVertex * vtx = pcGroup[i];
		CVector3D & pnt=vtx->point_;
		transBackSingPnt(pnt, cos_theta,sin_theta,mode);  
		CVector3D & pn=vtx->normal_;
		transBackSingPnt(pn,cos_theta,sin_theta,mode);
	}
} 
 

void CCoordTransf::translatePntCloud( CPointCloud *pc, const double tx, const double ty, const double tz, const double pScale /*= 1.0*/ )
{
	if (pScale == 1.0)
	{ 
		pc->m_cBoundingBox->Reset(); 
		CVector3D _pos (tx,ty,tz);
		for (std::vector<CScanVertex *>::iterator it= pc->m_vtxPC.begin(); it != pc->m_vtxPC.end();it++)
		{
			CScanVertex * tempVtx = *it;
			tempVtx->point_ = tempVtx->point_ + _pos;
		} 
		pc->build_boundingBox();
	}
	else
	{ 
		pc->m_cBoundingBox->Reset(); 
		CVector3D _pos (tx,ty,tz);
		for (std::vector<CScanVertex *>::iterator it= pc->m_vtxPC.begin(); it != pc->m_vtxPC.end();it++)
		{
			CScanVertex * tempVtx = *it;
			tempVtx->point_ = (tempVtx->point_ + _pos) * pScale;
		} 
		pc->build_boundingBox();
	}
}
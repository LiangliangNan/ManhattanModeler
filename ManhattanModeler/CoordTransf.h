
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

//
#include "math_global.h"


class CPointCloud;
class CVector3D;
class CVertexGroup;
class CCoordTransf
{
public:
	CCoordTransf(void);
	~CCoordTransf(void);
private:
	void rotateSinglePnt( CVector3D & m_vecPoint, double cos_theta, double sin_theta, CoordTranferMode mode);
	void rotateSinglePnt( CVector3D & m_vecPoint, double theta,CoordTranferMode mode);
	void transBackSingPnt( CVector3D & m_vecPoint, double cos_theta, double sin_theta, CoordTranferMode mode);
	void transBackSingPnt( CVector3D & m_vecPoint, double theta, CoordTranferMode mode);
public: 
	void translatePntCloud(CPointCloud *pc, const double tx, const double ty, const double tz, const double pScale = 1.0);// x0 + tx; y0 + ty; z0 +tz.
	void rotatePntCloud( CPointCloud & pc, double radian_theta, CoordTranferMode mode );
	void transBackPntCloud( CPointCloud & pc, double radian_theta, CoordTranferMode mode ); 
	void transGroupPC( CVertexGroup & pcGroup, double radian_theta, CoordTranferMode mode );
	void transBackGroupPC( CVertexGroup & pcGroup, double radian_theta, CoordTranferMode mode );
};

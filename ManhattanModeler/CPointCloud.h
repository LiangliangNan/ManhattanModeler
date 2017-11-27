
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

#include <vector>

#include "BoundingBox.h"
#include <time.h>

#include "math_types.h"
#include "math_global.h"

#include "CVertexGroup.h"
#include "kd_tree/kdTree.h"

#include "CMeshObj.h"
  
 
 

class CPointCloud
{
public:
	CPointCloud(void); 
	~CPointCloud(void); 

public:
	std::vector<CScanVertex*> m_vtxPC; // the critical data storing all points. 

	bool hasMW;
	int m_nb_vtx;
	bool toShow; 
	bool m_bSelect;
	bool showHeightColor;
	CBoundingBox * m_cBoundingBox;  
public:  
	std::vector<CVertexGroup*> m_vtxGroup_depthmap; // vertex groups identified from depth map by detecting rectangle lines.
	   
	CMeshObj * m_MeshObj;  

	std::vector<cgTriangle> m_vCGTriangles; // vector of mesh triangles, used for AABB tree query, for reconstruction error analysis. 
private:
	cgAABBTree * m_aabbTree; 

public:
	void clearAll();
	void deletKDtree();  
	CPointCloud & CPointCloud::operator=( const CPointCloud & pc_ );
	void build_boundingBox(); 
	  
	void drawPC(float pntSize/*, int interval = 1*/);
	void drawLabels();
	bool save_ply(const char * file_name);
	  
	std::vector<int>  m_vecLabels; // this indicates the segmentation results for each point. its size = vtxPnt.size
      
public: 
	static void load_ply( const char * filename, CPointCloud * ptrPC ); 
	static CPointCloud * load_ply( const char * filename );  

	static void transPntCloud(CPointCloud & pc, double theta, CoordTranferMode mode);
	static void transBackPntCloud(CPointCloud & pc, double theta, CoordTranferMode mode);
	static void trSinglePnt( CVector3D & m_vecPoint, double theta,CoordTranferMode mode);
	static void trBackSingPnt( CVector3D & m_vecPoint, double theta, CoordTranferMode mode);
	static void calculateXYZ(cgPlane3f & abcd1, cgPlane3f &abcd2, cgPlane3f &abcd3, CVector3D & interscPnt);
	
public:  
	void smooth_PC_By_Range( const double pRange, unsigned int k = 8 );
	unsigned int get_K_nearest_points( CScanVertex * vQuery, std::vector<CScanVertex*>& neighbors, int K =30 ); 
	unsigned int get_K_nearest_points( CScanVertex * vQuery, std::vector<CScanVertex*> & result, int K /*= 30*/, double fRange /*= 0.5*/);
	unsigned int get_K_nearest_points( CVector3D & vQuery, std::vector<CScanVertex*> & neighbors, int K /*= 10*/, double fRange = -1); 
	unsigned int get_K_nearest_pntIdx( CVector3D & vQuery, std::vector<int>& vPositionIndex, int K /*=30*/ );
	void get_1_nearest_CVector3D( CVector3D & vQuery, CVector3D & result);
	 
private:	
	static void trSinglePnt( CVector3D & m_vecPoint, double cos_theta, double sin_theta, CoordTranferMode mode);
	static void trBackSingPnt( CVector3D & m_vecPoint,double cos_theta, double sin_theta , CoordTranferMode mode);

private: 
	void build_kd_tree( int nMaxBucketSize= 16 );   
   
	CMeshObj * gen_meshFromXYZ_idx(std::vector<double> &pntxyz, std::vector<int> &faceIdx012);
	kdtree::KdTree*			m_ptrKDTree;

	std::vector<Colorf> m_colorTab;
};










#pragma once
#include <vector>

#include "CVertexGroup.h"

struct _pCoord{
	int xId;
	int yId;
	int zId;
	_pCoord(int px,int py, int pz)
	{xId = px; yId = py; zId = pz;}
};

class CPointCloud;
class CFittingBox
{
public:
	CFittingBox(CPointCloud* pc);
	~CFittingBox(void);
  
	int num_xPln, num_yPln, num_zPln;

public:
	std::vector<std::vector<CVector3D*>> vecBox8Pnts;    // the result of fitting box procedure.
	std::vector<std::vector<CVertexGroup*>> vecBox6Plns; // the result of fitting box procedure.
	std::vector<_pCoord> vecBoxCoord;					 // the coordinate of a box, i.e. xpln, ypln, zpln started from 0.

public:
	bool fitBox_0(std::vector<CVertexGroup*> & vPtrPG);  // without axis alignment.
	bool fitBox_1(std::vector<CVertexGroup*> & vPtrPG);  // sort along axis, and fit all boxes.
	bool fitBox_2(std::vector<CVertexGroup*> & vPtrPG);  // sort along axis, and only fit basic boxes.

	void mergeSimilarPlns(std::vector<CVertexGroup*> & vPtrPG);
	static void sortPlns(std::vector<CVertexGroup*> & vPG, std::vector<CVertexGroup*> & XvPln, std::vector<CVertexGroup*> & YvPln, std::vector<CVertexGroup*> & ZvPln);
	
private: 
	bool checkPlnNoCross(std::vector<CVector3D*> & boxp);
	void cal_vecBox8Pnts(std::vector<std::vector<CVertexGroup*>> & tempVecBox6Plns,		// input
							std::vector<std::vector<CVector3D*>> & p_vecBox8Pnts,		// result
							std::vector<std::vector<CVertexGroup*>> & p_vecBox6Plns);	// result 
private:
	CPointCloud *m_pc;
};

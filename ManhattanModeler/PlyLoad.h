#pragma once
#include <iostream>
#include <vector>
#include "rply.h"

class CPointCloud;
class CVector3D;
class CScanVertex;

class CPlyLoad
{
public:
	CPlyLoad(void);
	~CPlyLoad(void);
	void loadPly(const char* filename,CPointCloud * pcPtr);
private:
	static int color_cb(p_ply_argument argument);
	static int vertex_cb(p_ply_argument argument);
	static int normal_cb(p_ply_argument argument);
	static int face_cb(p_ply_argument argument);
	int get_color_data(p_ply_argument argument);
	int get_vertex_data(p_ply_argument argument);
	int get_normal_data(p_ply_argument argument);
	int get_face_data(p_ply_argument argument);
	void check_for_Elements(p_ply ply);
	static CPlyLoad* plyload(p_ply_argument argument);
 
	std::vector<CScanVertex*> vVextex_;
	double	xyz_[3] ;
	double	normal_[3];
	double	rgb_[3] ;
	static int		*idx3;
	std::vector<std::vector<int>>	faceIdx;
	std::vector<int> idxx;
	bool hasFaces;

};
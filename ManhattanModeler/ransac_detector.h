
#pragma once
 
#include "CPointCloud.h"  
class RansacDetector
{
	RansacDetector(void);
	~RansacDetector(void);
public:   
	static unsigned int ransac_apply( CPointCloud * pntCloud, std::vector<CVertexGroup*> & pntGroup /*, CPointCloud * pntCloudRemaind */);
	   
};




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


#include "Parameters.h"
#include <stdlib.h>
namespace lml_param
{
	
	int    lml_param::ransac_minimum_support = 100;
	double lml_param::ransac_distance_threshold = 0.1;
	double lml_param::ransac_bitmap_resolution = 0.4;
	double lml_param::ransac_normalThresh = 0.2;

	int    lml_param::ransac_minimum_support2 = 300;
	double lml_param::douransac_distance_threshold2 = 0.1;
	double lml_param::ransac_bitmap_resolution2 = 0.4;
	double lml_param::ransac_probability = 0.001;

	double lml_param::threshold_Paral	=0.965;	//   sin(75?
	double lml_param::threshold_vertl	=0.258;  //  >75 degree. cos(75?
	double lml_param::threshold_merge =	0.1;

	double lml_param::optim_lambda = 0.1; 

	int lml_param::sample_pnt_per_m2 = 0;
	double lml_param::sample_add_noise = 0.1;
	int lml_param::showPointSize = 2;
	
	double lml_param::gridWidth= 0.35;
	double lml_param::GroundZ=-35.5;
	double lml_param::DeltaZ = 1.2;

	bool lml_param::hideCamera = false;
	bool lml_param::hideGrid = false; 
	bool lml_param::hideRegionRANSAC = false; 
	bool lml_param::hideBounding = true; 
	bool lml_param::hidePolyhedron = false;
	bool lml_param::hidePoisson = false;
	bool lml_param::showNormal = false;
	bool lml_param::showPHWirframe = false;
	bool lml_param::rdmColor = true;
	bool lml_param::smoothShading = true;

	bool lml_param::shift1= false;
	bool lml_param::shift2= false;

	double lml_param::neighborRadius = 0.1;

	int lml_param::SLIC_num = 200;
	int lml_param::SLIC_Compactness = 10;

	double lml_param::clrBarMax = 1.0;
	double lml_param::clrBarMin = 0;

	double lml_param::regionMRF_dataT1=4;
	double lml_param::regionMRF_dataT2=2;
	double lml_param::regionMRF_dataT3=7; 
	float lml_param::regionMRF_smoothT1 = 1.5;

	double lml_param::roofMRF_smoothT1=8; 

	double  lml_param::Poisson_depth = 7;
	double  lml_param::Poisson_solverDivide = 8;
	double  lml_param::Poisson_IsoDivide = 8;
	double  lml_param::Poisson_MinDepth = 5;
	double  lml_param::Poisson_scale = 1.1;
	double  lml_param::Poisson_accuracy = 0.001;
	double  lml_param::Poisson_pointweight = 4;

	double lml_param::contourTracingParam = 1;
	
	// control openGL view
	bool	lml_param::lighting = true; 
	bool	lml_param::isTransparent = true;
	bool    lml_param::is_drawCornerAxis = true;

	double	lml_param::cameraScale = 0.1;
	double  lml_param::NrlLength = 0.2;
	// control openGL view
}
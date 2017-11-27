
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




#ifndef PARAM_H_
#define PARAM_H_

namespace lml_param
{
	// RANSAC
	extern int ransac_minimum_support;
	extern double ransac_distance_threshold;
	extern double ransac_bitmap_resolution;
	extern double ransac_normalThresh;
	extern int ransac_minimum_support2;
	extern double douransac_distance_threshold2;
	extern double ransac_bitmap_resolution2;
	extern double ransac_probability;

	//fit box 
	extern double threshold_Paral;	 
	extern double threshold_vertl; 
	extern double threshold_merge;

	extern double optim_lambda; // used for optimization

	// point cloud
	extern int sample_pnt_per_m2;
	extern double sample_add_noise;// add noise to the sampling point. this parameter indicates the "mean error"
	extern int showPointSize;

	extern double gridWidth;
	extern double GroundZ;
	extern double DeltaZ;

	extern bool hideCamera;
	extern bool hideGrid; 
	extern bool hideRegionRANSAC; 
	extern bool hideBounding;
	extern bool hidePolyhedron;
	extern bool showNormal;
	extern bool showPHWirframe;
	extern bool rdmColor; 
	extern bool hidePoisson;
	 
	extern	bool	shift1; // used flexible for some shift states.
	extern	bool	shift2; // used flexible for some shift states.

	// 
	extern double neighborRadius;

	// Poisson reconstruction & Octree 
	extern double  Poisson_depth;
	extern double  Poisson_solverDivide;
	extern double  Poisson_IsoDivide;
	extern double  Poisson_MinDepth;
	extern double  Poisson_scale;
	extern double  Poisson_accuracy;
	extern double  Poisson_pointweight;

	//

	// parameters used in super-pixel segmentation.
	extern int		SLIC_num;//Desired number of super-pixels.
	extern int		SLIC_Compactness;//Compactness factor. use a value ranging from 10 to 40 depending on your needs. Default is 10

	extern double clrBarMax;
	extern double clrBarMin;

	extern	double regionMRF_dataT1;
	extern	double regionMRF_dataT2;
	extern	double regionMRF_dataT3 ;
	extern	float regionMRF_smoothT1;

	extern	double roofMRF_smoothT1;
	extern	double roofMRF_smoothT2;
  
	extern	double contourTracingParam;
	  
	// control openGL view
	extern  bool   lighting;
	extern	bool   smoothShading; 
	extern  bool   isTransparent;
	extern  bool   is_drawCornerAxis;

	extern  double cameraScale;
	extern	double NrlLength;
	// control openGL view
}	
#endif
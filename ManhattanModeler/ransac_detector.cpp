
#include "ransac_detector.h"

#include <stdio.h> 

////////////////////////////////////////////////////////////////////////
#include "../3rd_ransac/RansacShapeDetector.h"
#include "../3rd_ransac/PlanePrimitiveShapeConstructor.h"
#include "../3rd_ransac/CylinderPrimitiveShapeConstructor.h"
#include "../3rd_ransac/SpherePrimitiveShapeConstructor.h"
#include "../3rd_ransac/ConePrimitiveShapeConstructor.h"
#include "../3rd_ransac/TorusPrimitiveShapeConstructor.h"
#include "../3rd_ransac/PlanePrimitiveShape.h"
#include "../3rd_ransac/SpherePrimitiveShape.h"
#include "../3rd_ransac/CylinderPrimitiveShape.h"
#include "../3rd_ransac/ConePrimitiveShape.h"
#include "../3rd_ransac/TorusPrimitiveShape.h"

#include "Parameters.h"
 

unsigned int sum_of_first_i_elements(unsigned int i, const std::vector<unsigned int>& sizes) 
{
	if (i == 0)
		return sizes[0];
	else 
		return sizes[i] + sum_of_first_i_elements(i-1, sizes);
}


void range(const MiscLib::Vector< std::pair< MiscLib::RefCountPtr< PrimitiveShape >, size_t > >& shapes,
		   unsigned int total_num, 
		   unsigned int index,
		   unsigned int& left,
		   unsigned int& right)
{
	std::vector<unsigned int>   sizes;

	MiscLib::Vector< std::pair< MiscLib::RefCountPtr< PrimitiveShape >, size_t > >::const_iterator itr = shapes.begin();
	for (; itr != shapes.end(); ++itr) {
		unsigned int size = itr->second;
		sizes.push_back(size);
	}

	//////////////////////////////////////////////////////////////////////////
	// the points belonging to the first shape (shapes[0]) have been sorted to the end of pc,
	// i.e. into the range [ pc.size() - shapes[0].second, pc.size() )
	// the points of shape i are found in the range
	// [ pc.size() - \sum_{j=0..i} shapes[j].second, pc.size() - \sum_{j=0..i-1} shapes[j].second )
	if (index == 0) {
		left = total_num - sum_of_first_i_elements(0, sizes);
		right = total_num;
	} else if (index > 0) {
		left = total_num - sum_of_first_i_elements(index, sizes);
		right = total_num - sum_of_first_i_elements(index - 1, sizes);
	}
}



void clustering(CPointCloud * pntCloud,  const PointCloud& pc,
				const MiscLib::Vector< std::pair< MiscLib::RefCountPtr< PrimitiveShape >, size_t > >& shapes,
				std::vector<CVertexGroup*> & pntGroupVec 
				) 
{
	std::vector< std::pair<std::vector<Point>, const PrimitiveShape*> >   groups;
	groups.clear();
	MiscLib::Vector< std::pair< MiscLib::RefCountPtr< PrimitiveShape >, size_t > >::const_iterator itr = shapes.begin();
	for (unsigned int i=0; itr != shapes.end(); ++itr, ++i) 
	{

		unsigned int left = 0;
		unsigned int right = 0;
		range(shapes, pc.size(), i, left, right);

		std::pair<std::vector<Point>, const PrimitiveShape*> g;
		for (unsigned int j=left; j<right; ++j) 
		{
			(g.first).push_back(pc[j]);
		}

		g.second = itr->first; 
		groups.push_back(g);
	} 

	for (unsigned int i=0; i<groups.size(); ++i) 
	{
		const std::vector<Point>& pntGroup = groups[i].first;
		const PrimitiveShape* primitive = groups[i].second;

		if (pntGroup.size() <lml_param::ransac_minimum_support)
			continue;

		CVertexGroup* one_Group =new CVertexGroup();
		// grab the plane parameter

		Plane p = dynamic_cast<const PlanePrimitiveShape*>(primitive)->Internal();
		Vec3f nor = p.getNormal();
		Vec3f pos = p.getPosition();
		CVector3D queryPos(pos[0], pos[1], pos[2]);

		// judge the direction of the group plane
		std::vector<CScanVertex* > neighbour;
		pntCloud->get_K_nearest_points(queryPos,neighbour,5);
		CVector3D nrl(0,0,0);
		for (int ni= 0;ni<5;ni++) 
			nrl+=neighbour[ni]->normal_;  
		CVector3D nrlPln (nor[0], nor[1], nor[2]);
		cgPlane3f tmpPlane;
		if (nrl*nrlPln<0)
		{
			tmpPlane = cgPlane3f(cgPoint3f(pos[0], pos[1], pos[2]), cgVector3f(-nor[0], -nor[1], -nor[2]));
			one_Group->m_cgPlane=tmpPlane;
		}
		else
		{
			tmpPlane = cgPlane3f(cgPoint3f(pos[0], pos[1], pos[2]), cgVector3f(nor[0], nor[1], nor[2]));
			one_Group->m_cgPlane=tmpPlane;
		}
		// 

		Colorf clr;

		bool useRdmClr = true;
		if (useRdmClr)
		{
			float cred = (rand()%256)/255.0f;
			float cgreen = (rand()%256)/255.0f;
			float cblue = (rand()%256)/255.0f;  
			clr = Colorf(cred,cgreen,cblue);
		}
		else
			clr = Global::color_from_table(10 + i*235/(groups.size()+1));


		CVector3D midpnt(0,0,0); 

		for (unsigned int j=0; j<pntGroup.size(); ++j) 
		{
			const Vec3f& p = pntGroup[j].pos; 

			//// ********** get the vertex pointer to the group ********** 
			//std::vector<CScanVertex* > posi;
			//CVector3D query(p[0], p[1], p[2]);
			//pntCloud->get_K_nearest_points(query,posi,1);
			//if (posi.size()>0) 
			//	one_Group->m_pnts.push_back(posi.back()); 

			// ********** save the projected points  ***************
			cgPoint3f cgP_src(p[0], p[1], p[2]); 
			cgPoint3f cgP_dst=tmpPlane.projection(cgP_src);
			CVector3D tmpPnt(cgP_dst.x(),cgP_dst.y(),cgP_dst.z()); 
			CVector3D tmpNrl(one_Group->m_cgPlane.a(),one_Group->m_cgPlane.b(),one_Group->m_cgPlane.c());
			one_Group->push_back(new CScanVertex(tmpPnt,tmpNrl,clr)); 

			midpnt+=tmpPnt; 

			// ********** save the original points  ***************
			CVector3D queryP = CVector3D(p[0], p[1], p[2]);
			std::vector<CScanVertex* > neighbour;
			if (pntCloud->get_K_nearest_points(queryP,neighbour,1))
			{
				one_Group->m_pnts.push_back(neighbour[0]);
			} 
			 
		}
		//std::sort(one_Group.begin(), one_Group.end());
		one_Group->m_toshow = true;
		midpnt=midpnt/one_Group->size();
		one_Group->m_midPnt=midpnt;
		one_Group->m_color = clr;
		pntGroupVec.push_back(one_Group);

	}
}

unsigned int do_detect( CPointCloud * pntCloud, PointCloud& pc, 
					   unsigned int size, 
					   MiscLib::Vector< std::pair< MiscLib::RefCountPtr< PrimitiveShape >, size_t > >& shapes,
					   std::vector<CVertexGroup*> & pntGroupVec 
					   )
{
	RansacShapeDetector::Options ransacOptions;
	// set distance threshold to .01f of bounding box width
	ransacOptions.m_epsilon = lml_param::ransac_distance_threshold; // .01f * pc.getScale(); 	// the point cloud has been normalized
	// NOTE: Internally the distance threshold is taken as 3 * ransacOptions.m_epsilon!!!

	// set bitmap resolution to .02f of bounding box width
	ransacOptions.m_bitmapEpsilon = lml_param::ransac_bitmap_resolution; // .02f * pc.getScale(); 	// the point cloud has been normalized
	// NOTE: This threshold is NOT multiplied internally!

	ransacOptions.m_normalThresh = lml_param::ransac_normalThresh; // this is the cos of the maximal normal deviation
	ransacOptions.m_minSupport = lml_param::ransac_minimum_support; // this is the minimal number of points required for a primitive
	ransacOptions.m_probability = lml_param::ransac_probability; // this is the "probability" with which a primitive is overlooked

	RansacShapeDetector detector(ransacOptions); // the detector object

	// set which primitives are to be detected by adding the respective constructors
	detector.Add(new PlanePrimitiveShapeConstructor());	

	shapes.clear(); // stores the detected shapes
 
	size_t remaining = detector.Detect(pc, 0, pc.size(), &shapes); // run detection
	
	// returns number of unassigned points
	// the array shapes is filled with pointers to the detected shapes
	// the second element per shapes gives the number of points assigned to that primitive (the support)
	// the points belonging to the first shape (shapes[0]) have been sorted to the end of pc,
	// i.e. into the range [ pc.size() - shapes[0].second, pc.size() )
	// the points of shape i are found in the range
	// [ pc.size() - \sum_{j=0..i} shapes[j].second, pc.size() - \sum_{j=0..i-1} shapes[j].second )

	//////////////////////////////////////////////////////////////////////////
	clustering(pntCloud, pc, shapes, pntGroupVec );

	return remaining;
}

unsigned int RansacDetector::ransac_apply( CPointCloud * pntCloud, std::vector<CVertexGroup*> & pntGroupVec/*, CPointCloud * pntCloudRemaind*/  )
{
	if (pntCloud->m_vtxPC.size() < 3) 
	{
		std::cout<<"point cloud does not meet requirement!"<<std::endl;
		return pntCloud->m_vtxPC.size();
	}  
	
	// clean the vertex group pointer vector.
	while(!pntGroupVec.empty()) 
	{
		delete pntGroupVec.back();
		pntGroupVec.pop_back();
	}

	unsigned int size = pntCloud->m_vtxPC.size();

	Point* points = new Point[size];
	for (unsigned int i=0; i<size; ++i) 
	{ 
		points[i] = Point(
			Vec3f(pntCloud->m_vtxPC[i]->point_.x_(), pntCloud->m_vtxPC[i]->point_.y_(),pntCloud->m_vtxPC[i]->point_.z_()), 
			//Vec3f(pntCloud.m_vecPoint[i].pVec[0], pntCloud.m_vecPoint[i].pVec[1],pntCloud.m_vecPoint[i].pVec[2]));
			Vec3f(pntCloud->m_vtxPC[i]->normal_.x_(),pntCloud->m_vtxPC[i]->normal_.y_(), pntCloud->m_vtxPC[i]->normal_.z_()));
		//points[i] = Point(
		//	Vec3f(pntCloud->m_vecPoint[i].pVec[0], pntCloud->m_vecPoint[i].pVec[1],pntCloud->m_vecPoint[i].pVec[2]));
	}

	MiscLib::Vector< std::pair< MiscLib::RefCountPtr< PrimitiveShape >, size_t > > shapes;

	PointCloud pc(points, size);
	// the point cloud has been normalized
	// pc.setBBox(Vec3f(bbox.xmin(), bbox.ymin(), bbox.zmin()), Vec3f(bbox.xmax(), bbox.ymax(), bbox.zmax()));
	delete points;

	unsigned int remaining = do_detect(pntCloud, pc, size, shapes, pntGroupVec );
	//std::vector<CVector3D> & vecPos = pntCloudRemaind->m_vecPoint;
	//std::vector<CVector3D> & vecNrl = pntCloudRemaind->m_vecNormal;
	//for (int i=0;i<remaining;i++)
	//{
	//	CVector3D posi(pc[i].pos[0],pc[i].pos[1],pc[i].pos[2]);
	//	CVector3D nrli(pc[i].normal[0],pc[i].normal[1],pc[i].normal[2]);
	//	vecPos.push_back(posi);
	//	vecNrl.push_back(nrli);
	//}
	//std::cout<<""

	return remaining;
}


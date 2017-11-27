
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


#include "fitBoxes.h"
#include "CVertexGroup.h"
#include "BoundingBox.h" 
#include "logger.h"
#include "ProgressBar.h"

#include <time.h>
#include <set> 
#include "Parameters.h" 
  
#include "../3rd_graphcut/GCoptimization.h" // for Graph cut optimization.

#include <CGAL/Bbox_3.h> 
#include <CGAL/boost/graph/graph_traits_Polyhedron_3.h>  
typedef cgAABBTree::Primitive_id Primitive_id;
typedef cgPolyhedron::Halfedge_around_facet_circulator phHalfedge_facet_circulator;

CCubeBoxes::CCubeBoxes()
	:m_pc(nil),
	m_finalPhModel(nil)
{
}

// constructor: from the results of fitting box procedure. 
CCubeBoxes::CCubeBoxes( CPointCloud * pc,CFittingBox * fb )
{
	this->m_pc = pc;
	m_finalPhModel = NULL;

	m_vecBox8pnts.clear(); 
	m_vecBox6Plns.clear();
	m_vecBoxCoord.clear();

	if (fb->vecBox6Plns.size()<1 || fb->vecBox8Pnts.size()<1 || fb->vecBox6Plns.size()!=fb->vecBox8Pnts.size())
	{
		Logger::output("error: there is not enough planes and points for boxes.\n");
		return;
	}
	int boxnum = fb->vecBox8Pnts.size();
	for (int i=0;i<boxnum; i++ )
	{
		m_vecBox8pnts.push_back(fb->vecBox8Pnts[i]);
		m_vecBox6Plns.push_back(fb->vecBox6Plns[i]);
		m_vecBoxCoord.push_back(fb->vecBoxCoord[i]);
	} 

	m_numX = fb->num_xPln-1;
	m_numY = fb->num_yPln-1;
	m_numZ = fb->num_zPln-1;
	// ***** using 8 points to generate a cube (box).*****
	buildCubeFrom8Pnt();  
	identifyCubeNeighbor();
	cal_cubeScore();
}

CCubeBoxes::~CCubeBoxes()
{  
}

void CCubeBoxes::buildCubeFrom8Pnt()
{   
	double minx=1e6,miny=1e6,minz=1e6,maxx=-1e6,maxy=-1e6,maxz=-1e6;

	int numBox = m_vecBox8pnts.size();  

	int boxnum = m_numX * m_numY * m_numZ;
	if ( numBox != boxnum || this->m_vecBoxCoord.size() != boxnum) 
	{
		Logger::output("the number of boxes has problems.\n");
		return;
	}

	for (int i=0;i<numBox;i++)
	{   
		 std::vector<CVector3D*> & boxp = m_vecBox8pnts[i];
		// 1. prepare for the bounding box for canvas
		for (int bi=0;bi<8;bi++)
		{
			float xi = boxp[bi]->x_();
			float yi = boxp[bi]->y_();
			float zi = boxp[bi]->z_();
			if (xi<minx) { minx = xi; }
			if (xi>maxx) { maxx = xi; }
			if (yi<miny) { miny = yi; }
			if (yi>maxy) { maxy = yi; }
			if (zi<minz) { minz = zi; }
			if (zi>maxz) { maxz = zi; }
		} 

		// 2. create a polyhedron. CGAL formate 
		CVector3D * corner0 = boxp[0];
		CVector3D * corner1 = boxp[1];
		CVector3D * corner2 = boxp[2];
		CVector3D * corner3 = boxp[3];
		CVector3D * corner4 = boxp[4];
		CVector3D * corner5 = boxp[5];
		CVector3D * corner6 = boxp[6];
		CVector3D * corner7 = boxp[7];  

		cgPolyhedron pPolyhedron;
		cgPolyPoint_3 pp0(corner0->x_(),corner0->y_(),corner0->z_());  
		cgPolyPoint_3 pp1(corner1->x_(),corner1->y_(),corner1->z_()); 
		cgPolyPoint_3 pp2(corner2->x_(),corner2->y_(),corner2->z_()); 
		cgPolyPoint_3 pp3(corner3->x_(),corner3->y_(),corner3->z_()); 
		cgPolyPoint_3 pp4(corner4->x_(),corner4->y_(),corner4->z_()); 
		cgPolyPoint_3 pp5(corner5->x_(),corner5->y_(),corner5->z_()); 
		cgPolyPoint_3 pp6(corner6->x_(),corner6->y_(),corner6->z_()); 
		cgPolyPoint_3 pp7(corner7->x_(),corner7->y_(),corner7->z_()); 

		CPolyHedron::make_cube_3(pPolyhedron,pp0,pp1,pp2,pp3,pp4,pp5,pp6,pp7);

		CPolyHedron * tempPH = new CPolyHedron(pPolyhedron);
		if (!tempPH->is_closed())
		{
			Logger::output("error:the "+Logger::convInt2Str(i)+"th cube is un-closed!\n");
		} 

		float r = (rand()%256)*1.0/255.0;
		float g = (rand()%256)*1.0/255.0;
		float b = (rand()%256)*1.0/255.0;
		Colorf colour =Colorf(r,g,b,1.0f); 
		tempPH->set_color(colour);
		tempPH->set_show(true);
		
		CCube * aCube = new CCube(tempPH);
		aCube->m_v6Plns = m_vecBox6Plns[i];
		aCube->m_v8Pnts = m_vecBox8pnts[i];
		aCube->m_pc = this->m_pc;
		aCube->m_id = i;

		this->push_back(aCube);
	}

	this->m_bbox = CBoundingBox(minx,miny,minz,maxx,maxy,maxz);

	int SZ_polyhedron = this->size(); 
	Logger::output("Accomplished fit boxes (polyhedron). \nBox num: "+Logger::convInt2Str(SZ_polyhedron)+"\n"); 
}

void CCubeBoxes::identifyCubeNeighbor()
{
	int boxnum = m_numX * m_numY * m_numZ;
	if (this->size() != boxnum || this->m_vecBoxCoord.size() != boxnum) 
	{
		Logger::output("the number of boxes has problems.\n");
		return;
	}

	for (int i=0; i<this->size();i++)
	{
		CCube * aCube = this->at(i);
		_pCoord & p_coord = this->m_vecBoxCoord[i];
		int dx[6]={-1,1,0,0,0,0};
		int dy[6]={0,0,-1,1,0,0};
		int dz[6]={0,0,0,0,-1,1}; 
		// 6 neighbor cubes: left, right, front, back, top, bottom.
		for (int di = 0; di<6; di++)
		{
			int x_ = p_coord.xId+dx[di];
			int y_ = p_coord.yId+dy[di];
			int z_ = p_coord.zId+dz[di];
			if (x_>=0 && x_<m_numX && y_>=0 && y_<m_numY && z_>=0 && z_<m_numZ)
			{
				int pId = x_ * (m_numY*m_numZ) + y_ * m_numZ + z_;
				aCube->m_6neighborCubeId.push_back(pId);
			} 
		}
	}
}
  
void CCubeBoxes::drawFitBoxes(DrawMode drawmodel)
{
	switch (drawmodel)
	{
	case DM_CANDICATE_BOXES:  
		for (int i = 0;i<this->size(); i++)
		{
			CPolyHedron * ph =  this->at(i)->m_CPolyHedron;
			ph->drawPolyhedron(0.8f); 
		} 
		break;  
	case DM_OPTIMIZE_BOXES:
		if (this->m_finalPhModel!=NULL && lml_param::shift1)
		{
			this->m_finalPhModel->drawPolyhedron();
		}
		else
		{
			for (int i = 0;i<this->size(); i++)
			{ 
				CPolyHedron * ph =   this->at(i)->m_CPolyHedron;
				if (ph->onModel)
				{
					ph->drawPolyhedron(0.8f); 
					continue;
				} 
				if (!lml_param::shift1 && !lml_param::shift2 && ph->onModelTotally)
				{
					ph->toShow = true;
				}
				else if (lml_param::shift1 && !lml_param::shift2 && (ph->onModelTotally || ph->onModelMaybe))
				{
					ph->toShow = true;
				}
				else if (lml_param::shift2 && !lml_param::shift1  && ph->onModelMaybe)
				{
					ph->toShow = true;
				}
				else if (lml_param::shift1 && lml_param::shift2 && !ph->onModelTotally && !ph->onModelMaybe)
				{
					ph->toShow = true;
				}
				else
					ph->toShow = false;
				ph->drawPolyhedron(0.8f); 
			}
		}
		break;

	default: 
		break;
	}

} 

void CCubeBoxes::drawViewScore()
{ 
	if (!lml_param::shift1)
	{
		int Nb_box = this->size();
		for (int i=0;i<Nb_box; i++)
		{  
			CCube * aCube = this->at(i);
			aCube->draw_facetPnts();
		}  
	}
}
  
QString CCubeBoxes::genIthName(int i, const char * str)
{ 
	QString pname = str;
	pname += QString::number(i);
	return pname;
}
 
 
void CCubeBoxes::graphcutClassify()
{
	int numLabel = 2;	// two labels: 1 means inside the solid, 0 means at the outside.
	int numSite = m_numX*m_numY*m_numZ;

	GCoptimizationGeneralGraph *gc = new GCoptimizationGeneralGraph(numSite,numLabel);
	
	//-----1. set up the array for data costs-----  
	float *data = new float[numSite*numLabel];
	for (int xi=0; xi<m_numX; xi++)
	{
		for (int yi=0; yi<m_numY; yi++)
		{
			for (int zi=0;zi<m_numZ; zi++)
			{
				int boxId = xi*(m_numY * m_numZ) + yi *m_numZ +zi;
				double _score = this->at(boxId)->m_score;
				for (int li=0 ; li<numLabel ; li++) 
				{
					if(  li == 0 ) 
					{ data[boxId*numLabel+li] = _score;}
					else 
					{ data[boxId*numLabel+li] = -_score; }  
				}
			}
		}
	}

	//-----2. set neighbors
	for (int si =0; si<numSite; si++)
	{
		CCube * aCube = this->at(si);
		std::vector<int> & neigb = aCube->m_6neighborCubeId;
		for (int n_i=0;n_i<neigb.size();n_i++)
		{
			int & tempId = neigb[n_i];
			if (tempId<si) 
				continue; 
			else 
				gc->setNeighbors(si,tempId); 
		}
	}
	 
	//-----2. set up the array for smooth costs----- 
	float *smooth = new float[numLabel*numLabel];
	for ( int l1 = 0; l1 < numLabel; l1++ )
	{
		for (int l2 = 0; l2 < numLabel; l2++ )
		{
			smooth[l1+l2*numLabel] = (l1-l2)*(l1-l2) == 0 ? 0:lml_param::optim_lambda; 
		}
	} 

	//-----3. graph cut ----- 
	try
	{ 
		gc->setDataCost(data);
		gc->setSmoothCost(smooth);
		Logger::output("Before optimization energy is "+ Logger::convDouble2Str(gc->compute_energy())+"\n");
		gc->expansion(2);// run expansion for 2 iterations. For swap use gc->swap(num_iterations);
		Logger::output("After optimization energy is "+ Logger::convDouble2Str(gc->compute_energy())+"\n");
		
		for ( int  i = 0; i < numSite; i++ )
		{
			CCube * aCube = this->at(i);
			if (gc->whatLabel(i) == 0)
			{
				aCube->m_CPolyHedron->onModel = false;
				aCube->m_CPolyHedron->toShow  = false;
			}
			else
			{
				aCube->m_CPolyHedron->onModel = true;
				aCube->m_CPolyHedron->toShow  = true;
			}
		} 
		delete gc;
	}
	catch (GCException e)
	{
		e.Report();
	}

	delete [] smooth;
	delete [] data;  
}
 
void CCubeBoxes::mergeUnionPolyhedron()
{
	if (this->size()<2 || this->size()!= this->m_vecBox8pnts.size())
	{
		Logger::output("when merging polyhedron, it has size problems.\n");
		return;
	} 

	cgNef_polyhedron_3 nef_Union;

	Logger::output("[Start to merge union polyhedron.]\n");

	ProgressLogger progress(this->size());

	for (int i=0; i< this->size();i++)
	{
		progress.notify(i);
		if (progress.is_canceled())
			break;

		CPolyHedron * cph =   this->at(i)->m_CPolyHedron;
		if (cph->empty() || !cph->onModelTotally)
		{
			continue;
		}

		std::vector<CVector3D *> & v8Pnts = this->m_vecBox8pnts[i]; 

		this->convert_to_exact_polyhedron(v8Pnts,cph->m_nefPolyHedron);

		if (nef_Union.is_empty())
		{
			nef_Union = cph->m_nefPolyHedron;
		}
		else
		{
			nef_Union = nef_Union + cph->m_nefPolyHedron;
		} 
	}
	if (nef_Union.is_empty())
	{
		Logger::output("Unaccomplished merging the polyhedron to union.\n");
		return;
	} 
	
	nef_Union = nef_Union.closure();

	if (!nef_Union.is_simple())
	{
		Logger::output("Unable to merge the polyhedron to union.\n nefPolyhedron is not a simple kernel.\n");
		//return;
	}
	if (!nef_Union.is_valid())
	{
		Logger::output("Unable to merge the polyhedron to union.\n nefPolyhedron is not valid.\n");
		return;
	}
	int nb = nef_Union.number_of_vertices();
	Logger::output("union polyhedron vertex number: "+Logger::convInt2Str(nb)+"\n");
	if (nb<4)
	{
		return;
	}

	cgNewPolyhedron_3  tempPH; 
	nef_Union.convert_to_Polyhedron(tempPH);
	int nbFacet = tempPH.size_of_facets();
	Logger::output("union polyhedron facet number: ")<<(nbFacet)<<Logger::endl();
	//tempPH.fill_hole();
	Logger::output("Done union polyhedron!\nBuild CPolyhedron object ...\n");

	if (!tempPH.is_valid(true))
	{
		Logger::output("warn: the merged polyhedron is not valid!\n");
		return;
	}
	if (!tempPH.is_closed())
	{
		Logger::output("warn: the merged polyhedron is not closed!\n");
		return;
	}
	 
	cgPolyhedron final_PH;
	CPolyHedron::convert_PH_kernel(tempPH,final_PH);
	Logger::output("Done union polyhedron!\nBuild CPolyhedron object ...\n");
	this->m_finalPhModel = new CPolyHedron(final_PH);
	this->m_finalPhModel->colour = Colorf(0.9f,0.9f,0.9f,1.0f);
	this->m_finalPhModel->toShow = true;
	this->m_finalPhModel->onModel = true;
}

void CCubeBoxes::convert_to_exact_polyhedron(std::vector<CVector3D*> & v8Pnts, cgNef_polyhedron_3 &outNefPH)
{  
	typedef cgNef_polyhedron_3::Point_3 newPolyPnt_3;

	std::vector<newPolyPnt_3> cg8pnts;
	for (int i=0;i<8;i++)
	{ 
		double xi = v8Pnts[i]->x_();
		double yi = v8Pnts[i]->y_();
		double zi = v8Pnts[i]->z_();
		newPolyPnt_3 pi(xi,yi,zi);
		cg8pnts.push_back(pi);
	}

	cgNewPolyhedron_3 pPolyhedron;
	CPolyHedron::make_cube_3(pPolyhedron,cg8pnts[0],cg8pnts[1],cg8pnts[2],cg8pnts[3],cg8pnts[4],cg8pnts[5],cg8pnts[6],cg8pnts[7]);

	if (pPolyhedron.empty() || pPolyhedron.size_of_vertices ()<8 || !pPolyhedron.is_closed() || !pPolyhedron.is_pure_quad ())
	{ 
		Logger::output("\nerror: new kernal polyhedron problems!\n");
		return;
	}
	int sisis =0;
	for ( cgNewPolyhedron_3::Facet_iterator i = pPolyhedron.facets_begin(); i != pPolyhedron.facets_end(); ++i) 
	{
		cgNewPolyhedron_3::Halfedge_around_facet_circulator ci = i->facet_begin();
		if ( CGAL::circulator_size(ci) != 4)
		{
			continue;
		}
		cgNewPolyhedron_3::Halfedge_handle j1 = i->halfedge();

		cgNewPolyhedron_3::Halfedge_handle j2 = j1->next()->next();
		pPolyhedron.split_facet(j1,j2); 
	}

	outNefPH=cgNef_polyhedron_3(pPolyhedron); 
}

void CCubeBoxes::cal_cubeScore()
{
	int num = (int)this->size();
	Logger::output("[distribute the point cloud on each cube's facets!]\n");
	for (int i=0;i<num;i++)
	{
		CCube * aCube = this->at(i);
		aCube->dist_pnts_on6facet();
	}
	Logger::output("[calculate the cover rate of each cube's facets!]\n");
	for (int i=0;i<num;i++)
	{
		CCube * aCube = this->at(i);
		aCube->cal_6coverRate();
	}
}


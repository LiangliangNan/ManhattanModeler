#include "CFittingBox.h"
#include "Parameters.h" 
#include "CVertexGroup.h"
#include "CPointCloud.h"

#include <set>

CFittingBox::CFittingBox(CPointCloud* pc)
{
	m_pc = pc;
	num_xPln = 0;
	num_yPln = 0;
	num_zPln = 0;
}

CFittingBox::~CFittingBox(void)
{
	//while(vecBox8Pnts.empty()) 
	//{
	//	delete vecBox8Pnts.back();
	//	vecBox8Pnts.pop_back();
	//}
	//while(vecBox6Plns.empty()) 
	//{
	//	delete vecBox6Plns.back();
	//	vecBox6Plns.pop_back();
	//} 
}


// 1. detect if two plane are parallel and very close to each other. if so merge to one plane.
void CFittingBox::mergeSimilarPlns(std::vector<CVertexGroup*> & vPtrPG)
{
	unsigned int nbPlans =vPtrPG.size();
	std::cout<<nbPlans<<" planes are selected. combine similar planes...\n";
	std::set<int> idxs_to_delete;
	for (int i =0;i<nbPlans;i++)
	{
		cgPlane3f & pln_i = vPtrPG[i]->m_cgPlane;
		CVector3D & midpnti = vPtrPG[i]->m_midPnt;
		CVector3D nli (pln_i.a(),pln_i.b(),pln_i.c());

		for (int j =i+1;j<nbPlans;j++)
		{
			cgPlane3f & pln_j = vPtrPG[j]->m_cgPlane;
			CVector3D & midpntj = vPtrPG[j]->m_midPnt;
			CVector3D midpnts = midpnti - midpntj;
			CVector3D nlj (pln_j.a(),pln_j.b(),pln_j.c()); 
			double threshold_merg = lml_param::threshold_merge;
			if (nli*nlj>0.95 && abs(midpnts*nli)<threshold_merg && abs(midpnts*nlj)<threshold_merg)// threshold
			{
				vPtrPG[i]->mergeGroup(vPtrPG[j]);
				std::cout<<j+1<<"-th point cloud is add to "<<i+1<<std::endl;

				//std::cout<<vPtrPG[i]->size()<<" "<<vPtrPG[i]->m_pnts.size()<<"\n";
				vPtrPG[i]->refitPln_least_squares(); 

				idxs_to_delete.insert(j);
			}
		}
	}

	std::vector<int> tempIdx;
	for (std::set<int>::iterator it = idxs_to_delete.begin(); it!=idxs_to_delete.end(); it++ ) 
		tempIdx.push_back(*it); 
	for (int i=tempIdx.size()-1; i>=0; i--)
	{
		delete *(vPtrPG.begin()+tempIdx[i]);
		vPtrPG.erase(vPtrPG.begin()+tempIdx[i]); 
	}


}


// 2. sort the planes along axis direction by ascend order.
void CFittingBox::sortPlns(std::vector<CVertexGroup*> & vPG, std::vector<CVertexGroup*> & XvPln, std::vector<CVertexGroup*> & YvPln, std::vector<CVertexGroup*> & ZvPln)
{ 
	std::vector<double> vecX;
	std::vector<double> vecY;
	std::vector<double> vecZ;   

	XvPln.clear();
	YvPln.clear();
	ZvPln.clear(); 
	std::vector<int> XPlnIdx;
	std::vector<int> YPlnIdx;
	std::vector<int> ZPlnIdx;

	for (int i=0;i< vPG.size();i++)
	{ 
		CVertexGroup * opg =vPG[i];
		cgVector3f nrl;
		cgPoint3f midPnt = cgPoint3f(opg->m_midPnt.x_(),opg->m_midPnt.y_(),opg->m_midPnt.z_());
		if (abs(opg->m_cgPlane.a())>=0.866)		// normal.a>= sqr(3)/2. 60 degree
		{
			if (opg->m_cgPlane.a()>0)
				nrl = cgVector3f(1.0f,0,0);
			else
				nrl = cgVector3f(-1.0f,0,0);

			opg->m_cgPlane=cgPlane3f(midPnt,nrl);

			vecX.push_back(opg->m_midPnt.x_());
			XPlnIdx.push_back(i);
		}
		else if (abs(opg->m_cgPlane.b())>=0.866)  // normal.b>= sqr(3)/2. 60 degree
		{
			if (opg->m_cgPlane.b()>0)
				nrl = cgVector3f(0,1.0f,0);
			else
				nrl = cgVector3f(0,-1.0f,0);

			opg->m_cgPlane=cgPlane3f(midPnt,nrl);

			vecY.push_back(opg->m_midPnt.y_());
			YPlnIdx.push_back(i);
		} 
		else if (abs(opg->m_cgPlane.c())>=0.94)   // 70 degree
		{
			if (opg->m_cgPlane.c()>0)
				nrl = cgVector3f(0,0,1.0f);
			else
				nrl = cgVector3f(0,0,-1.0f);

			opg->m_cgPlane=cgPlane3f(midPnt,nrl);

			vecZ.push_back(opg->m_midPnt.z_());
			ZPlnIdx.push_back(i);
		}
		else
		{
			opg->m_toshow = false;
		}
	}


	// sort the plane in a ascent direction 
	int nb_xpln = XPlnIdx.size();
	int nb_ypln = YPlnIdx.size();
	int nb_zpln = ZPlnIdx.size();

	for (int i=0;i<nb_xpln;i++) 
	{ 
		for (int j=0; j<i; j++)
		{
			if (vecX[i]<vecX[j])
			{
				double tempX=vecX[i];
				vecX[i]=vecX[j];
				vecX[j]=tempX;
				int tempi=XPlnIdx[i];
				XPlnIdx[i]=XPlnIdx[j];
				XPlnIdx[j]=tempi;
			}
		}
	}

	for (int i=0;i<nb_ypln;i++) 
	{ 
		for (int j=0; j<i; j++)
		{
			if (vecY[i]<vecY[j])
			{
				double tempY=vecY[i];
				vecY[i]=vecY[j];
				vecY[j]=tempY;
				int tempi=YPlnIdx[i];
				YPlnIdx[i]=YPlnIdx[j];
				YPlnIdx[j]=tempi;
			}
		}
	} 

	for (int i=0;i<nb_zpln;i++) 
	{ 
		for (int j=0; j<i; j++)
		{
			if (vecZ[i]<vecZ[j])
			{
				double tempZ=vecZ[i];
				vecZ[i]=vecZ[j];
				vecZ[j]=tempZ;
				int tempi=ZPlnIdx[i];
				ZPlnIdx[i]=ZPlnIdx[j];
				ZPlnIdx[j]=tempi;
			}
		}
	}

	for (int i=0;i<XPlnIdx.size();i++)
	{
		XvPln.push_back(vPG[XPlnIdx[i]]);
	}
	for (int i=0;i<YPlnIdx.size();i++)
	{
		YvPln.push_back(vPG[YPlnIdx[i]]);
	}
	for (int i=0;i<ZPlnIdx.size();i++)
	{
		ZvPln.push_back(vPG[ZPlnIdx[i]]);
	} 
}

 
// 4. calculate the 8 points of each box.
void CFittingBox::cal_vecBox8Pnts(std::vector<std::vector<CVertexGroup*>> & tempVecBox6Plns,std::vector<std::vector<CVector3D*>> & p_vecBox8Pnts, std::vector<std::vector<CVertexGroup*>> & p_vecBox6Plns)
{
	for (unsigned int i=0; i< tempVecBox6Plns.size(); i++)
	{
		std::vector<CVertexGroup*> & aBox6PG = tempVecBox6Plns[i];
		std::vector<CVector3D*> box8p;
		for (unsigned int bi=0; bi<2; bi++)
		{ 
			cgPlane3f & pln_i=aBox6PG[bi]->m_cgPlane;
			for (unsigned int bj=2;bj<4;bj++)
			{
				cgPlane3f & pln_j=aBox6PG[bj]->m_cgPlane;
				for (unsigned int bk=4; bk<6;bk++)
				{ 
					cgPlane3f & pln_k=aBox6PG[bk]->m_cgPlane;
					CVector3D *p =new CVector3D();
					CPointCloud::calculateXYZ(pln_i,pln_j,pln_k,*p);

					box8p.push_back(p);
				}			
			}			
		}	
		if (checkPlnNoCross(box8p))
		{
			p_vecBox6Plns.push_back(aBox6PG);
			p_vecBox8Pnts.push_back(box8p);
		}
	} 
} 

bool CFittingBox::fitBox_0(std::vector<CVertexGroup*> & vPtrPG)
{ 
	vecBox6Plns.clear();
	vecBox8Pnts.clear();

	double threshold_Paral	=lml_param::threshold_Paral;	//   sin(75?
	double threshold_vertl	=lml_param::threshold_vertl;    //  >75 degree. cos(75?
  
	std::vector<std::vector<CVertexGroup*>> tempVecBox6Plns;

	typedef std::vector<int> pln6Idxes;
	std::vector<pln6Idxes> boxPlansIdxes;
	std::vector<pln6Idxes> box6PlnIdxes;

	// 1. merge similar planes.
	mergeSimilarPlns(vPtrPG); 
	unsigned int nbPlans = vPtrPG.size(); 
	std::cout<<nbPlans<<" planes are used to fitboxes. fit box ...\n";

	// 2. collect the 6 planes.
	for (int i=0;i<nbPlans-1;i++)
	{
		CVector3D nrli, nrlii, nrlj, nrljj, nrlk, nrlkk;

		cgPlane3f & pln_i =vPtrPG[i]->m_cgPlane;
		nrli.pVec[0]=pln_i.a(); nrli.pVec[1] =pln_i.b(); nrli.pVec[2]=pln_i.c();
		nrli.normalize();
		for (int ii=i+1;ii<nbPlans; ii++)
		{
			cgPlane3f & pln_ii = vPtrPG[ii]->m_cgPlane;
			nrlii.pVec[0]=pln_ii.a(); nrlii.pVec[1]=pln_ii.b(); nrlii.pVec[2] =pln_ii.c();
			nrlii.normalize();
			if (abs(nrlii*nrli)>threshold_Paral)  //***
			{		 
				for (int j=i+1; j<nbPlans; j++ )
				{
					cgPlane3f & pln_j = vPtrPG[j]->m_cgPlane;
					nrlj.pVec[0] =pln_j.a(); nrlj.pVec[1] =pln_j.b(); nrlj.pVec[2] = pln_j.c();
					nrlj.normalize();
					
					if (abs(nrlj*nrli)<threshold_vertl 
						&& abs(nrlj* nrlii)<threshold_vertl)  //***
					{
						for (int jj= j+1; jj<nbPlans; jj++)
						{
							cgPlane3f & pln_jj =vPtrPG[jj]->m_cgPlane;
							nrljj.pVec[0] = pln_jj.a(); nrljj.pVec[1] = pln_jj.b(); nrljj.pVec[2] =pln_jj.c();
							nrljj.normalize();

							if ( abs (nrljj*nrlii)<threshold_vertl 
								&& abs (nrljj*nrli)<threshold_vertl 
								&& abs (nrljj * nrlj)>threshold_Paral)//***
							{
								for (int k=j+1; k<nbPlans; k++)
								{
									cgPlane3f & pln_k =vPtrPG[k]->m_cgPlane;
									nrlk.pVec[0]=pln_k.a(); nrlk.pVec[1]=pln_k.b(); nrlk.pVec[2]=pln_k.c();
									nrlk.normalize();
									if (abs(nrlk*nrli)<threshold_vertl 
										&& abs(nrlk*nrlii)<threshold_vertl 
										&& abs(nrlk*nrlj)<threshold_vertl
										&& abs(nrlk*nrljj)<threshold_vertl)//***
									{
										for (int kk= k+1; kk<nbPlans; kk++)
										{
											cgPlane3f & pln_kk = vPtrPG[kk]->m_cgPlane;
											nrlkk.pVec[0]=pln_kk.a(); nrlkk.pVec[1]=pln_kk.b(); nrlkk.pVec[2]=pln_kk.c();
											nrlkk.normalize();
											if (abs(nrlkk*nrlk)>threshold_Paral
												&& abs(nrlkk* nrli)<threshold_vertl 
												&& abs(nrlkk* nrlii)<threshold_vertl 
												&& abs(nrlkk*nrlj)<threshold_vertl 
												&& abs(nrlkk*nrljj)<threshold_vertl)//***
											{
												std::vector<CVertexGroup*> onebox6pg;
												onebox6pg.push_back(vPtrPG[i]);
												onebox6pg.push_back(vPtrPG[ii]);
												onebox6pg.push_back(vPtrPG[j]);
												onebox6pg.push_back(vPtrPG[jj]);
												onebox6pg.push_back(vPtrPG[k]);
												onebox6pg.push_back(vPtrPG[kk]); 		
												tempVecBox6Plns.push_back(onebox6pg);
												std::vector<int> pln6;
												pln6.push_back(i);
												pln6.push_back(ii);
												pln6.push_back(j);
												pln6.push_back(jj);
												pln6.push_back(k);
												pln6.push_back(kk);
												boxPlansIdxes.push_back(pln6);
											}											
										}										
									}							
								}
							}													
						}						
					}					
				}				
			}			
		}	
	}		


	// 3. calculate the 8 corner point of each box.
	// 4. calculate the 8 corner point of each box.
	cal_vecBox8Pnts(tempVecBox6Plns,this->vecBox8Pnts,this->vecBox6Plns);

	if (vecBox6Plns.size()>0 && vecBox8Pnts.size()>0 && vecBox8Pnts.size()==vecBox6Plns.size())
		return true;
	else
		return false;
}


bool CFittingBox::fitBox_1(std::vector<CVertexGroup*> & vPtrPG)
{  

	vecBox6Plns.clear();
	vecBox8Pnts.clear();

	double threshold_Paral	=lml_param::threshold_Paral;	//   sin(75?
	double threshold_vertl	=lml_param::threshold_vertl;  //  >75 degree. cos(75?
	
	// 1. merge similar planes
	mergeSimilarPlns(vPtrPG);

	std::vector<CVertexGroup*> xPlns;
	std::vector<CVertexGroup*> yPlns;
	std::vector<CVertexGroup*> zPlns; 

	// 2. sort along X Y Z axis.
	sortPlns(vPtrPG, xPlns, yPlns, zPlns);

	int num_xPln = xPlns.size();
	int num_yPln = yPlns.size();
	int num_zPln = zPlns.size();
	if ( num_xPln<2 || num_yPln<2||num_zPln<2 )
	{ 
		std::cout<<"error: not enough planes for fiting boxes!\n";
		return false;
	} 

	// 3. collect the 6 planes.
	std::vector<std::vector<CVertexGroup*>> tempVecBox6Plns;
	for (int i=0;i<num_xPln-1; i++)
	{
		CVector3D nrl_x0, nrl_x1, nrl_y0, nrl_y1, nrl_z0, nrl_z1;
		cgPlane3f & pln_x0 = xPlns[i]->m_cgPlane;
		nrl_x0.pVec[0]=pln_x0.a(); 
		nrl_x0.pVec[1]=pln_x0.b(); 
		nrl_x0.pVec[2]=pln_x0.c();
			
		for (int ii = i+1; ii<num_xPln;ii++)
		{
			cgPlane3f & pln_x1 = xPlns[ii]->m_cgPlane;
			nrl_x1.pVec[0] = pln_x1.a(); 
			nrl_x1.pVec[1] = pln_x1.b(); 
			nrl_x1.pVec[2] = pln_x1.c(); 

			for (int j=0;j<num_yPln-1;j++)
			{
				cgPlane3f & pln_j = yPlns[j]->m_cgPlane;
				nrl_y0.pVec[0] =pln_j.a(); 
				nrl_y0.pVec[1] =pln_j.b(); 
				nrl_y0.pVec[2] = pln_j.c();

				for (int jj=j+1;jj<num_yPln;jj++)
				{
					cgPlane3f & pln_jj = yPlns[jj]->m_cgPlane;
					nrl_y1.pVec[0] = pln_jj.a(); 
					nrl_y1.pVec[1] = pln_jj.b(); 
					nrl_y1.pVec[2] =pln_jj.c(); 

					for (int k=0;k<num_zPln-1;k++)
					{
						cgPlane3f & pln_k = zPlns[k]->m_cgPlane;
						nrl_z0.pVec[0]=pln_k.a(); 
						nrl_z0.pVec[1]=pln_k.b(); 
						nrl_z0.pVec[2]=pln_k.c();

						for (int kk =k+1;kk<num_zPln;kk++)
						{
							cgPlane3f & pln_kk = zPlns[kk]->m_cgPlane;
							nrl_z1.pVec[0]=pln_kk.a(); 
							nrl_z1.pVec[1]=pln_kk.b(); 
							nrl_z1.pVec[2]=pln_kk.c();

							std::vector<CVertexGroup*> onebox6pg;
							onebox6pg.push_back(xPlns[i]);
							onebox6pg.push_back(xPlns[ii]);
							onebox6pg.push_back(yPlns[j]);
							onebox6pg.push_back(yPlns[jj]);
							onebox6pg.push_back(zPlns[k]);
							onebox6pg.push_back(zPlns[kk]); 		
							tempVecBox6Plns.push_back(onebox6pg); 
						}
					}
				}

			}
		}	
	}

	// 4. calculate the 8 corner point of each box.
	cal_vecBox8Pnts(tempVecBox6Plns,this->vecBox8Pnts,this->vecBox6Plns);

	if (vecBox6Plns.size()>0 && vecBox8Pnts.size()>0 && vecBox8Pnts.size()==vecBox6Plns.size())
		return true;
	else
		return false;
}
bool CFittingBox::fitBox_2(std::vector<CVertexGroup*> & vPtrPG )
{  
	this->vecBox6Plns.clear();
	this->vecBox8Pnts.clear(); 

	std::vector<CVertexGroup*> xPlns;
	std::vector<CVertexGroup*> yPlns;
	std::vector<CVertexGroup*> zPlns; 

	//  sort along X Y Z axis.
	sortPlns(vPtrPG, xPlns, yPlns, zPlns);

	//  distribute the point set to the new plane set.
	for (int pi=0;pi<vPtrPG.size();pi++)
	{
		CVertexGroup * pg = vPtrPG[pi];
		//pg->m_pnts.clear();
		//pg->clear();

		pg->refreshProjectionVtx();
	} 
	//double suqareDistThresh = lml_param::ransac_distance_threshold*lml_param::ransac_distance_threshold;
	//for (int pi=0;pi<vPtrPG.size();pi++)
	//{
	//	CVertexGroup * pg = vPtrPG[pi]; 
	//	for (int i=0;i<this->m_pc->m_vtxPC.size();i++)
	//	{
	//		CScanVertex * vtx = this->m_pc->m_vtxPC[i];
	//		pg->check_add(vtx,suqareDistThresh,lml_param::ransac_normalThresh);
	//	}
	//}
	// 

	num_xPln = xPlns.size();
	num_yPln = yPlns.size();
	num_zPln = zPlns.size();

	if ( num_xPln<2 || num_yPln<2||num_zPln<2 )
	{ 
		std::cout<<"error: not enough planes for fiting boxes!\n";
		return false;
	} 
 

	// 3. collect the 6 planes.
	std::vector<std::vector<CVertexGroup*>> tempVecBox6Plns;
	for (int i=0;i<num_xPln-1; i++)
	{  
		for (int j=0;j<num_yPln-1;j++)
		{
			for (int k=0;k<num_zPln-1;k++)
			{
				std::vector<CVertexGroup*> aBox6Pln;
				_pCoord aCoord = _pCoord(i,j,k);
				this->vecBoxCoord.push_back(aCoord);

				aBox6Pln.push_back(xPlns[i]);
				aBox6Pln.push_back(xPlns[i+1]);
				aBox6Pln.push_back(yPlns[j]);
				aBox6Pln.push_back(yPlns[j+1]);
				aBox6Pln.push_back(zPlns[k]);
				aBox6Pln.push_back(zPlns[k+1]); 		
				tempVecBox6Plns.push_back(aBox6Pln); 

			}
		}
	} 
 
	// 4. calculate the 8 corner point of each box.
	cal_vecBox8Pnts(tempVecBox6Plns,this->vecBox8Pnts,this->vecBox6Plns);

	if (vecBox6Plns.size()>0 && vecBox8Pnts.size()>0 && vecBox8Pnts.size()==vecBox6Plns.size())
		return true;
	else
		return false;
}

bool CFittingBox::checkPlnNoCross(std::vector<CVector3D*> & boxp)
{ // to make sure there is no plan pair that is too close and cross
	CVector3D & p_s0 = *boxp[0]; 
	CVector3D & p_s1 = *boxp[1]; 
	CVector3D & p_s2 = *boxp[2]; 
	CVector3D & p_s3 = *boxp[3]; 
	CVector3D & p_s4 = *boxp[4]; 
	CVector3D & p_s5 = *boxp[5]; 
	CVector3D & p_s6 = *boxp[6]; 
	CVector3D & p_s7 = *boxp[7]; 

	CVector3D v01 = p_s0-p_s1;
	CVector3D v04 = p_s0-p_s4;
	CVector3D v51 = p_s5-p_s1;
	CVector3D v54 = p_s5-p_s4;

	if ((v01^v04)*(v51^v54)>0)
	{
		return false;
	}


	CVector3D v23 = p_s2-p_s3;
	CVector3D v26 = p_s2-p_s6;
	CVector3D v73 = p_s7-p_s3;
	CVector3D v76 = p_s7-p_s6;

	if ((v23^v26)*(v73^v76)>0)
	{
		return false;
	}

	//CVector3D v01 = p_s0-p_s1;
	CVector3D v02 = p_s0-p_s2;
	CVector3D v31 = p_s3-p_s1;
	CVector3D v32 = p_s3-p_s2;

	if ((v01^v02)*(v31^v32)>0)
	{
		return false;
	}

	CVector3D v45 = p_s4-p_s5;
	CVector3D v46 = p_s4-p_s6;
	CVector3D v75 = p_s7-p_s5;
	//CVector3D v76 = p_s7-p_s6;

	if ((v45^v46)*(v75^v76)>0)
	{
		return false;
	}

	CVector3D v15 = p_s1-p_s5;
	CVector3D v13 = p_s1-p_s3;
	//CVector3D v75 = p_s7-p_s5;
	//CVector3D v73 = p_s7-p_s3;

	if ((v15^v13)*(v75^v73)>0)
	{
		return false;
	}

	//CVector3D v04 = p_s0-p_s4;
	//CVector3D v02 = p_s0-p_s2;
	CVector3D v64 = p_s6-p_s4;
	CVector3D v62 = p_s6-p_s2;

	if ((v04^v02)*(v64^v62)>0)
	{
		return false;
	}

	return true; 
}




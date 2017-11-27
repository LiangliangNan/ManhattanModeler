#ifndef TREE_ITEM_H
#define TREE_ITEM_H

#include <QTreeWidgetItem> 
#include "CPointCloud.h"
#include "CPolyHedron.h"
 
class CTreeItem : public QTreeWidgetItem
{
public:
	CTreeItem() : pc(nil),ph(nil),pg(nil),optzedPH(nil),bShow(false)
	{ 
		setFlags(flags() | Qt::ItemIsUserCheckable); 
	}

	~CTreeItem() {}  

	void setPntGroup(CVertexGroup * pg_) {pg = pg_;}
	  
	void setPntCloud(CPointCloud*  pc_){pc=pc_;}

	void setPolyHedron(CPolyHedron *ph_){ph=ph_;} 

	void setScoreNum(int num){scorePntNum=num;}

	void setOptimizedPolyHedron(CPolyHedron *ph_){optzedPH=ph_;}
	 

	void setShow(bool bl)
	{
		bShow = bl; 
		if (pg)
		{
			pg->m_toshow = bl;
		}
		if (pc)
		{
			pc->m_bSelect = bl;
			pc->toShow = bl;
		}
		if (ph)
		{
			ph->toShow = bl;
		}
		if (optzedPH)
		{
			optzedPH->toShow = bl;
		} 
	}
	bool getShow(){return bShow;}

public:

	bool bShow;
	CPolyHedron *ph;
	CVertexGroup *pg;
	CPointCloud* pc; 
private:
	int scorePntNum; 
	CPolyHedron *optzedPH;
};

#endif // TREE_ITEM_H

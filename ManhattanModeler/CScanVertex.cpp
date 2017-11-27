#include "CScanVertex.h"

CScanVertex::CScanVertex(void): color_error(NULL),color_height(NULL),selected_(false)
{
	_index = -1;
}

CScanVertex::~CScanVertex(void)
{
	if (this->color_error)
	{
		delete this->color_error;
		this->color_error = NULL;
	}
	if (this->color_height)
	{
		delete this->color_height;
		this->color_height = NULL;
	}
}

CScanVertex::CScanVertex( CVector3D & pnt,CVector3D & nrl, Colorf & clr ): color_error(NULL),color_height(NULL),selected_(false)
{
	point_ = pnt;
	normal_ = nrl;
	color_ = clr; 
}
CScanVertex::CScanVertex(CScanVertex & vtx): color_error(NULL),color_height(NULL),selected_(false)
{
	point_ = vtx.point_;
	normal_= vtx.normal_;
	color_ = vtx.color_;
	selected_ = vtx.selected_;
}
 
 
void CScanVertex::setColor( double r,double g,double b,double a /*=1*/ )
{
	color_ = Colorf(r,g,b,a);
}

 
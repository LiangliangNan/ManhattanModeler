
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



#include "CDlgAddPlane.h"


CDlgAddPlane::CDlgAddPlane(QWidget *parent)
{
	setupUi(this);

	parentWindow = (MainWindow*)parent;
	connect(pushButton, SIGNAL(clicked()), this, SLOT(accept())); 
}


CDlgAddPlane::~CDlgAddPlane(void)
{
}

void CDlgAddPlane::accept()
{ 
	mx= this->lineEdit->text().toDouble(); 
	my= this->lineEdit_2->text().toDouble();
	mz= this->lineEdit_3->text().toDouble();
	nx= this->lineEdit_4->text().toDouble(); 
	ny= this->lineEdit_5->text().toDouble();
	nz= this->lineEdit_6->text().toDouble();

	this->parentWindow->runAddPln(mx, my, mz, nx, ny, nz);

	this->close();
}

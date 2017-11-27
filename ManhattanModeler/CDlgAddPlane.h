
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



#pragma once
 

#include <QDialog>
#include "ui_dlgAddPln.h"
#include "main_window.h"

class CDlgAddPlane: public QDialog, public Ui::dlgAddPlane
{
public:

	Q_OBJECT
public:
	CDlgAddPlane(QWidget *parent);
	~CDlgAddPlane(void); 

	private slots: 
		void accept();
private:
	MainWindow *parentWindow;
	 
	double mx,my,mz;
	double nx,ny,nz;
};


#include "DlgransacParam.h"
#include "Parameters.h"

CDlg_ransacParam::CDlg_ransacParam(QWidget *parent)
{
	setupUi(this);

	parentWindow = (MainWindow*)parent;
	connect(btnRANSAC, SIGNAL(clicked()), this, SLOT(accept())); 
 
	//this->setResult(QDialog::Rejected);
}

CDlg_ransacParam::~CDlg_ransacParam(void)
{
}

void CDlg_ransacParam::accept()
{
	lml_param::ransac_bitmap_resolution	= this->line_ransac_bitmap->text().toDouble();
	lml_param::ransac_normalThresh		= this->line_ransac_nrlThresh->text().toDouble();
	lml_param::ransac_minimum_support	= this->line_ransac_minimum->text().toDouble();
	lml_param::ransac_probability		= this->line_ransac_probability->text().toDouble();
	lml_param::ransac_distance_threshold= this->line_ransac_distance->text().toDouble(); 
	//this->setResult(QDialog::Accepted);
	this->done(QDialog::Accepted);
}

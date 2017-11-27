#include "DlgFittingBox.h"
#include "Parameters.h"

DlgFittingBox::DlgFittingBox(QWidget *parent)
{
	setupUi(this);

	parentWindow = (MainWindow*)parent;
	connect(pushButton, SIGNAL(clicked()), this, SLOT(accept())); 
}

DlgFittingBox::~DlgFittingBox(void)
{
}

void DlgFittingBox::accept()
{ 
	lml_param::threshold_Paral= this->line_parall->text().toFloat(); 
	lml_param::threshold_vertl= this->line_vertical->text().toFloat();
	lml_param::threshold_merge= this->line_merge->text().toFloat();

	this->parentWindow->runFittingBox();

	this->close();
}

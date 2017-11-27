#ifndef DLGRANSACDETECTIONPARAMETERS_H
#define DLGRANSACDETECTIONPARAMETERS_H

#include <QDialog>
#include "ui_dlg_param_control.h"

class CDlgParameterControl : public QDialog, public Ui::CDlgParamControl
{
	Q_OBJECT

public:
	CDlgParameterControl(QWidget *parent = 0);
	~CDlgParameterControl();

private slots:
	void apply();
	void reset();	

private:
	QString default_min_support_;
	QString default_distance_threshold_;
	QString default_bitmap_resolution_;
};

#endif // DLGRANSACDETECTIONPARAMETERS_H

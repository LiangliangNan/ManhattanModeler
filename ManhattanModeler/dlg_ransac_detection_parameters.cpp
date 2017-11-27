#include "dlg_ransac_detection_parameters.h"
#include "../algorithm/ransac_detector.h"
#include "../geometry/geometry_global.h"


#include <QIntValidator>
#include <QDoubleValidator>



CDlgParameterControl::CDlgParameterControl(QWidget *parent)
	: QDialog(parent)
{
	setupUi(this);

	default_min_support_ = QString("%1").arg(Geom::ransac_minimum_support);
	default_distance_threshold_ = QString("%1").arg(Geom::ransac_distance_threshold);
	default_bitmap_resolution_ = QString("%1").arg(Geom::ransac_bitmap_resolution);

	lineEditMinimumSupport->setValidator(new QIntValidator(10, 1000000, this));
	lineEditDistanceThreshold->setValidator(new QDoubleValidator(0.00001, 1.0, 5, this));
	lineEditBitmapResolution->setValidator(new QDoubleValidator(0.00001, 1.0, 5, this));

	reset();

	connect(resetButton, SIGNAL(clicked()), this, SLOT(reset()));
	connect(applyButton, SIGNAL(clicked()), this, SLOT(apply()));
	connect(okButton, SIGNAL(clicked()), this, SLOT(apply()));
	connect(okButton, SIGNAL(clicked()), this, SLOT(accept()));
}

CDlgParameterControl::~CDlgParameterControl()
{

}


void CDlgParameterControl::apply()
{
	Geom::ransac_minimum_support = lineEditMinimumSupport->text().toUInt(); 
	Geom::ransac_distance_threshold = lineEditDistanceThreshold->text().toDouble();
	Geom::ransac_bitmap_resolution = lineEditBitmapResolution->text().toDouble();		 
}

void CDlgParameterControl::reset()
{
	lineEditMinimumSupport->setText(default_min_support_);
	lineEditDistanceThreshold->setText(default_distance_threshold_);
	lineEditBitmapResolution->setText(default_bitmap_resolution_);
}

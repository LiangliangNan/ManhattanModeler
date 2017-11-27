
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
#include <QMessageBox>
#include <QFileDialog>
#include <QLabel>
#include <QStatusBar>
#include <QSettings>
#include <QCloseEvent>
#include <QPlainTextEdit>
#include <QAbstractItemModel>
#include <QStandardItemModel>
#include <QGroupBox>
#include <QColorDialog>
#include <QProgressBar>
#include <QComboBox> 
 

#include <math.h>
#include <fstream>
#include <iostream>
#include <set> 

#include "main_window.h"
#include "paint_canvas.h" 
#include "math_global.h"  
 

#include "DlgransacParam.h"
#include "DlgFittingBox.h" 
#include "simpleWidget.h" 
#include "CDlgAddPlane.h"
 
#include "ransac_detector.h"
#include <string.h>
#include <cstring> 
#include "tree_item.h"

#include "ioFile.h"
#include "Parameters.h"
 
#include "CoordTransf.h"
#include "CFittingBox.h"  
 
#include "logger.h"  
 

using namespace std;
using namespace qglviewer;


MainWindow::MainWindow(QWidget *parent, Qt::WindowFlags flags)
	: QMainWindow(parent, flags)
	, curDataDirectory_(".")
	, m_bigTheta(0)
	, dlg_ransacParam(nil)
	,dlg_fittingbox(nil)
	,dlg_pointSize(nil) 
	,dlg_addPln(nil)
{	 

	setupUi(this); 
	Progress::instance()->set_client((ProgressClient *)this);

	Logger::initSet((LoggerClient*)this);
	//showDlgMRFModeler();

	// creat main paint Canvas. 
	QGLFormat format = QGLFormat::defaultFormat();
	format.setSampleBuffers(true);
	format.setSamples(8);

	mainCanvas_ = new PaintCanvas(CT_FRONT, format, this);
	mainCanvas_->setAttribute(Qt::WA_MouseTracking); 
	mainCanvas_->camera()->setType(Camera::PERSPECTIVE); 
	setCentralWidget(mainCanvas_);
 
	createTreeWidgetTable();
	createComboBox();
	createStatusBar();
	createActionsForFileMenu();
	createActionsForToolBar();
	createMenus();
	connect(actionAbout, SIGNAL(triggered()), this, SLOT(about()));
	connect(actionAboutQt, SIGNAL(triggered()), this, SLOT(aboutQt()));

	readSettings();
	setWindowTitle("ManhattanModeler");

	setContextMenuPolicy(Qt::CustomContextMenu);
	setWindowIcon(QIcon(":/Resources/icon.png"));

	//setFixedSize(1200, 900);
	setWindowState(Qt::WindowMaximized);

	setFocusPolicy(Qt::ClickFocus);

	showMaximized();

	actionTransparent->setChecked(true);
	actionLight->setChecked(true);


	//recentFiles_ = "";
	curMeshFileName_= ""; 
   
	m_NumOfTreeItems=0;
	hasRANSAC = false;
	hasMW = false;
	//bigTheta=0;
	m_unChechAll = false;
	m_ChechAll = false;
	m_isIntitial_tree = false;  
	lml_param::optim_lambda = 50;
}

MainWindow::~MainWindow()
{
	Progress::instance()->set_client(NULL);
}


void MainWindow::closeEvent(QCloseEvent *event)
{
	writeSettings();
}
 
void MainWindow::notify_progress(int value) 
{
	int last_value = progress_bar_->value()>0? progress_bar_->value():0;
	progress_bar_->setValue(value);
	progress_bar_->setTextVisible((last_value == 0 && value == 0) || value != 0);
	this->mainCanvas_->forceUpdate();
}

void MainWindow::createComboBox() 
{
	// tool bar
	comboBoxDrawMode_ = new QComboBox(toolBar);
	comboBoxDrawMode_->setObjectName("Draw mode");
	comboBoxDrawMode_->setEditable(false);
	comboBoxDrawMode_->setMinimumHeight(26);
	comboBoxDrawMode_->addItem("Initial PC");
	comboBoxDrawMode_->addItem("RANSAC Result");  
	comboBoxDrawMode_->addItem("Box Candidate");
	comboBoxDrawMode_->addItem("Box Optimize"); 
	comboBoxDrawMode_->addItem("DM_MESH");
	connect(comboBoxDrawMode_, SIGNAL(currentIndexChanged(QString)), this, SLOT(comBoxItemChanged(QString)));

	toolBar->addSeparator();
	toolBar->addWidget(comboBoxDrawMode_);
	
	// combobox for algorithm parameter.
	comboBoxParamType = new QComboBox(dockWidget_Algorithm0);  
	comboBoxParamType->setObjectName("comboBoxParamType");
	comboBoxParamType->setEditable(false); 
	comboBoxParamType->setMinimumHeight(26);
	comboBoxParamType->addItem("Box Optimal Lambda");
	comboBoxParamType->addItem("GroundZ");   

	lineEditTxt = new QLineEdit(dockWidget_Algorithm0);
	lineEditTxt->setObjectName(QString::fromUtf8("lineEditTxt")); 
	lineEditTxt->setText( "50");
	connect(lineEditTxt, SIGNAL(editingFinished()), this, SLOT(editTxtChangeSlot()));
	 
	QHBoxLayout *hLayout = new QHBoxLayout(groupBox_3);
	hLayout->setSpacing(2);
	hLayout->setContentsMargins(2, 2, 2, 2);
	hLayout->setObjectName(QString::fromUtf8("hLayout"));
	hLayout->addWidget(comboBoxParamType);
	hLayout->addWidget(lineEditTxt);

	// comboBox for testing small functions. 
	comboBox_functions->setEditable(true); 
	comboBox_functions->addItem("Add Arbitrary Plane");
	//comboBox_functions->addItem("addPlanes");  
	comboBox_functions->addItem("addGroundPlane");
	connect(pushBtn_functions, SIGNAL(clicked()), this, SLOT(testSmallFunctions()));
}

void MainWindow::createStatusBar()
{	
	statusLabel = new QLabel("Ready");
	statusLabel->setAlignment(Qt::AlignHCenter);
	statusLabel->setMinimumSize(statusLabel->sizeHint());
	statusBar()->addWidget(statusLabel);

	spacerLabel = new QLabel;
	statusBar()->addWidget(spacerLabel, 0);

	coordinateUnderMouseLabel = new QLabel(this);
	coordinateUnderMouseLabel->setAlignment(Qt::AlignLeft);
	//coordinateUnderMouseLabel->setMinimumSize(coordinateUnderMouseLabel->sizeHint());
	statusBar()->addWidget(coordinateUnderMouseLabel, 1); 

	coordPixelUnderMouseLabel = new QLabel(this);
	coordPixelUnderMouseLabel->setAlignment(Qt::AlignLeft);
	statusBar()->addWidget(coordPixelUnderMouseLabel,1);

	//QPushButton* cancelButton = new QPushButton;
	//connect(cancelButton, SIGNAL(pressed()), this,  SLOT(cancelTask()));
	//cancelButton->setIcon(QIcon(":/Resources/cancel.png"));
	//statusBar()->addPermanentWidget(cancelButton);

	numVerticesLabel = new QLabel;
	numVerticesLabel->setIndent(80);
	numVerticesLabel->setAlignment(Qt::AlignRight);
	//numVerticesLabel->setMinimumSize(numVerticesLabel->sizeHint()); 
	statusBar()->addWidget(numVerticesLabel, 0);

	progress_bar_ = new QProgressBar;
	progress_bar_->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
	progress_bar_->setFixedWidth(400);
	statusBar()->addPermanentWidget(progress_bar_, 0);

	updateStatusBar();
}
void MainWindow::updateStatusBar()
{ 
	size_t nb_vertices = 0;

	if (this->mainCanvas_->m_ptrPC)
	{
		DrawMode & dm = this->mainCanvas_->draw_mode_;
		if (dm == DM_INITIAL_POINTS) 
			nb_vertices = this->mainCanvas_->m_ptrPC->m_vtxPC.size();
	}

	numVerticesLabel->setText( QString("vertices: %1").arg(nb_vertices) );
} 

void MainWindow::showCoordinateUnderMouse( float p[3], bool found )
{ 
	if (found) 
	{
		QString coordString = QString(" XYZ = [%1, %2, %3]").arg(p[0]).arg(p[1]).arg(p[2]);
		coordinateUnderMouseLabel->setText(coordString);
	} else 
	{
		coordinateUnderMouseLabel->setText(" XYZ = [-0.0, -0.0, -0.0]");
	}
}

void MainWindow::showPixelPositionUnderMouse(const QPoint& p) {
	QString coordString = QString(" XY = [%1, %2]").arg(p.x()).arg(p.y());
	coordPixelUnderMouseLabel->setText(coordString); 
}


void MainWindow::createMenus() 
{
	menuFile->addSeparator();
	actionClean = new QAction(this); 
	actionClean->setObjectName(QString::fromUtf8("clean_pc"));
	actionClean->setVisible(true);
	actionClean->setText("clean pc");
	connect(actionClean, SIGNAL(triggered()),this,SLOT(cleanModel()));
	//menuFile->addAction(actionClean);
	menuFile->insertAction(actionExit, actionClean);
	actionSeparator = menuFile->addSeparator();
}

void MainWindow::createTreeWidgetTable()
{
	QStringList scanHeaderLabels;
	scanHeaderLabels << "Index" << "Show" << "Preview" << "R" << "G" << "B";
	treeWidgetTable->setHeaderLabels(scanHeaderLabels);
	//treeWidgetColorTable->setSortingEnabled(true);
	treeWidgetTable->headerItem()->setTextAlignment(0, Qt::AlignHCenter);

	treeWidgetTable->setColumnWidth(0, 40);
	treeWidgetTable->setColumnWidth(1, 40);
	treeWidgetTable->setColumnWidth(2, 50);
	treeWidgetTable->setColumnWidth(3, 70);
	treeWidgetTable->setColumnWidth(4, 70);
	treeWidgetTable->setColumnWidth(5, 70);
	treeWidgetTable->setSelectionMode(QAbstractItemView::ExtendedSelection);
	//treeWidgetColorTable->setEditTriggers(QAbstractItemView::SelectedClicked);
	
	
	//////////////////////////////////////////////////////////////////////////
	connect(treeWidgetTable, SIGNAL(currentItemChanged(QTreeWidgetItem*, QTreeWidgetItem*)), this, SLOT(boxTableCurrentItemChanged(QTreeWidgetItem*, QTreeWidgetItem*)));
	connect(treeWidgetTable, SIGNAL(itemChanged ( QTreeWidgetItem * , int )),this, SLOT(boxTableItemChanged(QTreeWidgetItem*, int)));
	connect(treeWidgetTable, SIGNAL(itemSelectionChanged ()),this, SLOT(boxTabSelectChanged()));
	connect(pushButtonCheckAll, SIGNAL(clicked()), this, SLOT(check_all_Items())); 
	connect(pushButtonUncheckAll, SIGNAL(clicked()), this, SLOT(uncheck_All_Items()));  

	connect(btn_Load_Items, SIGNAL(clicked()), this, SLOT(load_PCGroup()));
	connect(btn_Export_Items, SIGNAL(clicked()), this, SLOT(export_Items()));
} 
 
void MainWindow::createActionsForFileMenu() 
{
	//////////////////////////////////////////////////////////////////////////
	for (int i = 0; i < MaxRecentFiles; ++i) 
	{
		actionsRecentFile[i] = new QAction(this);
		actionsRecentFile[i]->setVisible(false);
		connect(actionsRecentFile[i], SIGNAL(triggered()), this, SLOT(openRecentFile()));
	}

	connect(actionOpen, SIGNAL(triggered()), this, SLOT(openModelFile()));
	actionOpen->setShortcut(QString("Ctrl+O"));

	connect(actionSaveModel, SIGNAL(triggered()), this, SLOT(saveViewingItems()));
	actionSaveModel->setShortcut(QString("Ctrl+S"));

	connect(actionExit, SIGNAL(triggered()), this, SLOT(close()));
	actionExit->setShortcut(QString("Ctrl+Q"));
	  
	// window view action
	connect(actionDump_output_window,SIGNAL(triggered()),this,SLOT(showWidget_DumpOutput()));
	connect(actionItem_information_window,SIGNAL(triggered()),this,SLOT(showWidget_ItemInfor()));
	connect(actionAlgorithm_fitboxWindow,SIGNAL(triggered()),this,SLOT(showWidget_AlgorithmFitBox()));
	connect(actionViewing_item_window,SIGNAL(triggered()),this,SLOT(showWidget_ViewItems())); 
	   

}


void MainWindow::about()
{
	QString text = QMessageBox::tr(
		"<p><h4> Version 1.0</h4></p>"
		"<p>This program implements the reconstruction method described in:<br>"
		"   <span style=\"font-style:italic;\">Manhattan-world Urban Reconstruction from Point Clouds.</span> ECCV 2016.<br>"
		"   Minglei Li (<a href=\"mailto:minglei_li@126.com\">minglei_li@126.com</a>)<br>"
		"   Peter Wonka (<a href=\"mailto:pwonka@gmail.com\">pwonka@gmail.com</a>)<br>"
		"   Liangliang Nan (<a href=\"mailto:liangliang.nan@gmail.com\">liangliang.nan@gmail.com</a>)<br>"
		"All rights reserved.</p>"
		);

	//QMessageBox::about(this, title, text);
	QMessageBox::about(this, "About ManhattanModeler", text);
}

void MainWindow::aboutQt()
{
	QMessageBox::aboutQt(this, tr("About Qt"));
}


void MainWindow::createActionsForToolBar() 
{  
	connect(actionOpenFile,SIGNAL(triggered()),this,SLOT(openModelFile()));

	//////////////////////////////////////////////////////////////////////////
	connect(actionSnap_shot,SIGNAL(triggered()),this,SLOT(snapshot()));
	connect(actionFullScreen, SIGNAL(toggled(bool)), this, SLOT(setFullScreen(bool)));
	menuView->insertAction(actionFullScreen, dockWidget_ItemInfor->toggleViewAction());
	menuView->insertSeparator(actionFullScreen);
	connect(actionFitWindow, SIGNAL(triggered()), mainCanvas_, SLOT(fitScreen())); 
	connect(actionTransparent,SIGNAL(toggled(bool)),this,SLOT(ONOFF_Transparent(bool)));
	connect(actionLight, SIGNAL(toggled(bool)),this,SLOT(ONOFF_Light(bool))); 

	connect(actionShowWire,SIGNAL(toggled(bool)),this,SLOT(ONOFF_wire(bool)));

  
	/////  fit boxes  algorithm/////////////////////////////////////////////////////////////////////
	//connect(btnSetRANSAC,SIGNAL(clicked()),this,SLOT(showDlg_ransacParam()));
	connect(btnRANSAC,SIGNAL(clicked()),this,SLOT(ransacAct())); 
	connect(btnConvertMW,SIGNAL(clicked()),this,SLOT(conPCtoMW()));
	connect(btnFitbox,SIGNAL(clicked()),this,SLOT(showDlgFittingBox()));
	connect(btnScore,SIGNAL(clicked()),this,SLOT(calculateScore())); 

	//////  checkBox   ////////////////////////////////////////////////////////////////////
	connect(checkBox_PC,SIGNAL(clicked()), this, SLOT(canvasShowCheckChange()));  
	connect(checkBox_RANSAC,SIGNAL(clicked()), this, SLOT(canvasShowCheckChange()));   
	connect(checkBox_Polyhedron,SIGNAL(clicked()),this,SLOT(canvasShowCheckChange()));
	connect(checkBox_rdmColor,SIGNAL(clicked()),this,SLOT(canvasShowCheckChange()));

	
	checkBox_PC->setCheckState(Qt::Checked); // Qt::Checked	2	The item is checked.
	checkBox_RANSAC->setCheckState(Qt::Checked); 
	checkBox_Polyhedron->setCheckState(Qt::Checked); 
	checkBox_rdmColor->setCheckState(Qt::Checked); 

	//////////////////////////////////////////////////////////////////////////
	////btnSetRANSAC->setVisible(false);
	//btnRANSAC->setText("Extract Planes");
	//btnConvertMW->setText("Re-orient");
	// 
	//btnFitbox->setText("Candidate Box"); 
	//btnScore->setText("Optimization"); 
}
 
 

void MainWindow::setFullScreen(bool b)
{
	if (b)
		showFullScreen();
	else
		showMaximized();
}

//"JPEG, PNG, EPS, PS, PPM, BMP"
bool MainWindow::snapshot() 
{
	if (lml_param::is_drawCornerAxis) 
	{
		lml_param::is_drawCornerAxis = false;
		this->mainCanvas_->update(); 

		mainCanvas_->saveSnapshot(false);

		lml_param::is_drawCornerAxis = true;
		this->mainCanvas_->update(); 
	}
	else
		mainCanvas_->saveSnapshot(false); 
	return true;
} 

void MainWindow::readSettings()
{
	QSettings settings("VCC-KAUST", "ManhattanModeler");

	recentFiles_ = settings.value("recentFiles").toStringList();
	updateRecentFileActions();

	curDataDirectory_ = settings.value("currentDirectory").toString();	
}

void MainWindow::writeSettings()
{
	QSettings settings("VCC-KAUST", "ManhattanModeler");
	settings.setValue("recentFiles", recentFiles_);
	settings.setValue("currentDirectory", curDataDirectory_);
}
 

void MainWindow::setCurrentOpenFile(const QString &fileName)
{
	curMeshFileName_ = fileName;
	curDataDirectory_ = fileName.left(fileName.lastIndexOf("/"));

	setWindowModified(false);

	QString shownName = "Untitled";
	if (!curMeshFileName_.isEmpty())
	{
		shownName = strippedName(curMeshFileName_);
		recentFiles_.removeAll(curMeshFileName_);
		recentFiles_.prepend(curMeshFileName_);
		updateRecentFileActions();
	}

	setWindowTitle(tr("%1[*] - %2").arg(shownName).arg(tr("BuildingMolde")));
}


bool MainWindow::saveViewingItems()
{
	QString fileName = QFileDialog::getSaveFileName(this, 
		tr("Save file"), curMeshFileName_,
		tr("Supported format (*.ply *.obj)")
		);

	if (fileName.isEmpty())
		return false;

	std::string str = fileName.toStdString();
	char * flName = new char[str.size() + 1];
	std::copy(str.begin(), str.end(), flName);
	flName[str.size()] = '\0'; // don't forget the terminating 0

	if (this->mainCanvas_->draw_mode_ == DM_MESH)
	{
		if (this->mainCanvas_->m_meshObj)
		{
			this->mainCanvas_->m_meshObj->SaveToObj(flName);
			return true;
		}
		if (this->mainCanvas_->m_ptrPC && this->mainCanvas_->m_ptrPC->m_MeshObj)
		{
			this->mainCanvas_->m_ptrPC->m_MeshObj->SaveToObj(flName);
			return true;
		}
		return false;
	}
	else if (this->mainCanvas_->draw_mode_ == DM_INITIAL_POINTS && this->mainCanvas_->m_ptrPC->toShow)
	{
		this->mainCanvas_->m_ptrPC->save_ply(flName);
	}
	else if (this->mainCanvas_->m_fitBoxes && this->mainCanvas_->m_fitBoxes->m_finalPhModel !=NULL)
	{
		//this->mainCanvas_->m_fitBoxes->m_finalPhModel->save_PHGroup_Ply()
		std::vector<CPolyHedron*> tempObj;
		tempObj.push_back(this->mainCanvas_->m_fitBoxes->m_finalPhModel);
		CPolyHedron::save_PHGroup_Ply(tempObj,fileName.toStdString().c_str(),m_bigTheta);
	} 

	else if (mainCanvas_->draw_mode_ == DM_CANDICATE_BOXES || mainCanvas_->draw_mode_ == DM_OPTIMIZE_BOXES)
	{
		std::vector<CPolyHedron*> phGroup;
		for (unsigned int i = 0; i < m_NumOfTreeItems; ++i)
		{
			CTreeItem* item = dynamic_cast<CTreeItem*>(treeWidgetTable->topLevelItem(i));
			if (item == nil) {
				std::cout << "fatal error" << std::endl;
				return false;
			}
			else if (item->checkState(1) == Qt::Checked){
				CPolyHedron * c = item->ph;
				phGroup.push_back(c);
			}
		}
		std::string ext = lml_IO::get_filename_extension(fileName.toStdString());
		if (ext == "ply")
		{
			//CPolyHedron::save_PHGroup_Ply(phGroup,fileName.toStdString().c_str());
			CPolyHedron::save_PHGroup_Ply(phGroup, fileName.toStdString().c_str(), m_bigTheta);
		}
		if (ext == "obj")
		{
			CPolyHedron::save_PHGroup_Obj(phGroup, fileName.toStdString().c_str(), m_bigTheta);
		}

		return true;
	}
	delete[] flName;
	return 1;
}


void MainWindow::updateRecentFileActions()
{
	QMutableStringListIterator i(recentFiles_);
	while (i.hasNext()) {
		if (!QFile::exists(i.next()))
			i.remove();
	}

	for (int j = 0; j < MaxRecentFiles; ++j) {
		if (j < recentFiles_.count()) {
			QString text = tr("&%1 %2").arg(j + 1).arg(strippedName(recentFiles_[j]));
			actionsRecentFile[j]->setText(text);
			actionsRecentFile[j]->setData(recentFiles_[j]);
			actionsRecentFile[j]->setVisible(true);
		} else {
			actionsRecentFile[j]->setVisible(false);
		}
	}

	actionSeparator->setVisible(!recentFiles_.isEmpty());
}

QString MainWindow::strippedName(const QString &fullFileName)
{
	return QFileInfo(fullFileName).fileName();
}


void MainWindow::check_all_Items() 
{ 
	m_isIntitial_tree =true;
	int nb = m_NumOfTreeItems;
	for (int i=0; i<nb; ++i) 
	{
		CTreeItem* item = dynamic_cast<CTreeItem*>(treeWidgetTable->topLevelItem(i));
		if (item == nil) 
		{
			item = new CTreeItem;
			treeWidgetTable->addTopLevelItem(item);
		}		
		item->setCheckState(1, Qt::Checked);
	}
	m_ChechAll = true;
	treeWidgetTable->update();
	mainCanvas_->updateGL();
	m_isIntitial_tree = false;
}

void MainWindow::uncheck_All_Items() 
{
	m_isIntitial_tree =true;
	int nb = m_NumOfTreeItems;
	for (unsigned int i=0; i<nb; ++i) 
	{
		CTreeItem* item = dynamic_cast<CTreeItem*>(treeWidgetTable->topLevelItem(i));
		if (item == nil) 
		{
			item = new CTreeItem;
			treeWidgetTable->addTopLevelItem(item);
		}		
		item->setCheckState(1, Qt::Unchecked);
	}
	m_unChechAll = true;

	treeWidgetTable->update();
	mainCanvas_->updateGL();
	m_isIntitial_tree = false;
}

bool MainWindow::load_PCGroup()
{
	QString fileName = QFileDialog::getOpenFileName(this,
		tr("Open file"), curDataDirectory_,
		tr(
		"PCGroup (*.blab)\n"
		"All files (*.*)")
		);

	if (fileName.isEmpty())
		return false;
	std::vector<CVertexGroup*> pcGroup;
	lml_IO::load_blab_to_PCGroup(fileName.toStdString(),pcGroup);
	mainCanvas_->m_vPntGroup = pcGroup;
	mainCanvas_->setDrawMode(DM_RANSAC_GROUPS);
	this->hasRANSAC = true;
	this->updateTreewidget();
	mainCanvas_->updateGL();
	return true;
}
  

bool MainWindow::export_Items()
{ 
	if (mainCanvas_->draw_mode_ == DM_RANSAC_GROUPS)
	{
		QString fileName = QFileDialog::getSaveFileName(this,
			tr("Save file"), curDataDirectory_,
			tr(
			"Blab (*.blab)\n"
			"All files (*.*)")
			);

		if (fileName.isEmpty())
			return false;
 
		std::vector<CVertexGroup*> vPtrPG;
		for (unsigned int i=0; i<mainCanvas_->m_vPntGroup.size(); ++i) {
			CTreeItem* item = dynamic_cast<CTreeItem*>(treeWidgetTable->topLevelItem(i));
			if (item == nil) {
				std::cout << "fatal error" << std::endl;
				return false;
			} else if (item->checkState(1) == Qt::Checked){
				CVertexGroup * c = item->pg; 
				vPtrPG.push_back(c);
			}
		}

		lml_IO::write_blab_From_RANSAC(fileName.toStdString().c_str(),vPtrPG);
		return true;
	}
	if (mainCanvas_->draw_mode_ == DM_CANDICATE_BOXES || mainCanvas_->draw_mode_ == DM_OPTIMIZE_BOXES)
	{
		QString fileName = QFileDialog::getSaveFileName(this,
			tr("Save file"), curDataDirectory_,
			tr(
			"obj (*.obj)\nply (*.ply)\n"
			"All files (*.*)")
			);

		if (fileName.isEmpty())
			return false; 
		std::vector<CPolyHedron*> phGroup;
		for (unsigned int i=0; i<m_NumOfTreeItems; ++i)
		{
			CTreeItem* item = dynamic_cast<CTreeItem*>(treeWidgetTable->topLevelItem(i));
			if (item == nil) {
				std::cout << "fatal error" << std::endl;
				return false;
			} else if (item->checkState(1) == Qt::Checked){
				CPolyHedron * c = item->ph; 
				phGroup.push_back(c);
			}
		}
		std::string ext = lml_IO::get_filename_extension(fileName.toStdString());
		if (ext == "ply")
		{
			//CPolyHedron::save_PHGroup_Ply(phGroup,fileName.toStdString().c_str());
			CPolyHedron::save_PHGroup_Ply(phGroup,fileName.toStdString().c_str(),m_bigTheta);
		}
		if (ext == "obj")
		{
			CPolyHedron::save_PHGroup_Obj(phGroup,fileName.toStdString().c_str(),m_bigTheta);
		}
		
		return true;
	} 
	return false;
}
  
void MainWindow::cleanModel()
{
	this->mainCanvas_->reset();
	updateTreewidget(); 
	mainCanvas_->updateGL(); 
}


bool MainWindow::openModelFile()
{
	QString fileName = QFileDialog::getOpenFileName(this,
		tr("Open file"), curDataDirectory_,
		tr(
		"Points (*.ply *.obj *.blab *.off *.xyzn *.tif *.img *.mls)\n"
		"All files (*.*)")
		);

	if (fileName.isEmpty())
		return false;
  
	std::string ext = lml_IO::get_filename_extension(fileName.toLocal8Bit().constData());
	if (ext == "ply") 
	{
		return (openPly(fileName)) ;
	} 
	else if (ext == "off")  
	{	
		std::string tempStr = fileName.toLocal8Bit().constData();
		const char * chrFile = tempStr.c_str();
		CPolyHedron * tempPH = new CPolyHedron();
		if (!tempPH->read_Polyhedron_From_OFF(chrFile)) 
		{
			return false;
		}   
		mainCanvas_->setMesh(tempPH); 
		return true;
	}  
	else if (ext == "obj") 
	{	
		std::string tempStr = fileName.toLocal8Bit().constData();
		CMeshObj * mesh =  CMeshObj::loadFromObj(tempStr); 

		this->mainCanvas_->setMesh(mesh);

		setComBoxDWItem(DM_MESH);
		updateTreewidget();

		mainCanvas_->updateGL();
		return true; 
	}  
	else if (ext=="blab")
	{
		std::vector<CVertexGroup*> pcGroup;
		lml_IO::load_blab_to_PCGroup(fileName.toLocal8Bit().constData(),pcGroup);
		mainCanvas_->m_vPntGroup = pcGroup;
		mainCanvas_->setDrawMode(DM_RANSAC_GROUPS);
		this->hasRANSAC = true;
		this->updateTreewidget();
		return true;
	}
	else if (ext =="xyzn")
	{
		CPointCloud *ptCloud = new CPointCloud();
		std::string tempStr = fileName.toStdString();
		const char * chrFile = tempStr.c_str();
		lml_IO::load_PC_From_xyzn(ptCloud,chrFile);
		int sz = (int)ptCloud->m_vtxPC.size();
		output("[Open] "+QString::number(sz).toStdString());
		output(" points have been loaded from ");
		output(fileName.toStdString());
		output("!\n"); 
		mainCanvas_->addPC(ptCloud);
		setCurrentOpenFile(fileName);
		setComBoxDWItem(DM_INITIAL_POINTS);
		canvas()->updateGL();
		return true;
	}  
	 
	updateStatusBar();
	return true;
}


void MainWindow::openRecentFile()
{
	Logger::output("Unfinished function.\n");
	QAction *action = qobject_cast<QAction *>(sender());
	if (action) 
	{
		QString filename(action->data().toString());
		QFileInfo info(action->data().toString());
		QString ext = info.suffix().toLower(); 
		if (ext == "ply") 
		{
			openPly(filename);
		} 
		else if (ext == "off")  
		{	
			std::string tempStr = filename.toStdString();
			const char * chrFile = tempStr.c_str();
			CPolyHedron * tempPH = new CPolyHedron();
			if (!tempPH->read_Polyhedron_From_OFF(chrFile)) 
			{
				return;
			}   
			mainCanvas_->setMesh(tempPH); 
			return ;
		} 

		else if (ext == "obj") 
		{	
			std::string tempStr = filename.toStdString();
			CMeshObj * mesh =  CMeshObj::loadFromObj(tempStr); 

			this->mainCanvas_->setMesh(mesh);

			setComBoxDWItem(DM_MESH);
			updateTreewidget();

			mainCanvas_->updateGL();
			return; 
		}  
		else if (ext=="blab")
		{
			std::vector<CVertexGroup*> pcGroup;
			lml_IO::load_blab_to_PCGroup(filename.toStdString(),pcGroup);
			mainCanvas_->m_vPntGroup = pcGroup;
			mainCanvas_->setDrawMode(DM_RANSAC_GROUPS);
			this->hasRANSAC = true;
			this->updateTreewidget();
			return;
		}
		else if (ext =="xyzn")
		{
			CPointCloud *ptCloud = new CPointCloud();
			std::string tempStr = filename.toStdString();
			const char * chrFile = tempStr.c_str();
			lml_IO::load_PC_From_xyzn(ptCloud,chrFile);
			int sz = (int)ptCloud->m_vtxPC.size();
			output("[Open] "+ QString::number(sz).toStdString());
			output(" points have been loaded from ");
			output(filename.toStdString());
			output("!\n"); 
			mainCanvas_->addPC(ptCloud);
			setCurrentOpenFile(filename);
			setComBoxDWItem(DM_INITIAL_POINTS);
			canvas()->updateGL();
			return;
		} 

		updateStatusBar();
	}
}
  
bool MainWindow::openPly(QString & fileName)
{ 
	char * cstr = new char[fileName.length() + 1];
	strcpy(cstr,fileName.toLocal8Bit().constData());
 
	CPointCloud *ptCloud = new CPointCloud();
	CPointCloud::load_ply(cstr,ptCloud);

	if (ptCloud)
	{	  
		ptCloud->build_boundingBox();
		int sz = ptCloud->m_vtxPC.size(); 
		Logger::output(QString::number(sz).toStdString()+" point loaded from  "+fileName.toStdString()+"\n"); 
		mainCanvas_->addPC(ptCloud);
		setCurrentOpenFile(fileName);
		setComBoxDWItem(DM_INITIAL_POINTS); 
		updateTreewidget();
		updateStatusBar();
		canvas()->updateGL();
		return true;
	} else
	return false;
}
  

void MainWindow::ransacAct()
{  
	if (!mainCanvas_->m_ptrPC)
	{
		output("[Warn] There is no point cloud data!\n");
		return;
	}
	else
	{  
		if (!showDlg_ransacParam()) // dialog: set parameter for ransac.
			return; 

		ransac();
	}
}

void MainWindow::ransac()
{ 
	output("[RANSAC plane extraction]...\n");
	clock_t t0=clock();
	mainCanvas_->m_vPntGroup.clear(); 
	RansacDetector::ransac_apply(mainCanvas_->m_ptrPC,mainCanvas_->m_vPntGroup);
	hasRANSAC = true; 

	mainCanvas_->setGroup();
	int sz = mainCanvas_->m_vPntGroup.size();
	output(QString::number(sz).toStdString());
	output(" planes have been detected!\n"); 

	double pTime = (clock()-t0)*1000.0/CLOCKS_PER_SEC; 
	output(Logger::convDouble2Str(pTime)+" ms\n");
	setComBoxDWItem(DM_RANSAC_GROUPS);
	updateTreewidget() ;
}  

void MainWindow::conPCtoMW() // transform the point cloud to align on Manhattan World.
{ 
	if (!hasRANSAC)
	{
		output("[error] need to run RANSAC to detect the dominant plane first.\n");
		return;
	}
	if (this->mainCanvas_->m_ptrPC->hasMW)
	{
		return;
	}

	output("[Align to MW]...\n");
	clock_t t0=clock();
	double	bigTheta;
	int maxI=-1; // index of the dominate plane
	unsigned int maxNb=0; // the largest number of the dominate plane.
	double a , b;
	std::vector<CVertexGroup*> &pVecPntGroup = mainCanvas_->m_vPntGroup;
	for (int i=0;i< pVecPntGroup.size();i++)
	{ 
		if (abs(pVecPntGroup[i]->m_cgPlane.c())>=0.1)
		{
			continue;
		}
		else
		{
			if (maxNb<pVecPntGroup[i]->size())
			{
				maxNb=pVecPntGroup[i]->size(); // find the main vertical plane.
				maxI=i;
				b = pVecPntGroup[i]->m_cgPlane.b();
				a = pVecPntGroup[i]->m_cgPlane.a();

			}			
		}
	}
	if (maxI !=-1)
	{ 
		CCoordTransf cdt;
		bigTheta=atan(b/a);
		m_bigTheta = bigTheta;
		for (int pi=0;pi< pVecPntGroup.size();pi++)
		{
			cdt.transGroupPC(*(pVecPntGroup[pi]),bigTheta,TR_XY);
		}
		CPointCloud & pc = *(this->mainCanvas_->m_ptrPC);
		CPointCloud::transPntCloud(pc,bigTheta,TR_XY); 
		pc.deletKDtree();
		pc.hasMW = true; 
		hasMW = true;
		mainCanvas_->setGroup();
		setComBoxDWItem(DM_RANSAC_GROUPS);
		updateTreewidget() ;
		//ransac();
	} 
	else
	{
		output("[Error] no significant Manhattan structure detected.\n");
		output("Maybe it's better to manually adjust the point clouds.\n");
		return;
	}  
	double pTime = (clock()-t0)*1000.0/CLOCKS_PER_SEC; 
	output(Logger::convDouble2Str(pTime)+" ms\n");
}




void MainWindow::showDlgFittingBox()
{
	if (mainCanvas_->m_vPntGroup.size()<6)
	{
		output("[Warn] there is no enough point group!");
		return;
	}	
	if (mainCanvas_->draw_mode_!=DM_RANSAC_GROUPS)
	{
		output("[Warn] require ransac models!\n");
		return;
	}  

	if (this->dlg_fittingbox != NULL)
	{
		dlg_fittingbox->close();
		delete dlg_fittingbox;
	}
	this->dlg_fittingbox = new DlgFittingBox(this); 
	//A modal dialog

	this->dlg_fittingbox->show();
	this->dlg_fittingbox->raise();
	this->dlg_fittingbox->activateWindow();

}
void MainWindow::runFittingBox()
{  
	if (!hasMW)
	{
		////Discarded method
		//b_finish = fb->fitBox_0(vPtrPG);
		Logger::output("[Warn] Align to Manhattan World first.\n");
		return;
	} 

	Logger::output("[fit boxes from selected planes]\n");
	std::vector<CVertexGroup*> vPtrPG;
	for (unsigned int i=0; i<mainCanvas_->m_vPntGroup.size(); ++i) 
	{
		CTreeItem* item = dynamic_cast<CTreeItem*>(treeWidgetTable->topLevelItem(i));
		if (item == nil) 
		{
			Logger::output("[error] unrecognized plane element. Please refresh combox.\n"); 
			return;
		} else if (item->checkState(1) == Qt::Checked)
		{
			CVertexGroup * c = item->pg; 
			vPtrPG.push_back(c);
		}
	}

	clock_t t0= clock(); 
	//  fit boxes.
	/// CFittingBox is a class that is focused on processing fitting boxes
	/// CFitBoxes   is a class that stores the structure of boxes after fitting job.
	CFittingBox * fb = new CFittingBox(this->mainCanvas_->m_ptrPC);
	fb->mergeSimilarPlns(vPtrPG);
	this->mainCanvas_->resetVtxGroup(vPtrPG);


	bool b_finish = false;

	b_finish = fb->fitBox_2(vPtrPG);
	//   after fitting boxes, construct the boxes (cube\polyhedron).
	CCubeBoxes * fitboxes = new CCubeBoxes(this->mainCanvas_->m_ptrPC,fb); 

	mainCanvas_->setBoxes(fitboxes);

	setComBoxDWItem(DM_CANDICATE_BOXES); 


	int ms = (clock()-t0)*1000.0/CLOCKS_PER_SEC;
	QString str = QString::number(ms);
	Logger::output("Time: "+ str.toStdString() +" ms\n"); 

	this->updateTreewidget();
	this->mainCanvas_->updateGL();
}


/// Algorithm 1. fitting boxes.
// 2. calculate the score of every boxes.
void MainWindow::calculateScore()
{
	if (!mainCanvas_->m_fitBoxes)
	{
		output("[warn] no box candidates. Please fitting boxes first.\n");
		return;
	} 
	//////////////////////////////////////////////////////////////////////////
	{
		clock_t t0= clock(); 

		this->mainCanvas_->m_fitBoxes->graphcutClassify();

		int ms = (clock()-t0)*1000.0/CLOCKS_PER_SEC;
		QString str = QString::number(ms);
		output("Time: "+ str.toStdString() +" ms\n"); 

		setComBoxDWItem(DM_OPTIMIZE_BOXES); 
		this->updateTreewidget(); 
		this->mainCanvas_->updateGL();
		return; 
	}

	////////////////////////////////////////////////////////////////////////////
	//{
	//	clock_t t0= clock(); 

	//	this->mainCanvas_->m_fitBoxes->optimizationFunc(lml_param::optim_lambda);

	//	int ms = (clock()-t0)*1000.0/CLOCKS_PER_SEC;
	//	QString str = QString::number(ms);
	//	Logger::output("Time: "+ str.toStdString() +" ms\n"); 

	//	setComBoxDWItem(DM_OPTIMIZE_BOXES); 
	//	this->updateTreewidget(); 
	//	this->mainCanvas_->fitScreen();
	//	this->mainCanvas_->updateGL();
	//	return; 
	//} 
} 
 


void MainWindow::boxTableItemChanged(QTreeWidgetItem* it, int col)
{
	CTreeItem* item = dynamic_cast<CTreeItem*>(it);
	if (!item)
		return;

	if (col == 1) 
	{ 
		if (item->checkState(1) == Qt::Checked)
			item->setShow(true);
		else
			item->setShow(false);
		 
		if (!m_isIntitial_tree)
		{
			mainCanvas_->updateGL();
		} 
	} 	 
}
void MainWindow::boxTableCurrentItemChanged( QTreeWidgetItem *current, QTreeWidgetItem *previous )
{ 
	//if (item0) 
	//{
	//	item0->setShow(!(item0->bShow));
	//	if (item0->bShow)
	//	{
	//		item0->setCheckState(1, Qt::Checked);
	//	}
	//	else
	//	{
	//		item0->setCheckState(1, Qt::Unchecked);
	//	}
	//}
	//if (m_ChechAll || m_unChechAll)
	//{
	//	m_ChechAll =false;
	//	m_unChechAll = false;
	//	canvas()->updateGL();
	//	return;
	//} 
}

void MainWindow::boxTabSelectChanged()
{
	QList<QTreeWidgetItem *> lst = treeWidgetTable->selectedItems();
	for (int i =0;i<lst.size();i++)
	{ 
		CTreeItem* item0 = dynamic_cast<CTreeItem*>(lst[i]);
		item0->setCheckState(1, Qt::Checked); 
	}
	if (lst.size()>0)
	{
		CTreeItem* item0 = dynamic_cast<CTreeItem*>(lst.back());
		this->mainCanvas_->m_ptrPC = item0->pc;
		updateStatusBar();
	}
}


void MainWindow::updateTreewidget() 
{   
	treeWidgetTable->clear(); 
	treeWidgetTable->setCurrentItem(nil); 
	m_NumOfTreeItems = 0;

	m_isIntitial_tree = true;

	QStringList scanHeaderLabels;

	switch (mainCanvas_->draw_mode_) 
	{
	case  DM_INITIAL_POINTS: 
		treeWidgetTable->clear();
		treeWidgetTable->setHeaderLabels(scanHeaderLabels);
		scanHeaderLabels << "ID" << "Show"<<"File" << "Num" ;
		treeWidgetTable->setHeaderLabels(scanHeaderLabels);
		treeWidgetTable->headerItem()->setTextAlignment(0, Qt::AlignHCenter);
		treeWidgetTable->setColumnWidth(0, 40);
		treeWidgetTable->setColumnWidth(1, 40);
		treeWidgetTable->setColumnWidth(2, 40);
		treeWidgetTable->setColumnWidth(3, 50); 
		treeWidgetTable->setSelectionMode(QAbstractItemView::ExtendedSelection);
		if (mainCanvas_->m_vPC.size()>0)
		{ 
			unsigned int nPC=mainCanvas_->m_vPC.size(); 
			for (unsigned int i=0; i<nPC; i++)
			{ 
				CTreeItem* item = dynamic_cast<CTreeItem*>(treeWidgetTable->topLevelItem(i)); 
				if (item == nil) 
				{
					item = new CTreeItem;
					treeWidgetTable->addTopLevelItem(item); 
				}
				CPointCloud * opg = mainCanvas_->m_vPC[i];
				item->setPntCloud(opg); 
				item->setShow(opg->toShow);

				int idx = i + 1; 
				item->setData(0, Qt::DisplayRole, idx);

				if (opg->toShow) 
					item->setCheckState(1, Qt::Checked); 
				else 
					item->setCheckState(1, Qt::Unchecked); 

				QColor c(255,255,255);
				item->setData(2, Qt::DecorationRole, c);
				item->setData(3, Qt::DisplayRole, opg->m_vtxPC.size()); 
				m_NumOfTreeItems++;
			}
		}
		break;
	case DM_MESH:
		treeWidgetTable->clear();
		treeWidgetTable->setHeaderLabels(scanHeaderLabels);
		break;
	case DM_RANSAC_GROUPS:  
		treeWidgetTable->clear();
		treeWidgetTable->setHeaderLabels(scanHeaderLabels);
		scanHeaderLabels << "ID" << "Show"<< "view" << "nx" << "ny" << "nz" ;
		treeWidgetTable->setHeaderLabels(scanHeaderLabels);
		//treeWidgetColorTable->setSortingEnabled(true);
		treeWidgetTable->headerItem()->setTextAlignment(0, Qt::AlignHCenter);

		treeWidgetTable->setColumnWidth(0, 40);
		treeWidgetTable->setColumnWidth(1, 40);
		treeWidgetTable->setColumnWidth(2, 40);
		treeWidgetTable->setColumnWidth(3, 40);
		treeWidgetTable->setColumnWidth(4, 40);
		treeWidgetTable->setColumnWidth(5, 40);
		treeWidgetTable->setSelectionMode(QAbstractItemView::ExtendedSelection);
		if (mainCanvas_->m_vPntGroup.size()>0)
		{ 
			unsigned int nbGroup=mainCanvas_->m_vPntGroup.size(); 
			for (unsigned int i=0; i<nbGroup; i++)
			{ 
				CTreeItem* item = dynamic_cast<CTreeItem*>(treeWidgetTable->topLevelItem(i)); 
				if (item == nil) 
				{
					item = new CTreeItem;
					treeWidgetTable->addTopLevelItem(item); 
				}
				CVertexGroup * opg = mainCanvas_->m_vPntGroup[i];
				item->setPntGroup(mainCanvas_->m_vPntGroup[i]); 
				item->setShow(opg->m_toshow);
  
				int		idx = i + 1; 
				item->setData(0, Qt::DisplayRole, idx);

				if (opg->m_toshow)
				{
					item->setCheckState(1, Qt::Checked);
				}
				else
				{
					item->setCheckState(1, Qt::Unchecked);
				}

				QColor c(
					(int)(opg->m_color.r()*255),
					(int)(opg->m_color.g()*255),
					(int)(opg->m_color.b()*255));
				item->setData(2, Qt::DecorationRole, c);
				item->setData(3, Qt::DisplayRole, opg->m_cgPlane.a());
				item->setData(4, Qt::DisplayRole, opg->m_cgPlane.b());
				item->setData(5, Qt::DisplayRole, opg->m_cgPlane.c());
				m_NumOfTreeItems++;
			}
		}
		break;
	 
	case DM_CANDICATE_BOXES:
		treeWidgetTable->clear();
		treeWidgetTable->setHeaderLabels(scanHeaderLabels);
		scanHeaderLabels << "ID" << "Show" << "view" << "volume" << "Num" << "Score";
		treeWidgetTable->setHeaderLabels(scanHeaderLabels);
		//treeWidgetColorTable->setSortingEnabled(true);
		treeWidgetTable->headerItem()->setTextAlignment(0, Qt::AlignHCenter);

		treeWidgetTable->setColumnWidth(0, 40);
		treeWidgetTable->setColumnWidth(1, 40);
		treeWidgetTable->setColumnWidth(2, 40);
		treeWidgetTable->setColumnWidth(3, 50);
		treeWidgetTable->setColumnWidth(4, 60);
		treeWidgetTable->setColumnWidth(5, 60);
		treeWidgetTable->setSelectionMode(QAbstractItemView::ExtendedSelection);
		if (mainCanvas_->m_fitBoxes)
		{
			for (unsigned int i=0; i<mainCanvas_->m_fitBoxes->size(); ++i) 
			{
				CTreeItem* item = dynamic_cast<CTreeItem*>(treeWidgetTable->topLevelItem(i)); 
				if (item == nil) 
				{
					item = new CTreeItem;
					treeWidgetTable->addTopLevelItem(item); 
				}
				CPolyHedron * ph =  mainCanvas_->m_fitBoxes->at(i)->m_CPolyHedron;

				item->setPolyHedron(ph);
				item->setShow(ph->toShow);

				int		idx = i + 1; 
				item->setData(0, Qt::DisplayRole, idx);

				if (ph->toShow)
				{
					item->setCheckState(1, Qt::Checked);
				}
				else
				{
					item->setCheckState(1, Qt::Unchecked);
				}
				int cr = (int)(ph->colour.r()*255)%256;
				int cg = (int)(ph->colour.g()*255)%256;
				int cb = (int)(ph->colour.b()*255)%256;
				if (cr>255){cr=255;}
				if (cr<0){cr=0;} 
				if (cg>255){cg=255;}
				if (cg<0){cg=0;} 
				if (cb>255){cb=255;}
				if (cb<0){cb=0;} 
				//std::cout<<<<" "<<<<" "<<<<std::endl;
				QColor c(cr,cg,cb);
				item->setData(2, Qt::DecorationRole, c);
				item->setData(3, Qt::DisplayRole, item->ph->getVolum()); 
				item->setData(4, Qt::DisplayRole, mainCanvas_->m_fitBoxes->at(i)->m_numPntOnCube);  
				item->setData(5, Qt::DisplayRole, mainCanvas_->m_fitBoxes->at(i)->m_score); 
				m_NumOfTreeItems++;
			}
		}
		break;

	case DM_OPTIMIZE_BOXES:
		treeWidgetTable->clear();
		treeWidgetTable->setHeaderLabels(scanHeaderLabels);
		scanHeaderLabels << "ID" << "Show" << "view" << "volume" << "Num" << "Score";
		treeWidgetTable->setHeaderLabels(scanHeaderLabels);
		//treeWidgetColorTable->setSortingEnabled(true);
		treeWidgetTable->headerItem()->setTextAlignment(0, Qt::AlignHCenter);

		treeWidgetTable->setColumnWidth(0, 40);
		treeWidgetTable->setColumnWidth(1, 40);
		treeWidgetTable->setColumnWidth(2, 40);
		treeWidgetTable->setColumnWidth(3, 50);
		treeWidgetTable->setColumnWidth(4, 60);
		treeWidgetTable->setColumnWidth(5, 60);
		treeWidgetTable->setSelectionMode(QAbstractItemView::ExtendedSelection);
		if (mainCanvas_->m_fitBoxes>0)
		{
			for (unsigned int i=0; i<mainCanvas_->m_fitBoxes->size(); ++i) 
			{
				CPolyHedron * ph =   mainCanvas_->m_fitBoxes->at(i)->m_CPolyHedron;
				if (ph->onModel == false)
				{
					continue;
				}
				CTreeItem* item = dynamic_cast<CTreeItem*>(treeWidgetTable->topLevelItem(i)); 
				if (item == nil) 
				{
					item = new CTreeItem;
					treeWidgetTable->addTopLevelItem(item); 
				}
				item->setPolyHedron(ph);

				int		idx = i + 1; 
				item->setData(0, Qt::DisplayRole, idx);

				if (ph->toShow)
				{
					item->setCheckState(1, Qt::Checked);
				}
				else
				{
					item->setCheckState(1, Qt::Unchecked);
				}
				int cr = (int)(ph->colour.r()*255)%256;
				int cg = (int)(ph->colour.g()*255)%256;
				int cb = (int)(ph->colour.b()*255)%256;
				if (cr>255){cr=255;}
				if (cr<0){cr=0;} 
				if (cg>255){cg=255;}
				if (cg<0){cg=0;} 
				if (cb>255){cb=255;}
				if (cb<0){cb=0;} 
				//std::cout<<<<" "<<<<" "<<<<std::endl;
				QColor c(cr,cg,cb);
				item->setData(2, Qt::DecorationRole, c);
				item->setData(2, Qt::DecorationRole, c);
				item->setData(3, Qt::DisplayRole, item->ph->getVolum());
				item->setData(4, Qt::DisplayRole, mainCanvas_->m_fitBoxes->at(i)->m_numPntOnCube); 
				item->setData(5, Qt::DisplayRole, mainCanvas_->m_fitBoxes->at(i)->m_score); 
				m_NumOfTreeItems++;
			}
		}
		break;
	 
	 
	}
	m_isIntitial_tree = false;
	treeWidgetTable->update();
}


void MainWindow::setComBoxDWItem( DrawMode mode )
{
	switch (mode)
	{
	case DM_INITIAL_POINTS:
		comboBoxDrawMode_->setCurrentIndex(0);
		break;
	case DM_RANSAC_GROUPS:
		comboBoxDrawMode_->setCurrentIndex(1);
		break; 
	case DM_CANDICATE_BOXES:
		comboBoxDrawMode_->setCurrentIndex(2);
		break;
	case DM_OPTIMIZE_BOXES:
		comboBoxDrawMode_->setCurrentIndex(3);
		break; 
	case  DM_MESH:
		comboBoxDrawMode_->setCurrentIndex(4);
		break;
	default:
		break;
	}
	update();
}

void MainWindow::comBoxItemChanged( QString str)
{ 
	if (str == "Initial PC")
		mainCanvas_->setDrawMode(DM_INITIAL_POINTS);
	else if (str == "RANSAC Result")
		mainCanvas_->setDrawMode(DM_RANSAC_GROUPS);  
	else if (str == "Box Candidate")
		mainCanvas_->setDrawMode(DM_CANDICATE_BOXES);
	else if (str == "Box Optimize")
		mainCanvas_->setDrawMode(DM_OPTIMIZE_BOXES);
	else if (str == "DM_MESH")
		mainCanvas_->setDrawMode(DM_MESH);

	updateTreewidget();

	//updateStatusBar();
	mainCanvas_->updateGL();
} 
 
void MainWindow::editTxtChangeSlot()
{
	QString newValue = this->lineEditTxt->text();
	if (this->comboBoxParamType->currentText() == "Box Optimal Lambda")
	{	
		lml_param::optim_lambda = newValue.toDouble();
		Logger::output("set lambda of fit box to "+newValue.toStdString()+"\n");
	}
	else if (this->comboBoxParamType->currentText() == "GroundZ")
	{ 
		lml_param::GroundZ = newValue.toDouble();
		Logger::output("set ground z value to " +newValue.toStdString()+"\n");
	}
}

void MainWindow::testSmallFunctions()
{ 
	if (this->comboBox_functions->currentText() =="Add Arbitrary Plane")
	{  
		if (this->dlg_addPln ==NULL)
		{
			this->dlg_addPln= new CDlgAddPlane(this);
		}

		this->dlg_addPln->exec();
		this->dlg_addPln->raise();
		this->dlg_addPln->isActiveWindow();
	}
	else if (this->comboBox_functions->currentText() =="addGroundPlane")
	{ 
		this->runAddPln(0,0,lml_param::GroundZ,0,0,-1);
		return;
	}
	else  if (this->comboBox_functions->currentText() == "addPlanes") //  rarely used method, to add planes from depth-map gradient.
	{
		if (this->mainCanvas_->m_vPntGroup.size()<1 || this->mainCanvas_->m_ptrPC->m_vtxGroup_depthmap.size()<1)
		{
			Logger::output("Warn: there is no vertex groups.\n");
			return;
		}
		else
		{ 
			Logger::output("[Add the depth planes to canvas's PG]\n");
			for (int i=0; i<this->mainCanvas_->m_ptrPC->m_vtxGroup_depthmap.size(); i++)
			{
				CVertexGroup * aGroup = this->mainCanvas_->m_ptrPC->m_vtxGroup_depthmap[i];
				this->mainCanvas_->m_vPntGroup.push_back(aGroup);
			}
			this->mainCanvas_->m_ptrPC->m_vtxGroup_depthmap.clear();
			return;
		}
	}
}


void MainWindow::runAddPln( double px,double py, double pz, double nx, double ny, double nz )
{ 
	if (this->mainCanvas_->m_vPntGroup.size()<1)
	{
		Logger::output("Warn: there is no vertex groups.\n");
		return;
	}
	else
	{ 
		Logger::output("[Add a plane to canvas's PG]\n"); 
		CVertexGroup * aGroup = new CVertexGroup();
		aGroup->m_cgPlane = cgPlane3f(cgPoint3f(px, py, pz),cgVector3f(nx, ny, nz));
		aGroup->m_midPnt = CVector3D(px, py, pz); 
		aGroup->m_color = Colorf(0.9f,0.9f,0.9f,1);
		this->mainCanvas_->m_vPntGroup.push_back(aGroup);  
 
		updateTreewidget();
		return;
	}
}
 
  
void MainWindow::ONOFF_Transparent(bool b)
{ 
	if (b) 
		lml_param::isTransparent= true; 
	else

		lml_param::isTransparent= false;

	if (lml_param::isTransparent)
	{
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	}
	else
	{
		glDisable(GL_BLEND);
	}

	mainCanvas_->updateGL();
}

void MainWindow::ONOFF_Light(bool b)
{
	if (b) 
		lml_param::lighting = true; 
	else
		lml_param::lighting = false; 

	mainCanvas_->updateGL();
} 
void MainWindow::ONOFF_wire(bool b)
{
	if (b) 
		lml_param::showPHWirframe = true;
	else 
		lml_param::showPHWirframe = false;
	mainCanvas_->updateGL();
}
  
 
void MainWindow::canvasShowCheckChange()
{
	if (sender() == checkBox_PC)  
	{  
		this->mainCanvas_->m_ptrPC->toShow = checkBox_PC->isChecked() ? true : false;   
	}   
	else if (sender() == checkBox_RANSAC)  
	{  
		lml_param::hideRegionRANSAC = checkBox_RANSAC->isChecked() ? false : true; 
	}  
	else if (sender()==checkBox_Polyhedron)
	{
		lml_param::hidePolyhedron = checkBox_Polyhedron->isChecked() ? false : true;
	}
	else if (sender() == checkBox_rdmColor)
	{
		lml_param::rdmColor = checkBox_rdmColor->isChecked() ? true : false;
	}
	mainCanvas_->updateGL();
}  


bool MainWindow::showDlg_ransacParam()
{
	if (!dlg_ransacParam)
	{
		dlg_ransacParam  = new CDlg_ransacParam(this);
	}
	//dlg_ransacParam->line_ransac_minimum->setText(QApplication::translate("DlgRANSAC", "80", 0, QApplication::UnicodeUTF8));

	//A modal dialog
	dlg_ransacParam->exec();
	if (dlg_ransacParam->result() == QDialog::Accepted)
		return true;
	else 
		return false; 
}
 
void MainWindow::showWidget_DumpOutput()
{
	if (!this->dockWidget_DumpOutput->isVisible())
	{
		this->dockWidget_DumpOutput->show();
	}
}
void MainWindow::showWidget_ItemInfor()
{
	if (!this->dockWidget_ItemInfor->isVisible())
	{
		this->dockWidget_ItemInfor->show();
	}
}
void MainWindow::showWidget_AlgorithmFitBox()
{
	if (!this->dockWidget_Algorithm0->isVisible())
	{
		this->dockWidget_Algorithm0->show();
	}
}
void MainWindow::showWidget_ViewItems()
{
	if (!this->dockWidget_showItems->isVisible())
	{
		this->dockWidget_showItems->show();
	}
}


LoggerClient* MainWindow::output(int v) {
	output(QString::number(v).toStdString());
	return this;
}

LoggerClient* MainWindow::output(double v) {
	output(QString::number(v).toStdString());
	return this;
}

LoggerClient* MainWindow::output(const std::string& msg) 
{ 
	plainTextEdit->moveCursor(QTextCursor::End);
	plainTextEdit->insertPlainText(QString::fromStdString(msg));
	std::cout << msg ;
	plainTextEdit->repaint();
	return this;
}  

  
void MainWindow::updataCanvas()
{
	this->mainCanvas_->updateGL();
} 
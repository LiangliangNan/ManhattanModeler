
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


#ifndef POINTS_2_SURFACE_H
#define POINTS_2_SURFACE_H

#include <QMainWindow>

#include "ui_main_window.h"  
#include <QtGui/QKeyEvent>

#include "CPointCloud.h" 
#include "fitBoxes.h"
#include "tool_name.h"
#include "ProgressBar.h"


#include "logger_client.h"
class QLabel;
class QTabWidget;
class PaintCanvas;
 
class QProgressBar;  
class CDlg_ransacParam;
class DlgFittingBox;
class simpleWidget; 
class CDlgAddPlane; 

class MainWindow : public QMainWindow, public Ui::ManhattanModelerClass, public LoggerClient, public ProgressClient
{
	Q_OBJECT

public:
	MainWindow(QWidget *parent = 0, Qt::WindowFlags flags = 0);
	~MainWindow();
	
	PaintCanvas* canvas() { return mainCanvas_; }

public slots:
	bool openModelFile();
	void cleanModel();
	bool saveViewingItems();// save current point cloud or mesh shown in the paint canvas. 
	bool load_PCGroup();	// load ransac formate point cloud group.
	bool export_Items();	// save item tree objects. 
	void openRecentFile();	// Unfinished function.

	void setFullScreen(bool b);
	bool snapshot();

	void check_all_Items();
	void uncheck_All_Items();
	void updateTreewidget();		// also see 'setComBoxDWItem','comBoxItemChanged'
	void comBoxItemChanged(QString str);//also see 'updateTreewidget','setComBoxDWItem'
	void editTxtChangeSlot();
	void testSmallFunctions();
	void updateStatusBar();

	void boxTableCurrentItemChanged(QTreeWidgetItem *current, QTreeWidgetItem *previous);
	void boxTableItemChanged(QTreeWidgetItem* it, int col);
	void boxTabSelectChanged();
	
	  
	// RANSAC
	void canvasShowCheckChange(); // check current canvas display items.
	void ransacAct();   
	void calculateScore();

	// Point cloud process 
	void conPCtoMW();  

	// show widget window
	void showWidget_DumpOutput();
	void showWidget_ItemInfor();
	void showWidget_AlgorithmFitBox(); 
	void showWidget_ViewItems();  
	  
	// display status.
	void ONOFF_Transparent(bool b);
	void ONOFF_Light(bool b); 
	void ONOFF_wire(bool b);
	
	void showDlgFittingBox();	// Dialog for fitting boxes. 
	bool showDlg_ransacParam();	// Dialog for RANSAC parameter.
	  
	LoggerClient* output(int v) ;
	LoggerClient* output(double v) ;
	LoggerClient* output(const std::string& msg) ; 

	void about();
	void aboutQt();

private: 
	bool openPly(QString & fileName);
	void setComBoxDWItem(DrawMode mode) ;	// also see 'updateTreewidget','comBoxItemChanged'
	
	void createTreeWidgetTable();
	void createMenus();  
	void createComboBox();
	void createStatusBar();
	void createActionsForFileMenu();
	void createActionsForToolBar();

	void readSettings();
	void writeSettings();
 
	void setCurrentOpenFile(const QString &fileName);
	void updateRecentFileActions();
	QString strippedName(const QString &fullFileName);

	void showStatusMessage(const QString &text, int timeout = 0); 
	  
	void closeEvent(QCloseEvent *event);
public:
	void updataCanvas();
	void showCoordinateUnderMouse( float p[3], bool found );
	void showPixelPositionUnderMouse(const QPoint& p);
	 

	void ransac();  

	void runFittingBox();
	void runAddPln(double px,double py, double pz, double nx, double ny, double nz);
   
	void notify_progress(int value); 
	 

private:
	PaintCanvas*	mainCanvas_;

	float			paramLambda;
	QMenu*			contextMenu_;
	QComboBox *		comboBoxDrawMode_;
	QComboBox *		comboBoxParamType; // design a "comobox" and "lineEdit" to realize easily select parameter
	QLineEdit *		lineEditTxt;


	QLabel *statusLabel,
		*spacerLabel,
		*coordinateUnderMouseLabel,
		*coordPixelUnderMouseLabel,
		*numVerticesLabel;

	QProgressBar*	progress_bar_;


	QStringList		recentFiles_;
	QString			curMeshFileName_;
	QString			curDataDirectory_;
 
	enum { MaxRecentFiles = 5 };
	QAction			*actionsRecentFile[MaxRecentFiles],
					*actionSeparator;
	QAction			*actionClean;	

	//////////////////////////////////////////////////////////////////////////

	//std::vector<QColor>  colors_;
	
	//CPointCloud			* m_ptrPntcloudRemaind;
	double m_bigTheta;

	bool hasRANSAC;
	bool hasMW;
	 

	unsigned int m_NumOfTreeItems;

	// dialog from: setting parameters.
	
	CDlg_ransacParam	* dlg_ransacParam;
	DlgFittingBox		* dlg_fittingbox; 
	CDlgAddPlane		* dlg_addPln;

	simpleWidget * dlg_pointSize;

	bool m_ChechAll;
	bool m_unChechAll; 
	bool m_isIntitial_tree; 
};

#endif // TESTQGLVIEWER_H

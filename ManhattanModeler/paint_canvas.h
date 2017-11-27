
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




#ifndef PAINTCANVAS_H
#define PAINTCANVAS_H

#include "../3rd_qglviewer/qglviewer.h"
#include "color.h"
#include "math_types.h"
#include "draw_model.h"

#include "CVertexGroup.h"
#include "fitBoxes.h" 

class MainWindow; 
class CPointCloud;
class cpointgroup;
class CCubeBoxes;  
class ToolManager3D;
class CMeshObj;  

enum CanvasType {
	CT_FRONT,  CT_BACK,
	CT_LEFT,   CT_RIGHT,
	CT_TOP,    CT_BOTTOM,
	CT_ROTATION_FREE
};

enum onKey{
	key_null,key_spce,
	key_ctrl,
	key_Alt,
	Key_Shift
};


class PaintCanvas : public QGLViewer
{
	Q_OBJECT

public:
	PaintCanvas(CanvasType type, const QGLFormat& format, QWidget *parent);
	~PaintCanvas();

	DrawMode         draw_mode_;

protected:
	virtual void draw();
	virtual void init();

	// Mouse events functions
	virtual void mousePressEvent(QMouseEvent *e);
	virtual void mouseMoveEvent(QMouseEvent *e);
	virtual void mouseReleaseEvent(QMouseEvent *e);

	virtual void wheelEvent(QWheelEvent * e);

	//// Keyboard events functions
	void keyPressEvent( QKeyEvent *e );
	void keyReleaseEvent( QKeyEvent *e );
	  
public slots:
	void fitScreen() ;
	void reset();
	void drawCornerAxis() const;

	void selectVertexByLasso(bool b);

	void sceneManipulation(bool b) ;
	void measure2PntDist( bool b );
	void setProjectionMode(bool b);
	  
	void cam_getView(double & cx, double & cy, double & cz, double & dx, double & dy, double & dz);
	void cam_setView(double cx, double cy, double cz, double dx, double dy, double dz);

public:
	MainWindow* mainWindow() const { return main_window_; }

	void forceUpdate();
	void setDrawMode(DrawMode dm_){draw_mode_ = dm_;}
	void resetVtxGroup(std::vector<CVertexGroup*> & vGroup);
	 
	void addPC ( CPointCloud * ptrPC); 
	void setMesh(CMeshObj* m);
	void setMesh(CPolyHedron* ph);
	void setBoxes(CCubeBoxes * fb)
	{
		draw_mode_=DM_CANDICATE_BOXES;
		m_fitBoxes=fb; 
		fitScreen();
	}
	void setGroup()
	{
		draw_mode_=DM_RANSAC_GROUPS; 
		fitScreen(); 
	}

	cgPoint2f projectionOf(const cgPoint3f& p);
	cgPoint3f unProjectionOf(double winx, double winy, double winz);

private:    
	void showCoordinateUnderPixel(const QPoint& pos) const;
protected:
	MainWindow*      main_window_;


private:
	CanvasType		 type_; 

	bool             scene_manipulation_;


public: 
	//-- main point cloud data  
	std::vector<CPointCloud	 *> m_vPC; 
	CPointCloud	 * m_ptrPC;   
	CCubeBoxes       *m_fitBoxes;
	CMeshObj		*m_meshObj;
	CPolyHedron		*m_polyhedron;
	std::vector<CVertexGroup*> m_vPntGroup; 
	std::vector<std::pair<CVector3D,CVector3D>> vecLines;
	 

	ToolManager3D*   tool_manager_; // used when selecting points.
	onKey			m_onKey;
};


#endif // PAINTCANVAS_H


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


#include "paint_canvas.h"
#include "main_window.h" 
#include "math_global.h"

#include <QFileDialog>
#include <QMouseEvent>
#include <QMessageBox>
#include <cassert>
#include <fstream>
#include <memory>

#include "CPointCloud.h" 
#include "fitBoxes.h" 
#include "Parameters.h"
#include "tool_manager3d.h"

#include "logger.h" 

using namespace std;
using namespace qglviewer;


#ifndef GL_MULTISAMPLE
#define GL_MULTISAMPLE 0x809D
#endif


PaintCanvas::PaintCanvas(CanvasType type, const QGLFormat& format, QWidget *parent)
: QGLViewer(format, parent)
, type_(type)   
, m_ptrPC(nil) 
, m_fitBoxes(nil)
, m_meshObj(nil)
, m_polyhedron(nil) 
{
	main_window_ = dynamic_cast<MainWindow*>(parent);
	draw_mode_ = DM_INITIAL_POINTS; 
	
	tool_manager_ = new ToolManager3D(this);
	m_onKey = key_null;
	//////////////////////////////////////////////////////////////////////////
	// Move camera according to viewer type (on X, Y or Z axis)
	switch (type_) 
	{
	case CT_ROTATION_FREE:	camera()->setPosition(Vec(1.0,  1.0, 1.0));	break;
	case CT_FRONT:			camera()->setPosition(Vec(0.0, -1.0, 0.0));	break;
	case CT_BACK:			camera()->setPosition(Vec(0.0,  1.0, 0.0));	break;
	case CT_LEFT:			camera()->setPosition(Vec(-1.0, 0.0, 0.0));	break;
	case CT_RIGHT:			camera()->setPosition(Vec(1.0, 0.0, 0.0));	break;
	case CT_TOP:			camera()->setPosition(Vec(0.0, 0.0,  1.0));	break;
	case CT_BOTTOM:			camera()->setPosition(Vec(0.0, 0.0, -1.0));	break;
	default:	break;
	}

	camera()->lookAt(sceneCenter());
	camera()->setType(Camera::PERSPECTIVE);
	camera()->showEntireScene();
}


PaintCanvas::~PaintCanvas() 
{ 
	 
}

void PaintCanvas::forceUpdate() {
	repaint(); 

	// This approach has significant drawbacks. For example, imagine you wanted to perform two such loops 
	// in parallel-calling one of them would effectively halt the other until the first one is finished 
	// (so you can't distribute computing power among different tasks). It also makes the application react
	// with delays to events. Furthermore the code is difficult to read and analyze, therefore this solution
	// is only suited for short and simple problems that are to be processed in a single thread, such as 
	// splash screens and the monitoring of short operations.
	QCoreApplication::processEvents();  

	main_window_->updateStatusBar();
}

void PaintCanvas::init()
{
	setStateFileName("");

	// Default value is 0.005, which is appropriate for most applications. 
	// A lower value will prevent clipping of very close objects at the 
	// expense of a worst Z precision.
	camera()->setZNearCoefficient(0.001f);

	// Default value is square root of 3.0 (so that a cube of size 
	// sceneRadius() is not clipped).
	camera()->setZClippingCoefficient(50.0f);

	camera()->setType(Camera::PERSPECTIVE);
	camera()->setViewDirection(qglviewer::Vec(-1.0, 0.0, -1.0));
	showEntireScene();

	camera()->frame()->setSpinningSensitivity(/*1.0f*/100.0f);
	setMouseTracking(true);

	//////////////////////////////////////////////////////////////////////////

	glEnable(GL_DEPTH_TEST);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);

	//////////////////////////////////////////////////////////////////////////

	QColor bkgrd_color = Qt::white;
	setBackgroundColor(bkgrd_color);

	//////////////////////////////////////////////////////////////////////////

	setMouseBinding(Qt::ShiftModifier | Qt::LeftButton, CAMERA, SCREEN_ROTATE);

	//////////////////////////////////////////////////////////////////////////

	// 	float pos[] = { 0.268889, 0.268889, 0.924877, 0.0f };
	// 	glLightfv(GL_LIGHT0, GL_POSITION, pos);

	// Setup light parameters
	glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE); /*GL_FALSE*/
	glLightModeli(GL_LIGHT_MODEL_LOCAL_VIEWER, GL_FALSE); // how specular reflection angles are computed

	// light0
	glEnable(GL_LIGHT0);		// Enable Light 0
	glEnable(GL_LIGHTING);
	glEnable(GL_NORMALIZE);

	//////////////////////////////////////////////////////////////////////////

	// specify the specular and shininess
	float shininess = 64.0f; 
	float specular[] = { 0.6f, 0.6f, 0.6f, 1.0f };
	glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, shininess);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, specular); 

	// make another side different
	float back_emission[] = { 0.0f, 0.0f, 0.5f, 0.0f };
	glMaterialfv(GL_BACK, GL_EMISSION, back_emission);

	// 	// Makes specular lighting work in texture mapping mode.
	// 	glLightModeli(GL_LIGHT_MODEL_COLOR_CONTROL, GL_SEPARATE_SPECULAR_COLOR);

	// to use facet color, the GL_COLOR_MATERIAL should be enabled
	glEnable(GL_COLOR_MATERIAL);
	// to use material color, the GL_COLOR_MATERIAL should be disabled
	//glDisable(GL_COLOR_MATERIAL);
}
 
void PaintCanvas::addPC( CPointCloud * ptrPC )
{
	if (ptrPC == NULL)
	{
		Logger::output("error: the PC loaded is not correct!\n");
		return;
	} 
	draw_mode_=DM_INITIAL_POINTS;  
	m_vPC.push_back(ptrPC); 
	this->m_ptrPC = ptrPC;
	this->m_ptrPC->m_bSelect = true;
	this->m_ptrPC->build_boundingBox();
	fitScreen(); 
} 

void PaintCanvas::setMesh(CMeshObj* m)
{
	if (!m)
	{
		Logger::output("error: the Mesh is not correct!\n");
		return;
	}
	if (!this->m_meshObj)
	{
		delete this->m_meshObj;
		this->m_meshObj = NULL;
	}
	draw_mode_=DM_MESH; 
	m_meshObj = m; 
	fitScreen();
}
void PaintCanvas::setMesh(CPolyHedron* ph)
{
	if (!ph)
	{
		Logger::output("error: the polyhedron has problems!\n");
		return;
	}
	if (this->m_polyhedron!=NULL)
	{
		delete this->m_polyhedron;
		this->m_polyhedron = NULL;
	}
	this->draw_mode_=DM_MESH; 
	this->m_polyhedron = ph; 
	fitScreen(); 

	if (!this->m_polyhedron)
	{
		Logger::output("no ph\n");
	}
}


void PaintCanvas::draw()
{
	if (lml_param::is_drawCornerAxis)
	{
		drawCornerAxis();
	} 

	glPointSize(lml_param::showPointSize);

	if (lml_param::lighting) 
		glEnable(GL_LIGHTING); 
	else
		glDisable(GL_LIGHTING);

	// shading option
	if(lml_param::smoothShading)
		glShadeModel(GL_SMOOTH);
	else
		glShadeModel(GL_FLAT); 


	// draw Point cloud.  
	if (this->m_vPC.size()>0 )
	{   
		glDisable(GL_MULTISAMPLE);

		for (std::vector<CPointCloud*>::iterator it = m_vPC.begin();it!=m_vPC.end();it++)
		{ 
			(*it)->drawPC(lml_param::showPointSize); 
		}

		// draw segmentation ....
		if (this->m_vPC[0]->m_vecLabels.size()>0 && lml_param::shift1)
		{
			this->m_vPC[0]->drawLabels();
		}

		if (this->vecLines.size()>0 && lml_param::shift2)
		{
			glEnable(GL_MULTISAMPLE);

			glLineWidth(8);
			glColor3d(0,0,1.);
			glBegin(GL_LINES);
			for (int li =0;li<vecLines.size(); li++) 
			{
				CVector3D & lineP1 = vecLines[li].first;
				CVector3D & lineP2 = vecLines[li].second;
				glVertex3d(lineP1.x_(),lineP1.y_(),lineP1.z_());
				glVertex3d(lineP2.x_(),lineP2.y_(),lineP2.z_());
			}
			glEnd();
		}
		// draw segmentation ....

	}  

	switch(draw_mode_)
	{
	case DM_MESH:
		if (this->m_meshObj) {
			glEnable(GL_MULTISAMPLE);
			this->m_meshObj->drawMesh();
		}

		if (this->m_polyhedron)
		{ 
			glEnable(GL_MULTISAMPLE);
			this->m_polyhedron->drawPolyhedron(0.9f);
		} 

		break;
	case DM_INITIAL_POINTS:   
		break;
	case DM_RANSAC_GROUPS: // show RANSAC result 
		if (m_vPntGroup.size()>0)
		{
			glDisable(GL_MULTISAMPLE);

			unsigned int nbGroup=m_vPntGroup.size(); 
			for (unsigned int i=0; i<nbGroup; i++)
			{
				m_vPntGroup[i]->drawGroup(2.0);
			}
		}
		break; 
	case DM_CANDICATE_BOXES:// fit box result 
		if (m_fitBoxes != NULL && m_fitBoxes->size()>0)
		{
			glEnable(GL_MULTISAMPLE);
			m_fitBoxes->drawFitBoxes(DM_CANDICATE_BOXES);
			m_fitBoxes->drawViewScore();			
		}
		break;
	case DM_OPTIMIZE_BOXES: // show final  boxes result after optimization
		if (m_fitBoxes != NULL && m_fitBoxes->size()>0)
		{
			glEnable(GL_MULTISAMPLE);
			m_fitBoxes->drawFitBoxes(DM_OPTIMIZE_BOXES); 
		}
		break;
	
	case DM_FOOT_GRID: 
		break;
	} 
  
	 

	//--------	draw while using tool  ------------
	if (tool_manager_->current_tool_name() != Tl::TOOL_EMPTY)
		tool_manager_->current_tool()->draw();
	//--------	draw while using tool  ------------

}


void PaintCanvas::fitScreen() 
{ 
	Vec vmin, vmax;

	switch(draw_mode_)
	{
	case DM_MESH:
		if (m_meshObj)	
		{
			cgBbox3f box = m_meshObj->bbox();
			vmin = Vec (box.xmin(), box.ymin(), box.zmin());
			vmax = Vec (box.xmax(), box.ymax(), box.zmax());  
		} 
		if (m_polyhedron)
		{
			CVector3D & maxP = m_polyhedron->m_bbox->m_vMax;
			CVector3D & minP = m_polyhedron->m_bbox->m_vMin; 

			vmin = Vec (minP.x_(),minP.y_(),minP.z_());
			vmax = Vec (maxP.x_(),maxP.y_(),maxP.z_()); 
		}
		break; 
	case DM_RANSAC_GROUPS: 
	case DM_INITIAL_POINTS:
		if (m_vPC.size()>0)	
		{
			vmin = Vec (10000000,10000000,10000000);
			vmax = Vec (-10000000,-10000000,-10000000); 
			for (std::vector<CPointCloud*>::iterator it = m_vPC.begin();it!=m_vPC.end();it++)
			{ 
				CVector3D & maxP = (*it)->m_cBoundingBox->m_vMin; 
				CVector3D & minP = (*it)->m_cBoundingBox->m_vMax;
				vmin.x = vmin.x>minP.x_()? minP.x_():vmin.x;
				vmin.y = vmin.y>minP.y_()? minP.y_():vmin.y;
				vmin.z = vmin.z>minP.z_()? minP.z_():vmin.z; 
				vmax.x = vmax.x<maxP.x_()? maxP.x_():vmax.x;
				vmax.y = vmax.y<maxP.y_()? maxP.y_():vmax.y;
				vmax.z = vmax.z<maxP.z_()? maxP.z_():vmax.z;
			}
		} 
	case DM_CANDICATE_BOXES: 
		if (m_fitBoxes)
		{
			CVector3D & maxP = m_fitBoxes->m_bbox.m_vMin; 
			CVector3D & minP = m_fitBoxes->m_bbox.m_vMax; 
			vmin = Vec (minP.x_(),minP.y_(),minP.z_());
			vmax = Vec (maxP.x_(),maxP.y_(),maxP.z_());
		}
		break; 
	}  
	setSceneBoundingBox(vmin, vmax); 
	showEntireScene();
	//camera()->lookAt(sceneCenter());
} 

void PaintCanvas::reset()
{  
	m_ptrPC = NULL;
	if (m_vPC.size()>0)
	{
		//for (int i=0;i<m_vPC.size();i++)
		//{
		//	if (m_vPC[i])
		//	{
		//		delete m_vPC[i];
		//		m_vPC[i]= NULL;
		//	}
		//}
		m_vPC.clear();
	}
	if (m_fitBoxes!=NULL)
	{
		delete m_fitBoxes;
		m_fitBoxes = nil;
	}
	this->m_vPntGroup.clear(); 
	if (this->m_polyhedron!=NULL)
	{
		delete this->m_polyhedron;
		this->m_polyhedron = NULL;
	}
	
	Logger::output("accomplish resetting.\n\n"); 
}

void PaintCanvas::drawCornerAxis() const 
{
	int viewport[4];
	int scissor[4];

	GLboolean on = glIsEnabled(GL_LIGHTING);
	if (on)
		glDisable(GL_LIGHTING);

	// The viewport and the scissor are changed to fit the lower left
	// corner. Original values are saved.
	glGetIntegerv(GL_VIEWPORT, viewport);
	glGetIntegerv(GL_SCISSOR_BOX, scissor);

	int coord_system_region_size_ = 150;
	// Axis viewport size, in pixels
	glViewport(0, 0, coord_system_region_size_, coord_system_region_size_);
	glScissor(0, 0, coord_system_region_size_, coord_system_region_size_);

	// The Z-buffer is cleared to make the axis appear over the
	// original image.
	glClear(GL_DEPTH_BUFFER_BIT);

	// Tune for best line rendering
	glLineWidth(3.0);

	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	glOrtho(-1, 1, -1, 1, -1, 1);

	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();
	glMultMatrixd(camera()->orientation().inverse().matrix());

	float axis_size = 0.9f; // other 0.2 space for drawing the x, y, z labels
	drawAxis(axis_size); 

	// Draw text id
	glColor3f(0, 0, 0);

	// It seems the renderText() func will disable multi-sample.
	// This is a bug in Qt ?
	GLboolean anti_alias = glIsEnabled(GL_MULTISAMPLE);
	const_cast<PaintCanvas*>(this)->renderText(axis_size, 0, 0, "X");
	const_cast<PaintCanvas*>(this)->renderText(0, axis_size, 0, "Y");
	const_cast<PaintCanvas*>(this)->renderText(0, 0, axis_size, "Z");
	if (anti_alias)
		glEnable(GL_MULTISAMPLE);

	glMatrixMode(GL_PROJECTION);
	glPopMatrix();

	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();

	// The viewport and the scissor are restored.
	glScissor(scissor[0], scissor[1], scissor[2], scissor[3]);
	glViewport(viewport[0], viewport[1], viewport[2], viewport[3]);

	if (on)
		glEnable(GL_LIGHTING);
}
  

void PaintCanvas::resetVtxGroup( std::vector<CVertexGroup*> & vGroup )
{
	if (this->m_vPntGroup.size()>0)
	{
		this->m_vPntGroup.clear();
	}

	this->m_vPntGroup.insert(this->m_vPntGroup.end(), vGroup.begin(),vGroup.end());
	draw_mode_=DM_RANSAC_GROUPS; 
	fitScreen();
}

cgPoint2f PaintCanvas::projectionOf(const cgPoint3f& p) 
{    // point to screen
	Vec v = camera()->projectedCoordinatesOf(Vec(p.x(), p.y(), p.z()));
	return cgPoint2f(v.x, v.y);
}


cgPoint3f PaintCanvas::unProjectionOf(double winx, double winy, double winz) 
{  // screen to point	
	Vec v = camera()->unprojectedCoordinatesOf(Vec(winx, winy, winz));
	return cgPoint3f(v.x, v.y, v.z);
}
  
void PaintCanvas::showCoordinateUnderPixel(const QPoint& pos) const 
{
	bool found = false;
	Vec q = camera()->pointUnderPixel(pos, found);
	float coord[3]={q.x, q.y, q.z};
	mainWindow()->showCoordinateUnderMouse(coord, found);
	mainWindow()->showPixelPositionUnderMouse(pos);
}

void PaintCanvas::mousePressEvent(QMouseEvent* e)
{
	if (tool_manager_->current_tool_name() == Tl::TOOL_EMPTY)
		QGLViewer::mousePressEvent(e);
	else 
	{
		tool_manager_->current_tool()->press(e);
		updateGL();
	}  
}
void PaintCanvas::mouseMoveEvent(QMouseEvent* e)
{
	if (e->button() == Qt::MidButton) 
	{
	} 
	else 
	{
		if (tool_manager_->current_tool_name() == Tl::TOOL_EMPTY)
			QGLViewer::mouseMoveEvent(e); 
		else 
		{
			tool_manager_->current_tool()->move(e);
			updateGL();
		}

		showCoordinateUnderPixel(e->pos());
	}
}

void PaintCanvas::mouseReleaseEvent(QMouseEvent* e)
{
	if (tool_manager_->current_tool_name() == Tl::TOOL_EMPTY)
		QGLViewer::mouseReleaseEvent(e); 
	else 
	{
		tool_manager_->current_tool()->release(e);
		updateGL();
	}
}


void PaintCanvas::wheelEvent( QWheelEvent * e )
{
	if (this->m_onKey == key_Alt)
	{ 
		return;
	} 
	if (tool_manager_->current_tool_name()==Tl::TOOL_EMPTY)
	{
		QGLViewer::wheelEvent(e);
	}
}

void PaintCanvas::keyPressEvent( QKeyEvent *e )
{
	switch (e->key())
	{
	case Qt::Key_Control:
		this->m_onKey = key_ctrl;
		break;
	case  Qt::Key_Shift:
		this->m_onKey = Key_Shift;
		break;
	case  Qt::Key_Alt:
		this->m_onKey = key_Alt;
		break; 
	case  Qt::Key_Space:
		this->m_onKey = key_spce;
		break; 
	//case Qt::Key_Escape:
	//	break;

	default:
		QGLViewer::keyPressEvent(e);
	}
}

void PaintCanvas::keyReleaseEvent( QKeyEvent *e )
{ 
	if (e->key() == Qt::Key_Control)
		update(); 
	//if (e->isAutoRepeat())
	//{
	//	e->ignore();
	//}
	switch (e->key())
	{
	case Qt::Key_Control:
		this->m_onKey = key_ctrl;
		break;
	case  Qt::Key_Shift:
		this->m_onKey = Key_Shift;
		break;
	case  Qt::Key_Alt:
		this->m_onKey = key_Alt;
		break; 
	case  Qt::Key_Space:
		this->m_onKey = key_spce;
		break; 
		//case Qt::Key_Escape:
		//	break;

	default:
		QGLViewer::keyPressEvent(e);
	}
}
void PaintCanvas::setProjectionMode(bool b) 
{
	if (b) {
		if (camera()->type() != Camera::PERSPECTIVE)
			camera()->setType(Camera::PERSPECTIVE);
	} else {	
		if (camera()->type() != Camera::ORTHOGRAPHIC)
			camera()->setType(Camera::ORTHOGRAPHIC);
	}

	updateGL();
}
void PaintCanvas::selectVertexByLasso( bool b )
{ 
	if (m_vPC.size()<0 || b == false)
		return;

	tool_manager_->set_tool(Tl::TOOL_SELECT_VERTEX_BY_LASSO);
	tool_manager_->set_select_method(Tl::SM_LASSO); 
	setCursor(Qt::PointingHandCursor);
	update();
}
void PaintCanvas::sceneManipulation( bool b )
{
	scene_manipulation_ = b; 
	if (b)
	{
		tool_manager_->set_tool(Tl::TOOL_EMPTY); 
		setCursor(Qt::ArrowCursor); 
	}

	update();
}

void PaintCanvas::measure2PntDist( bool b )
{
	if (m_vPC.size()<0 || b == false)
		return;
	if (b)
	{
		tool_manager_->set_tool(Tl::TOOL_MEASURE2PNT_DIST); 
		setCursor(Qt::ArrowCursor); 
	}

	update();
}

void PaintCanvas::cam_getView( double & cx, double & cy, double & cz, double & dx, double & dy, double & dz )
{ 
	Vec camPos = camera()->position();
	cx = camPos.x;
	cy = camPos.y;
	cz = camPos.z;

	Vec camDir = camera()->viewDirection();
	dx = camDir.x;
	dy = camDir.y;
	dz = camDir.z;
	this->updateGL();
}

void PaintCanvas::cam_setView( double cx, double cy, double cz, double dx, double dy, double dz )
{
	camera()->setPosition(Vec(cx,cy,cz));
	camera()->setViewDirection(Vec(dx,dy,dz));
	//camera()->lookAt(sceneCenter());
	//showEntireScene();
	this->updateGL();
}
  
 
#ifndef _TOOL3D_H_
#define _TOOL3D_H_
 
#include "counted.h"
#include "math_types.h"

#include <QPoint>
 
class Object ;
class PaintCanvas;
class QMouseEvent;
class VertexGroup;

namespace Tl {


	class Tool3D: public Counted 
	{
	public:
		Tool3D(PaintCanvas* canvas) ;

		virtual ~Tool3D() {}

	public:
		virtual void press(QMouseEvent *e) = 0;
		virtual void release(QMouseEvent *e) = 0;
		virtual void move(QMouseEvent *e) = 0;
		virtual void reset() = 0;

		virtual void draw() const = 0;

		virtual void status(const std::string& value);

	public:
		virtual PaintCanvas* canvas() const;
		virtual void set_canvas(PaintCanvas* canvas);

		cgLine3f	get_mouse_line_in3DSpace(const QPoint& mouse) const ; 
		cgSegment3f get_mouse_segment_in3DSpace(const QPoint& mouse) const;
		// NOTE: pointing to the outside of the screen
		cgVector3f	get_view_direction_in_3d_space(const QPoint& mouse) const ;

		cgPoint3f get_intersect_point(const cgPlane3f& plane, const QPoint& mouse, bool& success) const;
		// check whether a point is selected on the mouse pos. zDist reflects (!=) the depth to the device-window. 
		bool pnt_selected(const cgPoint3f& pnt, const QPoint& mouse, const float threshSqr, float & zDist) const;
		
	protected:
		PaintCanvas*  canvas_ ;

		bool  left_button_down_;
		bool  right_button_down_;
		bool  middle_button_down_;
	} ;

	//________________________________________________________________

	class EmptyTool3D : public Tool3D 
	{
	public:
		EmptyTool3D(PaintCanvas* canvas) ;
		virtual ~EmptyTool3D() {}

	public:
		virtual void press(QMouseEvent *) {}
		virtual void release(QMouseEvent *) {}
		virtual void move(QMouseEvent *) {}
		virtual void reset() {}

		virtual void draw() const {}
	} ;

}


typedef SmartPointer<Tl::Tool3D> Tool3D_var ;


#endif


#include "tool3d.h"
#include "paint_canvas.h" 
 

#include <QMouseEvent>


namespace Tl {

	Tool3D::Tool3D(PaintCanvas* canvas) 
		: canvas_(canvas)
		, left_button_down_(false)
		, right_button_down_(false)
		, middle_button_down_(false)
	{
	}

	PaintCanvas* Tool3D::canvas() const {
		return canvas_ ;
	}

	void Tool3D::set_canvas(PaintCanvas* canvas) {
		canvas_ = canvas ;
	}

	void Tool3D::status(const std::string& value) {
		std::cout << value << "TODO: prompt user in status bar" << std::endl;
		// status_message() ;
	}

	cgLine3f Tool3D::get_mouse_line_in3DSpace(const QPoint& mouse) const 
	{
		QPoint pos = mouse;
		cgPoint3f p_near = canvas()->unProjectionOf(pos.x(), pos.y(), 0.0f);
		cgPoint3f p_far  = canvas()->unProjectionOf(pos.x(), pos.y(), 1.0f);

		return cgLine3f(p_near, p_far);
	}
	cgSegment3f Tool3D::get_mouse_segment_in3DSpace(const QPoint& mouse) const 
	{
		QPoint pos = mouse;
		cgPoint3f p_near = canvas()->unProjectionOf(pos.x(), pos.y(), 0.0f);
		cgPoint3f p_far  = canvas()->unProjectionOf(pos.x(), pos.y(), 1.0f);

		return cgSegment3f(p_near, p_far);
	}

	cgVector3f Tool3D::get_view_direction_in_3d_space(const QPoint& mouse) const 
	{
		cgLine3f line = get_mouse_line_in3DSpace(mouse);
		cgVector3f dir = -line.to_vector();
		float len = std::sqrt(dir.squared_length());
		if (len != 0.0f)
			dir = dir / len; 
		return dir;
	}

	cgPoint3f Tool3D::get_intersect_point(const cgPlane3f& plane, const QPoint& mouse, bool& success) const
	{
		cgLine3f line = get_mouse_line_in3DSpace(mouse);

		CGAL::Object obj = CGAL::intersection(line, plane);
		if (const cgPoint3f * point = CGAL::object_cast<cgPoint3f>(&obj)) {
			success = true;
			return *point;
		} else {
			success = false;
			return cgPoint3f(0.0f, 0.0f, 0.0f);
		}
	}


	bool Tool3D::pnt_selected(const cgPoint3f& pnt, const QPoint& mouse, const float threshSqr, float & zDist) const
	{
		zDist = 1e6;
		QPoint pos = mouse;
		cgPoint3f p_near = canvas()->unProjectionOf(pos.x(), pos.y(), 0.0f);
		cgPoint3f p_far  = canvas()->unProjectionOf(pos.x(), pos.y(), 1.0f);
		cgSegment3f line = cgSegment3f(p_near, p_far);
		FT distSqr = CGAL::squared_distance(pnt,line);
		if (distSqr>threshSqr) 
			return false; 
		else
		{
			cgVector3f v1 = pnt	-	p_near;
			cgVector3f v2 = p_far - p_near;
			zDist = v1*v2; 
			return true;
		}
	}

 

	//////////////////////////////////////////////////////////////////////////	
	

	EmptyTool3D::EmptyTool3D(PaintCanvas* canvas) 
		: Tool3D(canvas)
	{
	}
 

}
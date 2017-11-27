#include "select_tool.h"
#include "paint_canvas.h"
#include <QMouseEvent>




namespace Tl {

	SelectTool::SelectTool(PaintCanvas* canvas) 
		: Tool3D(canvas)
		, select_mode_(SM_ADD)
		, select_method_(SM_HIT)
		, select_button_(SBT_LEFT)
		, select_buffer_(nil)
		, hit_resolution_(5)
		, multi_select_(false)
		, select_region_width_(hit_resolution_)
		, select_region_height_(hit_resolution_)
	{
	}

	SelectTool::~SelectTool() {
		delete [] select_buffer_;
	}

	void SelectTool::initialize_select_buffer() {
		size_t size = needed_buffer_size();
		set_select_buffer_size(size);
	}

	void SelectTool::select(const QPoint& point) {
		if (!multi_select_)
			unselect_all();
		selected_object_indices_.clear();

		begin_selection(point);
		draw_with_names();
		end_selection(point);
	}

	void SelectTool::highlight(const QPoint& point) {	
		unhighlight_all();
		highlighted_object_indices_.clear();

		begin_selection(point);
		draw_with_names();
		end_highlight(point);
	}

	void SelectTool::begin_selection(const QPoint& point) {
		// Make OpenGL context current (may be needed with several viewers ?)
		canvas()->makeCurrent();
		initialize_select_buffer();

		if (select_buffer_size_ <= 0)
			return;

		// Prepare the selection mode
		glSelectBuffer(select_buffer_size_, select_buffer_);
		glRenderMode(GL_SELECT);
		glInitNames();

		// Loads the matrices
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		static GLint viewport[4];
		canvas()->camera()->getViewport(viewport);
		gluPickMatrix(point.x(), point.y(), select_region_width_, select_region_height_, viewport);

		// loadProjectionMatrix() first resets the GL_PROJECTION matrix with a glLoadIdentity().
		// The false parameter prevents this and hence multiplies the matrices.
		canvas()->camera()->loadProjectionMatrix(false);
		// Reset the original (world coordinates) modelview matrix
		canvas()->camera()->loadModelViewMatrix();
	}


	void SelectTool::end_selection(const QPoint& point) {
		// Flush GL buffers
		glFlush();

		// Get the number of objects that were seen through the pick matrix frustum. Reset GL_RENDER mode.
		GLint nbHits = glRenderMode(GL_RENDER);
		if (nbHits > 0)	{
			// Interpret results : each object created 4 values in the selectBuffer().
			// (selectBuffer())[4*i+3] is the id pushed on the stack.
			for (int i=0; i<nbHits; ++i)
				selected_object_indices_.push_back(select_buffer_[4*i+3]);
		}

		canvas()->makeCurrent();
		make_selection(select_mode_ == SM_ADD);
	}


	void SelectTool::begin_highlight(const QPoint &point) {
		begin_selection(point);
	}


	void SelectTool::end_highlight(const QPoint& point) {
		// Flush GL buffers
		glFlush();

		// Get the number of objects that were seen through the pick matrix frustum. Reset GL_RENDER mode.
		GLint nbHits = glRenderMode(GL_RENDER);
		if (nbHits > 0)	{
			// Interpret results : each object created 4 values in the selectBuffer().
			// (selectBuffer())[4*i+3] is the id pushed on the stack.
			for (int i=0; i<nbHits; ++i)
				highlighted_object_indices_.push_back(select_buffer_[4*i+3]);
		}
		canvas()->makeCurrent();
		make_highlight();
	}


	const std::vector<int>&  SelectTool::get_selected_object_indices() const {
		return selected_object_indices_;
	}

	const std::vector<int>&  SelectTool::get_highlighted_object_indices() const {
		return highlighted_object_indices_;
	}

	void SelectTool::set_select_buffer_size(size_t size) {
		if (select_buffer_) {
			delete[] select_buffer_;
			select_buffer_ = nil;
		}

		select_buffer_size_ = size;
		select_buffer_ = new unsigned int[select_buffer_size_];
		//select_buffer_.resize(size);
	}

	void SelectTool::press(QMouseEvent *e) {
		if (e->button() == Qt::LeftButton)
			left_button_down_ = true;
		if (e->button() == Qt::RightButton)
			right_button_down_ = true;

		if (e->modifiers() == Qt::ControlModifier)
			multi_select_ = true;
		else
			multi_select_ = false;

		if (select_button_ == SBT_LEFT)	{
			if (!left_button_down_) { 
				if (right_button_down_) 
					select_mode_ = SM_REMOVE;
				else { 
					select_mode_ = SM_NONE;
					return;
				}
			} 
		} else if (select_button_ == SBT_RIGHT) {
			if (!right_button_down_) {
				if (left_button_down_)
					select_mode_ = SM_REMOVE;
				else {
					select_mode_ = SM_NONE;
					return;
				}
			}
		} else if (select_button_ == SBT_BOTH) {
			if ((!left_button_down_) && (!right_button_down_)) {
				select_mode_ = SM_NONE;
				return;
			}
		}

		mouse_pressed_pos_ = e->pos();

		//////////////////////////////////////////////////////////////////////////

		if (e->modifiers() != Qt::AltModifier)
			select_mode_ = SM_ADD;
		else 
			select_mode_ = SM_REMOVE;	

		switch (select_method_)
		{
		case SM_HIT:
			select(e->pos());
			break;
		case SM_RECTANGLE:
			rectangle_ = QRect(e->pos(), e->pos());
			break;
		default:
			break;
		}
	}

	void SelectTool::move(QMouseEvent *e) {
		mouse_moving_pos_ = e->pos();

		if (select_method_ == SM_HIT) {
			highlight(e->pos());
		} 

		if (left_button_down_ || right_button_down_) {
			drag(e);
		}
	}	

	void SelectTool::release(QMouseEvent *e) {
		if (e->button() == Qt::LeftButton)
			left_button_down_ = false;
		if (e->button() == Qt::RightButton)
			right_button_down_ = false;	

		if (select_method_ == SM_RECTANGLE) {
			// Actual selection on the rectangular area.
			// Possibly swap left/right and top/bottom to make rectangle_ valid.
			rectangle_ = rectangle_.normalized();

			// Define selection window dimensions
			select_region_width_ = rectangle_.width();
			select_region_height_= rectangle_.height();

			// Compute rectangle center and perform selection
			QPoint pos = rectangle_.center(); 

			select(pos);

			// reset
			select_region_width_ = hit_resolution_;
			select_region_height_= hit_resolution_;
		}

		rectangle_ = QRect(e->pos(), e->pos());	
	}

	void SelectTool::drag(QMouseEvent *e) {
		if (select_method_ == SM_RECTANGLE) {
			rectangle_.setBottomRight(e->pos());
			canvas()->updateGL();
		}
	}

	void SelectTool::draw() const {
		if (select_method_ == SM_RECTANGLE)
			if (left_button_down_ || right_button_down_) {
				draw_selection_rectangle();
			}			
	}

	void SelectTool::draw_selection_rectangle() const {
		canvas()->startScreenCoordinatesSystem();

		GLboolean on = glIsEnabled(GL_LIGHTING);
		if (on) 
			glDisable(GL_LIGHTING);

		glLineWidth(2.0);
		glColor4f(0.0f, 1.0f, 1.0f, 0.5f);
		glBegin(GL_LINE_LOOP);
		glVertex2i(rectangle_.left(),  rectangle_.top());
		glVertex2i(rectangle_.right(), rectangle_.top());
		glVertex2i(rectangle_.right(), rectangle_.bottom());
		glVertex2i(rectangle_.left(),  rectangle_.bottom());
		glEnd();	

		glEnable(GL_BLEND);
		glDepthMask(GL_FALSE);
		glColor4f(0.0, 0.0, 0.4f, 0.3f);
		glBegin(GL_QUADS);
		glVertex2i(rectangle_.left(),  rectangle_.top());
		glVertex2i(rectangle_.right(), rectangle_.top());
		glVertex2i(rectangle_.right(), rectangle_.bottom());
		glVertex2i(rectangle_.left(),  rectangle_.bottom());
		glEnd();
		glDisable(GL_BLEND);
		glDepthMask(GL_TRUE);

		if (on)
			glEnable(GL_LIGHTING);

		canvas()->stopScreenCoordinatesSystem();
	}

}


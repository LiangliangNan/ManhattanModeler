#ifndef _SELECT_TOOL_H_
#define _SELECT_TOOL_H_

#include "tool3d.h"

#include <vector>
#include <QRect>
 


namespace Tl {

	// Different selection modes
	enum SelectMode {
		SM_ADD, 
		SM_REMOVE,
		SM_NONE
	};


	enum SelectMethod {
		SM_HIT,
		SM_RECTANGLE,
		SM_LASSO
	};

	enum SelectButton {
		SBT_LEFT,
		SBT_RIGHT,
		SBT_BOTH
	};


	class SelectTool : public Tool3D 
	{
	public:
		SelectTool(PaintCanvas* canvas) ;
		virtual ~SelectTool() ;

		virtual void press(QMouseEvent *e) ;
		virtual void move(QMouseEvent *e) ;
		virtual void release(QMouseEvent *e) ;

		virtual void draw() const ;

		const std::vector<int>&  get_selected_object_indices() const ;
		const std::vector<int>&  get_highlighted_object_indices() const ;

		SelectMethod select_method() const                  { return select_method_;   }
		void         set_select_method(SelectMethod method) { select_method_ = method; }

		SelectButton select_button() const { return select_button_; }
		void         set_select_button(SelectButton bt) { select_button_ = bt; }

		// NOTE: very important, you must call this function in you constructor
		void		 initialize_select_buffer() ;  // set the buffer size

	protected:	
		void		 set_select_buffer_size(size_t size);

		virtual void draw_with_names() const = 0;
		virtual size_t  needed_buffer_size() const = 0;

		virtual void unselect_all() = 0;
		virtual void unhighlight_all() = 0;

		void		 select(const QPoint& point);
		void		 highlight(const QPoint& point);

		void		 begin_selection(const QPoint& point);
		void		 end_selection(const QPoint& point);
		virtual void make_selection(bool b = true) = 0;  

		void		 begin_highlight(const QPoint& point);
		void		 end_highlight(const QPoint& point);
		virtual void make_highlight() = 0;

		virtual void drag(QMouseEvent *e) ;
		virtual void reset() {}


	protected:
		void draw_selection_rectangle() const ; 

	protected:
		int             hit_resolution_;     // in pixels
		int				select_region_width_;
		int				select_region_height_;
		size_t			select_buffer_size_;
		//std::vector<unsigned int> select_buffer_;
		unsigned int*   select_buffer_;

		bool			multi_select_;

		SelectMode		select_mode_;
		SelectMethod	select_method_;
		SelectButton	select_button_;

		QPoint			mouse_pressed_pos_;
		QPoint          mouse_moving_pos_;

		// Current rectangular selection
		QRect			rectangle_;

		std::vector<int>  selected_object_indices_;   
		std::vector<int>  highlighted_object_indices_;
	} ;
}



#endif


#ifndef _TOOL_MANAGER3D_H_
#define _TOOL_MANAGER3D_H_


#include "basic_types.h"
#include "select_tool.h" 
#include "tool_name.h"

#include <map>

/* usage:

declaration: 
ToolManager3D toolManager_(rendering_context());

// mouse events:
toolManager_.current_tool()->press(rp);
toolManager_.current_tool()->release(rp);

// when surface changes:
toolManager_.set_surface(surface_);

// when job changes:
toolManager_.set_current_tool(ToolManager3D::TOOL_EDIT_ANCHORS);
*/


class ToolManager3D
{
public:
	ToolManager3D(PaintCanvas* canvas) ;
	virtual ~ToolManager3D() ;

	static std::string title() { return "[ToolManager3D]: "; }

	Tl::Tool3D* current_tool() { return current_tool_ ; }
	Tl::ToolName current_tool_name() const { return current_tool_name_; }

	void set_tool(Tl::ToolName name) ;

	void status(const std::string& value) ;

	virtual PaintCanvas* canvas() const;
	virtual void set_canvas(PaintCanvas* canvas);

	// for tools with select functions
	void set_select_method(Tl::SelectMethod method) ;
	void set_select_button(Tl::SelectButton button) ;

	// prepare a tool specified by name
	Tl::Tool3D* prepare_tool(Tl::ToolName name);

	void clear() ;

	void reset();

protected:
	virtual Tl::Tool3D* create_new_tool(Tl::ToolName name);

protected:
	PaintCanvas*  canvas_;

	Tl::Tool3D*	  current_tool_ ;
	Tl::ToolName  current_tool_name_;

	typedef std::map<Tl::ToolName, Tool3D_var> ToolMap ;
	ToolMap tools_ ;
} ;


#endif

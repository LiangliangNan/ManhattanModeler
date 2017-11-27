
#include "tool_manager3d.h"  
#include "../3rd_qglviewer/qglviewer.h"
#include "CPointCloud.h"  

using namespace Tl;

ToolManager3D::ToolManager3D(PaintCanvas* canvas)
: canvas_(canvas)
{
	set_tool(TOOL_EMPTY);
}

ToolManager3D::~ToolManager3D() 
{
}


void ToolManager3D::set_tool(ToolName name) 
{
	ToolMap::iterator it = tools_.find(name) ;
	if(it == tools_.end()) 
	{
		Tool3D* tool = create_new_tool(name);
		if (tool != NULL) 
		{
			tools_[name] = tool;
		}
	}

	it = tools_.find(name);
	if(it == tools_.end()) 
	{ 
		current_tool_ = nil ;
		current_tool_name_ = TOOL_EMPTY;
		return ;
	}
	current_tool_ = it->second ;
	current_tool_name_ = name;
}


Tl::Tool3D* ToolManager3D::prepare_tool(Tl::ToolName name) 
{
	ToolMap::iterator it = tools_.find(name) ;
	if(it == tools_.end()) 
	{
		Tool3D* tool = create_new_tool(name);
		if (tool != nil) 
		{
			tools_[name] = tool;
			return tool;
		}
	}

	it = tools_.find(name) ;
	if(it == tools_.end())
	{ 
		current_tool_ = nil ;
		current_tool_name_ = TOOL_EMPTY;
		return nil;
	}
	return it->second ;
}


void ToolManager3D::status(const std::string& value) 
{
	std::cout << value << "TODO: prompt user in status bar" << std::endl;
	// status_message() ;
}

PaintCanvas* ToolManager3D::canvas() const 
{
	return canvas_ ;
}


void ToolManager3D::clear() 
{
	tools_.clear();
	current_tool_ = NULL;

	current_tool_name_ = TOOL_EMPTY;
}

void ToolManager3D::set_canvas(PaintCanvas* canvas) 
{
	canvas_ = canvas ;
}

Tool3D* ToolManager3D::create_new_tool(Tl::ToolName name) 
{
	Tool3D* tool = nil;
	switch (name)
	{
	case TOOL_EMPTY:
		tool = new EmptyTool3D(canvas_);
		break;  
  
	default: 
		break;
	}

	return tool;
}


void ToolManager3D::set_select_method(Tl::SelectMethod method) 
{
	SelectTool* tool = dynamic_cast<SelectTool*>(current_tool_);
	if (tool)
		tool->set_select_method(method);
}

void ToolManager3D::set_select_button(Tl::SelectButton button) 
{
	SelectTool* tool = dynamic_cast<SelectTool*>(current_tool_);
	if (tool)
		tool->set_select_button(button);
}



void ToolManager3D::reset() 
{
	ToolMap::iterator itr = tools_.begin();
	for (; itr!=tools_.end(); ++itr) 
	{
		itr->second.get()->reset();
	}

}
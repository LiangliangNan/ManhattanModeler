#include "PlyLoad.h"
#include "CPointCloud.h"
#include "ProgressBar.h"

CPlyLoad::CPlyLoad(void)
{
}

CPlyLoad::~CPlyLoad(void)
{
}

CPlyLoad* CPlyLoad::plyload(p_ply_argument argument) {
	CPlyLoad* result = nil ;
	ply_get_argument_user_data(argument, (void**)(&result), nil) ;
	assert(result != nil) ;
	return result ;
}
int CPlyLoad::color_cb(p_ply_argument argument) 
{
	return plyload(argument)->get_color_data(argument) ;
}
int CPlyLoad::vertex_cb(p_ply_argument argument) 
{
	return plyload(argument)->get_vertex_data(argument) ;
}
int CPlyLoad::normal_cb(p_ply_argument argument) 
{
	return plyload(argument)->get_normal_data(argument) ;
} 

int CPlyLoad::face_cb(p_ply_argument argument) 
{
	return plyload(argument)->get_face_data(argument) ;
}

int CPlyLoad::get_face_data( p_ply_argument argument )
{ 
	long length, value_index;
	ply_get_argument_property(argument, NULL, &length, &value_index);
	switch (value_index) {
		case 0: 
			idxx.resize(3);
			idxx[0] = (int)ply_get_argument_value(argument);
			
		case 1: 
			idxx[1] = (int)ply_get_argument_value(argument);
			 
			break;
		case 2:
			idxx[2] = (int)ply_get_argument_value(argument);
 
			faceIdx.push_back(idxx);
			break;
		default: 
			break;
	}
	return 1;
} 

int CPlyLoad::get_color_data(p_ply_argument argument) {
	long coord ;
	ply_get_argument_user_data(argument, nil, &coord);
	assert(coord >= 0 && coord < 3) ;
	rgb_[coord] = double(ply_get_argument_value(argument)) ;// x color_mult_ ;
	if(coord == 2) 
	{ 
		 vVextex_.back()->setColor(rgb_[0]/255.0,rgb_[1]/255.0,rgb_[2]/255.0);
	}
	return 1 ;
}
int CPlyLoad::get_vertex_data(p_ply_argument argument) 
{
	long coord ;
	ply_get_argument_user_data(argument, nil, &coord);
	assert(coord >= 0 && coord < 3) ;
	xyz_[coord] = double(ply_get_argument_value(argument)) ;
	if(coord == 2) 
	{  
		CVector3D pnt = CVector3D(xyz_);
		CScanVertex * tempVtx = new CScanVertex();
		tempVtx->point_ = pnt;
		vVextex_.push_back(tempVtx);
	}
	return 1;
}

int CPlyLoad::get_normal_data(p_ply_argument argument) 
{
	long coord ;
	ply_get_argument_user_data(argument, nil, &coord);
	assert(coord >= 0 && coord < 3) ;
	normal_[coord] = double(ply_get_argument_value(argument));
	if(coord == 2) 
	{ 
		CVector3D nrl = CVector3D(normal_[0],normal_[1],normal_[2]);
		vVextex_.back()->normal_ = nrl;
	}
	return 1 ;
} 
void CPlyLoad::check_for_Elements(p_ply ply)
{
	p_ply_element element = nil ;
	hasFaces = false;
	bool has_r     = false ;
	bool has_g     = false ;
	bool has_b     = false ;

	bool has_red   = false ;
	bool has_green = false ;
	bool has_blue  = false ;

	bool has_diffuse_red   = false ;
	bool has_diffuse_green = false ;
	bool has_diffuse_blue  = false ;

	bool has_normals = false;

	for(;;)
	{
		element = ply_get_next_element(ply, element) ;
		if(element == nil) { break ; }
		const char* elt_name = nil ;
		ply_get_element_info(element, &elt_name, nil) ;

		if(!strcmp(elt_name, "vertex")) {
			p_ply_property property = nil ;
			for(;;) {
				property = ply_get_next_property(element, property) ;
				if(property == nil) 
					break ;

				const char* prop_name = nil ;
				ply_get_property_info(property, &prop_name, nil, nil, nil) ;
				has_r = has_r || !strcmp(prop_name, "r") ;
				has_g = has_g || !strcmp(prop_name, "g") ;
				has_b = has_b || !strcmp(prop_name, "b") ;
				has_red   = has_red   || !strcmp(prop_name, "red")  ;
				has_green = has_green || !strcmp(prop_name, "green");
				has_blue  = has_blue  || !strcmp(prop_name, "blue") ;

				has_diffuse_red   = has_diffuse_red   || !strcmp(prop_name, "diffuse_red") ;
				has_diffuse_green = has_diffuse_green || !strcmp(prop_name, "diffuse_green") ;
				has_diffuse_blue  = has_diffuse_blue  || !strcmp(prop_name, "diffuse_blue") ;

				has_normals  = has_normals || !strcmp(prop_name, "nx");
				has_normals  = has_normals || !strcmp(prop_name, "ny");
				has_normals  = has_normals || !strcmp(prop_name, "nz");
			}
		} 
		if(!strcmp(elt_name, "face")) 
		{
			hasFaces = true;
		}
	}

	if(has_r && has_g && has_b) 
	{
		//color_mult_ = 1.0 ;
		ply_set_read_cb(ply, "vertex", "r", color_cb, this, 0) ;
		ply_set_read_cb(ply, "vertex", "g", color_cb, this, 1) ;
		ply_set_read_cb(ply, "vertex", "b", color_cb, this, 2) ;
	} else if(has_red && has_green && has_blue) 
	{
		//color_mult_ = 1.0 / 255.0 ;
		ply_set_read_cb(ply, "vertex", "red",  color_cb, this, 0) ;
		ply_set_read_cb(ply, "vertex", "green", color_cb, this, 1) ;
		ply_set_read_cb(ply, "vertex", "blue",  color_cb, this, 2) ;
	} else if(has_diffuse_red && has_diffuse_green && has_diffuse_blue) 
	{
		//color_mult_ = 1.0 / 255.0 ;
		ply_set_read_cb(ply, "vertex", "diffuse_red",   color_cb, this, 0) ;
		ply_set_read_cb(ply, "vertex", "diffuse_green", color_cb, this, 1) ;
		ply_set_read_cb(ply, "vertex", "diffuse_blue",  color_cb, this, 2) ;
	} 
	if (has_normals) {
		ply_set_read_cb(ply, "vertex", "nx",   normal_cb, this, 0) ;
		ply_set_read_cb(ply, "vertex", "ny",   normal_cb, this, 1) ;
		ply_set_read_cb(ply, "vertex", "nz",   normal_cb, this, 2) ;
	}
	if (hasFaces)
	{
		int nFace = ply_set_read_cb(ply,"face","vertex_indices",face_cb,this,0);
	}

}

void CPlyLoad::loadPly(const char* filename,CPointCloud * pcPtr)
{
	pcPtr->clearAll();
	p_ply ply = ply_open(filename, nil, 0, nil) ;

	if(ply == nil)
	{
		return ;
	}

	if(!ply_read_header(ply)) 
	{
		ply_close(ply) ;
		return ;
	}
 
	check_for_Elements(ply) ;

	long nvertices = ply_set_read_cb(ply, "vertex", "x", vertex_cb, this, 0) ;
	ply_set_read_cb(ply, "vertex", "y", vertex_cb, this, 1) ;
	ply_set_read_cb(ply, "vertex", "z", vertex_cb, this, 2) ; 
	//vVextex_.resize(nvertices);

	if(!ply_read(ply)) 
	{
		ply_close(ply) ;  
		return ;
	}
	else
	{
		ply_close(ply) ;

		std::vector<CScanVertex*>::iterator it = vVextex_.begin();
		 
		ProgressLogger progress(vVextex_.size());
		int id = 0;
		//
		//int downScale = 4;	// temp download point cloud.
		//int tempDownI = 0;	// temp download point cloud.
		for (; it!=vVextex_.end();++it)
		{   
			progress.notify(id++);
			if (progress.is_canceled())
				break;
			//tempDownI ++;	// temp download point cloud.
			//if ( 0 != tempDownI % downScale )// temp download point cloud.
			//{				// temp download point cloud.
			//	continue;	// temp download point cloud.
			//}				// temp download point cloud.
			pcPtr->m_vtxPC.push_back(*it); 
		}
 
		return ;
	}
}




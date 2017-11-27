#pragma once
#include <vector>
#include <string>

#include "CPointCloud.h"
//#include "CVertexGroup.h"


namespace lml_IO
{
	std::string get_filename_extension(const std::string file_name);
	void read_model_file(std::string & filename);
	//bool read_Polyhedron_From_OFF(const char* filename, cgPolyhedron & ph);
	void load_blab_to_PCGroup(const std::string& file_name, std::vector<CVertexGroup*> & groups) ;
	bool write_blab_From_RANSAC(const char* file_name, std::vector<CVertexGroup*> & groups);
	//void write_fitbox_From_Fitbox( const char* filename , CFitBoxes * m_fitBoxes );
	//bool write_OFF_From_Polyhedron(const char* filename, cgPolyhedron & ph);
	void load_PC_From_xyzn(CPointCloud *& pc, const char * filename);
}
#include "ioFile.h"
#include <fstream>
#include <vector>
//#include "fitBoxes.h"
#include "CPolyHedron.h" 

namespace lml_IO
{
	std::string read_binary_string(std::ifstream& input) 
	{
		int size_int = sizeof(int);
		int size_char = sizeof(char);

		int num_char = 0;    
		input.read((char*)&num_char, size_int);  

		std::string str;
		for (int i=0; i<num_char; ++i) 
		{
			char c;
			input.read(&c, size_char);
			if (c != ' ')
				str.push_back(c);
			else
				str.push_back('-');
		}

		return str;
	}
	void load_blab_to_PCGroup(const std::string& file_name, std::vector<CVertexGroup*> & groups)
	{
		std::ifstream input(file_name.c_str(), std::fstream::binary) ;
		if(input.fail()) {
			//Logger::output(title()) << "could not open file\'" << file_name << "\'" << Logger::endl();
			return ;
		}

		int float_size = sizeof(float);
		int int_size = sizeof(int);

		int num_points = 0;
		input.read((char*)&num_points, int_size); 

		float* data = new float[num_points * 10]; // 10 coords per point.
		input.read((char*)data, num_points * 10 * float_size); 

		
		
		std::vector<CVector3D> pnts,nrls,clrs; 
		
		for (int i=0; i<num_points; ++i) 
		{
			CVector3D pnt(data[i*10], data[i*10 + 1], data[i*10 + 2]);
			CVector3D nrl(data[i*10 + 3], data[i*10 + 4], data[i*10 + 5]);
			CVector3D clr(data[i*10 + 6], data[i*10 + 7], data[i*10 + 8]);
			
			pnts.push_back(pnt);
			nrls.push_back(nrl);
			clrs.push_back(clr);
		}
		delete [] data;  

		int num_groups = 0;
		//groups.resize(num_groups);//[num_points];
		input.read((char*)&num_groups, int_size); 
 
		for (int i=0; i<num_groups; ++i) 
		{ 
			// each group file format:
			//////////////////////////////////////////////////////////////////////////
			// group_label: label  // the first group info
			// group_color: color
			// CVector3D	m_midPnt;		// planar center point 
			// cgPlane3f	m_cgPlane;		// plane 
			// group_num_points: num
			// idx ...
			std::string label = read_binary_string(input);
			float mColor[4];
			input.read((char*)mColor, 4*float_size); 
			double mx, my,mz; 
			float pa, pb, pc, pd;

			input.read((char*)&mx, sizeof(double)); 
			input.read((char*)&my, sizeof(double)); 
			input.read((char*)&mz, sizeof(double)); 
			input.read((char*)&pa, float_size); 
			input.read((char*)&pb, float_size); 
			input.read((char*)&pc, float_size); 
			input.read((char*)&pd, float_size);   


			CVertexGroup* pgi =new CVertexGroup(); 

			CVector3D midpnt (mx,my,mz);
			pgi->m_midPnt = midpnt;
			cgPlane3f pln(pa,pb,pc,pd);
			pgi->m_cgPlane = pln;
			pgi->m_color = Colorf(mColor[0],mColor[1],mColor[2],1.0f);

			int num_points = 0;
			input.read((char*)&num_points, int_size);  
			int* indices = new int[num_points];
			if (indices == nil) 
			{ 
				return;
			} 
			input.read((char*)indices, num_points * int_size); 

			for (int j=0;j<num_points;j++)
			{
				CScanVertex * tempVtx = new CScanVertex();
				tempVtx->point_ =  pnts[indices[j]];
				tempVtx->normal_ = nrls[indices[j]];  
				tempVtx->setColor (mColor[0],mColor[1],mColor[2]);
				pgi->push_back(tempVtx);
			}
			
			delete [] indices; 
			
			groups.push_back(pgi);
			int num_children = 0;
			input.read((char*)&num_children, int_size); 
			  
		}
	}
	// string are stored as array of chars in binary file
	void write_binary_string(std::ofstream& output, const std::string& str) 
	{
		int size_int = sizeof(int);
		int size_char = sizeof(char);

		int num_char = str.size();
		output.write((char*)&num_char, size_int);

		for (int i=0; i<num_char; ++i) 
		{
			char c = str[i];
			if (c == ' ')
				c = '-';
			output.write(&c, size_char);
		}
	}
	void blab_write_group(std::ofstream& output, CVertexGroup* group, std::vector<int> & indxes) 
	{
		int float_size = sizeof(float);
		int int_size = sizeof(int);

		std::string label = "unknown";//group->label();
		write_binary_string(output, label);

		//////////////////////////////////////////////////////////////////////////
		// Leon: check why not work
		//std::string label = group->label();
		//int label_size = sizeof(label);
		//output.write((char*)&label_size, int_size);
		//output.write((char*)&label, label_size);
		
		//////////////////////////////////////////////////////////////////////////
		// group_label: label  // the first group info
		// group_color: color
		// CVector3D	m_midPnt;		// planar center point 
		// cgPlane3f	m_cgPlane;		// plane 
		// group_num_points: num
		// idx ...

		Colorf c(group->m_color);
		CVector3D &mPnt = group->m_midPnt;
		cgPlane3f & pln = group->m_cgPlane;
		float r = c.r();
		float g = c.g();
		float b = c.b();
		float a = c.a();
		double mx = mPnt.pVec[0];
		double my = mPnt.pVec[1];
		double mz = mPnt.pVec[2];
		float pa, pb, pc, pd;
		pa = pln.a(); pb=pln.b();pc=pln.c();pd=pln.d();
		output.write((char*)&r, float_size);
		output.write((char*)&g, float_size);
		output.write((char*)&b, float_size);
		output.write((char*)&a, float_size);
		output.write((char*)&mx, sizeof(double));
		output.write((char*)&my, sizeof(double));
		output.write((char*)&mz, sizeof(double));
		output.write((char*)&pa, float_size);
		output.write((char*)&pb, float_size);
		output.write((char*)&pc, float_size);
		output.write((char*)&pd, float_size);


		int num_point = group->size();
		output.write((char*)&num_point, int_size);

		for (unsigned int j=0; j<num_point; ++j) 
		{
			int idx = indxes[j];
			output.write((char*)&idx, int_size);
		}
	}

	bool write_blab_From_RANSAC(const char* file_name, std::vector<CVertexGroup*> & groups)
	{
		if (groups.size()<0)
		{
			return false;
		} 
		std::ofstream output(file_name, std::fstream::binary) ;
		if(output.fail()) 
		{ 
			return false;
		}

		std::vector<CVector3D> vts,nrls,clrs;

		std::vector<std::vector<int>> vIdxs;

		for (unsigned int i=0; i<groups.size(); ++i) 
		{
			const CVertexGroup* g = groups[i];
			std::vector<int> indexs;
			for(int j=0;j<g->size();j++)
			{
				CVector3D pntj = g->at(j)->point_;
				CVector3D nrl  = g->at(j)->normal_;
				CVector3D clr  = CVector3D(g->at(j)->color_.r(),g->at(j)->color_.g(),g->at(j)->color_.b());
				vts.push_back(pntj);
				nrls.push_back(nrl);
				clrs.push_back(clr);
				indexs.push_back(vts.size()-1);
			}
			vIdxs.push_back(indexs);
		}

		int float_size = sizeof(float);
		int int_size = sizeof(int);

		unsigned int num = vts.size();

		//num
		//x  y  z  nx  ny  nz r g b a
		output.write((char*)&num, int_size);
		for (unsigned int i=0; i<num; ++i) 
		{ 
			CVector3D & p = vts[i];
			CVector3D & n = nrls[i];
			CVector3D & c = clrs[i]; 

			float values[10] = {p.pVec[0], p.pVec[1],p.pVec[2],n.pVec[0],n.pVec[1],n.pVec[2],c.pVec[0],c.pVec[1],c.pVec[2],1.0f};
			output.write((char*)values, 10 * float_size);
		}

		// num_groups: num
		// group_label: label  // the first group info
		// group_color: color
		// CVector3D	m_midPnt;		// planar center point 
		// cgPlane3f	m_cgPlane;		// plane 
		// group_num_points: num
		// idx ...
		int num_groups = groups.size();
		output.write((char*)&num_groups, int_size); 

		for (int i=0; i<num_groups; ++i) 
		{ 
			CVertexGroup* g = groups[i];
			std::vector<int> &indexes = vIdxs[i];
			blab_write_group(output, g, indexes);

			// children
			int chld_num = 0;
			output.write((char*)&chld_num, int_size);
			// 		std::vector<VertexGroup*> children = g->children();
			// 		int chld_num = children.size();
			// 		output.write((char*)&chld_num, int_size);
			// 		for (int j=0; j<chld_num; ++j) {
			// 			VertexGroup* chld = children[j];
			// 			blab_write_group(output, chld);
			// 		}
		}

		return true;
	}


	//bool read_Polyhedron_From_OFF(const char* filename, cgPolyhedron & ph)
	//{
	//	std::ifstream cin(filename);
	//	cin>>ph;

	//}
	bool write_OFF_From_Polyhedron(const char* filename, cgPolyhedron & ph)
	{
		std::ofstream out(filename);
		out << "OFF" << std::endl << ph.size_of_vertices() << ' '
			<< ph.size_of_facets() << " 0" << std::endl;
		std::copy( ph.points_begin(), ph.points_end(),
			std::ostream_iterator<cgPoint3f>( out, "\n"));
		for (cgPolyhedron::Facet_iterator i = ph.facets_begin(); i != ph.facets_end(); ++i) 
		{
			cgPolyhedron::Halfedge_around_facet_circulator j = i->facet_begin();
			// Facets in polyhedral surfaces are at least triangles.
			CGAL_assertion( CGAL::circulator_size(j) >= 3);
			out << CGAL::circulator_size(j) << ' ';
			do 
			{
				out << ' ' << std::distance(ph.vertices_begin(), j->vertex());
			} while ( ++j != i->facet_begin());
			out << std::endl;
		}
		return true;
	}

	void read_model_file( std::string & filename )
	{

	}

	std::string get_filename_extension( const std::string file_name )
	{
		std::string::size_type dot = file_name.find_last_of('.');
		const char * const PATH_SEPARATORS = "/\\";
		std::string::size_type slash = file_name.find_last_of(PATH_SEPARATORS);
		// extension(fileName.toStdString());
		if (dot == std::string::npos || (slash != std::string::npos && dot < slash)) 
		{
			return "";
		}
		else
		{
			std::string ext = std::string(file_name.begin() + dot + 1, file_name.end());
			return ext;
		}
	}

	//void write_fitbox_From_Fitbox( const char* filename , CFitBoxes * m_fitBoxes )
	//{
	//	// *.fitbox
	//	// num polyhedron.size
	//	// 
	//}
	void load_PC_From_xyzn(CPointCloud *& pc, const char * filename)
	{
		if (pc != NULL)
		{
			delete pc;
		}
		pc = new CPointCloud(); 
		double m_dbGroundZ;
		FILE * file;
		fopen_s( & file, filename, "r" );

		char buf[ 1024 ];
		double x, y, z, nx, ny, nz;
		while ( fgets( buf, 1024, file ) != NULL ) 
		{
			if ( buf[ 0 ] == '#' ) 
			{
				if ( strncmp( buf, "# ground ", 9 ) == 0 ) 
				{
					sscanf_s( & buf[ 9 ], "%lf", & m_dbGroundZ );
				}
			} else if (buf[0] == '%' || buf[0] == '\n')
			{
				continue;
			}
			else
			{
				int cnt = sscanf_s( buf, "%lf %lf %lf %lf %lf %lf", &x, &y, &z, &nx, &ny, &nz );
				if (cnt == 6)
				{
					CScanVertex * pnt = new CScanVertex(CVector3D(x,y,z),CVector3D(nx,ny,nz),Colorf(0.5f,0.5f,0.5f)); 
					pc->m_vtxPC.push_back(pnt);
				}
				else if (cnt == 3)
				{ 
					CScanVertex * pnt = new CScanVertex(CVector3D(x,y,z),CVector3D(0,0,0),Colorf(0.5f,0.5f,0.5f)); 
					pc->m_vtxPC.push_back(pnt);
				} 
			}
		}

		fclose( file );
		pc->build_boundingBox();
	}

}
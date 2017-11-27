#include "CMeshObj.h"  
#include <gl/glut.h>
#include <iostream>
#include <sstream>
#include <fstream>
 
#include "color.h"
#include "Parameters.h"
#include "math_global.h"

CMeshObj::CMeshObj(void) 
{ 
}

CMeshObj::~CMeshObj(void)
{
	m_vecVertex.clear();
	m_vecTriangle.clear(); 
	m_vecTriNrl.clear();
	m_vecElemType.clear();
}

// creat ////////////////////////////////////////////////////////////////////////
bool CMeshObj::IsDegenrateFace( CMeshObj::MeshTriangle & f ) 
{ 
	if ( f.i[0] == -1 || f.i[1] == -1 || f.i[2] == -1 || f.i[0] == f.i[1] || f.i[1] == f.i[2] || f.i[2] == f.i[0] )
		return true; 
	else
		return false;
}

void CMeshObj::pushVecVertice(std::vector<CVector3D> & pnts)
{
	for (int i=0;i<pnts.size();i++)
	{
		MeshVertex mv (pnts[i]);
		m_vecVertex.push_back(mv);
	}
}

void CMeshObj::pushOneVertex(CVector3D & pnt)
{ 
	MeshVertex mv (pnt);
	m_vecVertex.push_back(mv);
}

void CMeshObj::PushOneFace(int i, int j, int k ) 
{
	CMeshObj::MeshTriangle f;
	f.i[0] = i;
	f.i[1] = j;
	f.i[2] = k;
	if ( ! IsDegenrateFace( f ) )
		this->m_vecTriangle.push_back( f );
}
// load //////////////////////////////////////////////////////////////////////////
CMeshObj * CMeshObj::loadFromObj( const std::string& file_name )
{
	std::ifstream  input(file_name.c_str());
	if (input.fail()) 
	{
		std::cout << "unable to open file: \'" << file_name << "\'" << std::endl;
		return NULL;
	} 
	CMeshObj * mesh = new CMeshObj();

	// Vertex index starts by 1 in obj format.
	while(!input.eof()) 			
	{
		std::string line;
		std::getline(input, line);

		std::istringstream line_input(line);
		std::string keyword ;
		line_input >> keyword; 

		if(keyword == "v") 
		{ // read vertex coordinate
			double xi, yi, zi;
			line_input >> xi >> yi >> zi ; 
			MeshVertex vtx (CVector3D(xi,yi,zi));
			mesh->m_vecVertex.push_back(vtx);
		}				
		else if(keyword == "vt") 
		{  // read texture coordinate of vertex
			float x, y;
			line_input >> x >> y ;
			//tex_coords.push_back(t);
		}
		else if(keyword == "f") 
		{  
			int i0, i1, i2;
			line_input >> i0 >> i1 >> i2 ;
			MeshTriangle indices(i0-1,i1-1,i2-1);
			mesh->m_vecTriangle.push_back(indices);
			CVector3D & v0 = mesh->m_vecVertex[i0-1].v;
			CVector3D & v1 = mesh->m_vecVertex[i1-1].v;
			CVector3D & v2 = mesh->m_vecVertex[i2-1].v;
			CVector3D nrl = (v2-v1)^(v0-v1); 
			nrl.normalize();
			mesh->m_vecTriNrl.push_back(nrl);
			if (nrl.z_()<0.01)
			{
				mesh->m_vecElemType.push_back(T_WALL);
			}
			else
				mesh->m_vecElemType.push_back(T_ROOF);

		}
		else
			continue;
	}

	return mesh;
}

// draw ////////////////////////////////////////////////////////////////////////
void CMeshObj::drawMesh()
{  
	GLboolean on = glIsEnabled(GL_LIGHTING);
	
	// draw wire frame
	if (lml_param::showPHWirframe)
	{
		if(on)
			glDisable(GL_LIGHTING);

		//glEnable(GL_LINE_SMOOTH); 
		//glHint(GL_LINE_SMOOTH_HINT,GL_NICEST);
		glEnable(GL_POLYGON_SMOOTH); 
		glHint(GL_POLYGON_SMOOTH_HINT,GL_NICEST); 

		glColor4f(0.10f, 0.10f, 0.10f,1.0f); 
		glLineWidth(0.50f);
		glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);  

		for (unsigned int i=0; i<m_vecTriangle.size(); ++i) 
		{
			MeshTriangle & f = m_vecTriangle[i];
			int v0 = f.i[0];
			int v1 = f.i[1];
			int v2 = f.i[2];

			glBegin(GL_POLYGON);
			if (m_vecTriNrl.size()==m_vecTriangle.size())
			{
				CVector3D & nrl = m_vecTriNrl[i]; 
				glNormal3f((float)nrl.x_(), (float)nrl.y_(), (float)nrl.z_());
			}
			CVector3D & vtr0 = m_vecVertex[v0].v;
			glVertex3f((float)vtr0.x_(), (float)vtr0.y_(), (float)vtr0.z_());
			CVector3D & vtr1 = m_vecVertex[v1].v;
			glVertex3f((float)vtr1.x_(),(float) vtr1.y_(), (float)vtr1.z_());
			CVector3D & vtr2 = m_vecVertex[v2].v;
			glVertex3f((float)vtr2.x_(), (float)vtr2.y_(), (float)vtr2.z_());
			glEnd();
		}
		if(on)
			glEnable(GL_LIGHTING);
	}
	
	// draw a short line to present normal direction
	if (lml_param::showNormal && this->m_vecMeshMidPnt.size()==this->m_vecTriangle.size())
	{
		glColor4f(1.0f,1.0f,0,1.0f);
		glLineWidth(1.5f);
		for(int ni=0;ni<m_vecMeshMidPnt.size();ni+=10) 
		{
			CVector3D & pnt0 = m_vecMeshMidPnt[ni];
			CVector3D & nrl = m_vecTriNrl[ni];
			CVector3D pnt1 = pnt0 + 0.33*nrl;
			glBegin(GL_LINES);
			glVertex3f((float)pnt0.x_(),(float)pnt0.y_(),(float)pnt0.z_());
			glVertex3f((float)pnt1.x_(),(float)pnt1.y_(),(float)pnt1.z_());			
			glEnd();
		}
	}
	 

	// draw triangle facets

	//Colorf c(214.0f/255.0f,214.0f/255.0f,214.0f/255.0f,1.0f);
	//Colorf c(62.0f/255.0f,231.0f/255.0f,240.0f/255.0f,1.0f); 
	if(!on)
		glEnable(GL_LIGHTING);
	glEnable(GL_POLYGON_SMOOTH); 
	glHint(GL_POLYGON_SMOOTH_HINT,GL_NICEST);  
	glLineWidth(1.0f);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
 
	for (unsigned int i=0; i<m_vecTriangle.size(); ++i) 
	{   
		MeshTriangle & f = m_vecTriangle[i];
		int v0 = f.i[0];
		int v1 = f.i[1];
		int v2 = f.i[2];

		//Colorf c  (67/255.0f,225/255.0f,158/255.0f,1.0f); 
		
		if (!lml_param::rdmColor)
		{
			if (m_vecElemType.size() == m_vecTriangle.size())
			{ 
				switch (m_vecElemType[i])// draw color by roof and wall.
				{
				case T_WALL: 
					glColor4f(0.8f,0.8f,0.80f,0.80f);
					break;
				case T_ROOF: 
					//glColor4f(0.0f,0.6f,1.0f,0.80f);
					glColor4f(0.7f,0.70f,0.70f,0.80f);
					break;
				} 
			}
			else 
				glColor4f(0.5f,0.6f,0.7f,0.80f); 
		}  
		else
			glColor4f(0.9f,0.9f,0.9f,0.80f);

		glBegin(GL_POLYGON);
		if (m_vecTriNrl.size()==m_vecTriangle.size())
		{
			CVector3D & nrl = m_vecTriNrl[i]; 
			glNormal3f((float)nrl.x_(), (float)nrl.y_(), (float)nrl.z_());
		}
		CVector3D & vtr0 = m_vecVertex[v0].v;
		glVertex3f(vtr0.x_(), vtr0.y_(), vtr0.z_());
		CVector3D & vtr1 = m_vecVertex[v1].v;
		glVertex3f(vtr1.x_(), vtr1.y_(), vtr1.z_());
		CVector3D & vtr2 = m_vecVertex[v2].v;
		glVertex3f(vtr2.x_(), vtr2.y_(), vtr2.z_());

		glEnd();
	}
	if (!on)
	{
		glDisable(GL_LIGHTING);
	}

}
void CMeshObj::drawMeshObj( bool showNrl /*= false*/, bool rdmColor /*= false*/)
{ 
	GLboolean on = glIsEnabled(GL_LIGHTING);
	
	// draw the mesh contour.
	glColor4f(1.0f, 1.0f, 1.0f,1.0f); 
	glLineWidth(1.00f);
	/*glPolygonMode(GL_FRONT_AND_BACK, GL_LINE); 
	for (unsigned int i=0; i<m_vecTriangle.size(); ++i) 
	{
		MeshTriangle f = m_vecTriangle[i];
		int v0 = f.i[0];
		int v1 = f.i[1];
		int v2 = f.i[2];

		glBegin(GL_POLYGON);
		CVector3D & vtr0 = m_vecVertex[v0].v;
		glVertex3f((float)vtr0.x_(), (float)vtr0.y_(), (float)vtr0.z_());
		CVector3D & vtr1 = m_vecVertex[v1].v;
		glVertex3f((float)vtr1.x_(),(float) vtr1.y_(), (float)vtr1.z_());
		CVector3D & vtr2 = m_vecVertex[v2].v;
		glVertex3f((float)vtr2.x_(), (float)vtr2.y_(), (float)vtr2.z_());
		glEnd();
	}*/
	 
	for (int li=0;li<m_contour.size();li++)
	{
		glBegin(GL_LINES);
		CVector3D & vtr0 = m_vecVertex[m_contour[li]].v;
		glVertex3f((float)vtr0.x_(), (float)vtr0.y_(), (float)vtr0.z_());
		CVector3D & vtr1 = m_vecVertex[m_contour[++li]].v;
		glVertex3f((float)vtr1.x_(),(float) vtr1.y_(), (float)vtr1.z_());
		glEnd();
	}
	// draw the mesh contour.

	// draw a short line to present normal direction
	if (showNrl && this->m_vecMeshMidPnt.size()==this->m_vecTriangle.size())
	{
		glColor4f(1.0f,1.0f,0,1.0f);
		glLineWidth(3.0f);
		for(int ni=0;ni<m_vecMeshMidPnt.size();ni++)
		{
			CVector3D & pnt0 = m_vecMeshMidPnt[ni];
			CVector3D & nrl = m_vecTriNrl[ni];
			CVector3D pnt1 = pnt0 + 0.33*nrl;
			glBegin(GL_LINES);
			glVertex3f((float)pnt0.x_(),(float)pnt0.y_(),(float)pnt0.z_());
			glVertex3f((float)pnt1.x_(),(float)pnt1.y_(),(float)pnt1.z_());			
			glEnd();
		}
	}
	// draw a short line to present normal direction

	if (!on)
	{
		glEnable(GL_LIGHTING);
	} 

	glEnable( GL_POLYGON_SMOOTH ); 
	glHint(GL_POLYGON_SMOOTH_HINT,GL_NICEST);

	glPolygonMode(GL_FRONT, GL_FILL);

	for (unsigned int i=0; i<m_vecTriangle.size(); ++i) 
	{ 
		if (!rdmColor)
		{
			switch (m_vecElemType[i])// draw color by roof and wall.
			{
			case T_WALL:
				glColor4f(0.1f,0.95f,1.0f,1.0f);
				break;
			case T_ROOF:
				//glColor4f(0.1f,0.95f,1.0f,1.0f);
				glColor4f(0.0f,0.6f,1.0f,1.0f);
				break;
			} 
		}
		//glColor4f(0.8784f,1.0f,1.0f,1.0f);

		MeshTriangle f = m_vecTriangle[i];
		int v0 = f.i[0];
		int v1 = f.i[1];
		int v2 = f.i[2];

		CVector3D & nrl = m_vecTriNrl[i]; 
		glNormal3f((float)nrl.x_(), (float)nrl.y_(), (float)nrl.z_());

		glBegin(GL_POLYGON);
		CVector3D & vtr0 = m_vecVertex[v0].v;
		glVertex3f(vtr0.x_(), vtr0.y_(), vtr0.z_());
		CVector3D & vtr1 = m_vecVertex[v1].v;
		glVertex3f(vtr1.x_(), vtr1.y_(), vtr1.z_());
		CVector3D & vtr2 = m_vecVertex[v2].v;
		glVertex3f(vtr2.x_(), vtr2.y_(), vtr2.z_());

		glEnd();
	}

	if(!on)
		glDisable(GL_LIGHTING);
}

 
// save ////////////////////////////////////////////////////////////////////////

void CMeshObj::SaveToObj( const char * filename )
{ 
	FILE * m_pFile;
	fopen_s( & m_pFile, filename, "w" );
	fprintf_s( m_pFile, "#\n" );

	for ( int i = 0; i < ( int )m_vecVertex.size(); i++ ) 
	{
		fprintf_s( m_pFile, "v %.8f %.8f %.8f\n", m_vecVertex[ i ].v.x_(), m_vecVertex[ i ].v.y_(), m_vecVertex[ i ].v.z_() );
	}

	for ( int i = 0; i < ( int )m_vecTriangle.size(); i++ ) 
	{ 
		// In obj formate file, vertex index starts from 1. 
		fprintf_s( m_pFile, "f %d %d %d\n", m_vecTriangle[ i ].i[0]+1, m_vecTriangle[ i ].i[1]+1, m_vecTriangle[ i ].i[2]+1 );
	} 

	fclose( m_pFile );
	// only "delete" when this pointer is created by "new".
	//if (m_pFile != NULL)
	//{
	//	delete m_pFile;
	//}
}


void CMeshObj::combineTwoMeshObj( CMeshObj *&meshDst, CMeshObj *meshSrc )
{
	if (!meshSrc || !meshDst)
	{
		std::cout<<"error: mesh pointer has problems!\n";
	}
	int sz0 = meshDst->m_vecVertex.size();
	meshDst->m_vecVertex.insert(meshDst->m_vecVertex.end(),meshSrc->m_vecVertex.begin(),meshSrc->m_vecVertex.end());

	for (int i=0;i<meshSrc->m_vecTriangle.size(); i++)
	{
		int vi[3];
		vi[0] = meshSrc->m_vecTriangle[i].i[0]+sz0;
		vi[1] = meshSrc->m_vecTriangle[i].i[1]+sz0;
		vi[2] = meshSrc->m_vecTriangle[i].i[2]+sz0; 

		MeshTriangle tri(vi[0],vi[1],vi[2]);
		meshDst->m_vecTriangle.push_back(tri);
	}

}

cgBbox3f CMeshObj::bbox() 
{  
	std::vector<cgPoint3f> points;

	for (unsigned int i=0; i<m_vecVertex.size(); ++i) 
	{
		CVector3D & vi=m_vecVertex[i].v;
		points.push_back(cgPoint3f (vi.x_(),vi.y_(),vi.z_())); 
	} 
	 
	return Global::bbox_of(points); 
}

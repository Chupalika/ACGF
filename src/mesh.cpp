#include "glCanvas.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>

#include "mesh.h"
#include "edge.h"
#include "vertex.h"
#include "triangle.h"

// to give a unique id number to each triangles
int Triangle::next_triangle_id = 0;


// =======================================================================
// MESH DESTRUCTOR 
// =======================================================================

Mesh::~Mesh() {
  cleanupVBOs();

  // delete all the triangles
  std::vector<Triangle*> todo;
  for (triangleshashtype::iterator iter = triangles.begin();
       iter != triangles.end(); iter++) {
    Triangle *t = iter->second;
    todo.push_back(t);
  }
  int num_triangles = todo.size();
  for (int i = 0; i < num_triangles; i++) {
    removeTriangle(todo[i]);
  }
  // delete all the vertices
  int num_vertices = numVertices();
  for (int i = 0; i < num_vertices; i++) {
    delete vertices[i];
  }
}

// =======================================================================
// MODIFIERS:   ADD & REMOVE
// =======================================================================

Vertex* Mesh::addVertex(const glm::vec3 &position) {
  int index = numVertices();
  Vertex *v = new Vertex(index, position);
  vertices.push_back(v);
  if (numVertices() == 1)
    bbox = BoundingBox(position,position);
  else 
    bbox.Extend(position);
  return v;
}


void Mesh::addTriangle(Vertex *a, Vertex *b, Vertex *c) {
  // create the triangle
  Triangle *t = new Triangle();
  // create the edges
  Edge *ea = new Edge(a,b,t);
  Edge *eb = new Edge(b,c,t);
  Edge *ec = new Edge(c,a,t);
  // point the triangle to one of its edges
  t->setEdge(ea);
  // connect the edges to each other
  ea->setNext(eb);
  eb->setNext(ec);
  ec->setNext(ea);
  // verify these edges aren't already in the mesh 
  // (which would be a bug, or a non-manifold mesh)
  assert (edges.find(std::make_pair(a,b)) == edges.end());
  assert (edges.find(std::make_pair(b,c)) == edges.end());
  assert (edges.find(std::make_pair(c,a)) == edges.end());
  // add the edges to the master list
  edges[std::make_pair(a,b)] = ea;
  edges[std::make_pair(b,c)] = eb;
  edges[std::make_pair(c,a)] = ec;
  // connect up with opposite edges (if they exist)
  edgeshashtype::iterator ea_op = edges.find(std::make_pair(b,a)); 
  edgeshashtype::iterator eb_op = edges.find(std::make_pair(c,b)); 
  edgeshashtype::iterator ec_op = edges.find(std::make_pair(a,c)); 
  if (ea_op != edges.end()) { ea_op->second->setOpposite(ea); }
  if (eb_op != edges.end()) { eb_op->second->setOpposite(eb); }
  if (ec_op != edges.end()) { ec_op->second->setOpposite(ec); }
  // add the triangle to the master list
  assert (triangles.find(t->getID()) == triangles.end());
  triangles[t->getID()] = t;
}


void Mesh::removeTriangle(Triangle *t) {
  Edge *ea = t->getEdge();
  Edge *eb = ea->getNext();
  Edge *ec = eb->getNext();
  Vertex *a = ea->getStartVertex();
  Vertex *b = eb->getStartVertex();
  Vertex *c = ec->getStartVertex();
  // remove these elements from master lists
  edges.erase(std::make_pair(a,b)); 
  edges.erase(std::make_pair(b,c)); 
  edges.erase(std::make_pair(c,a)); 
  triangles.erase(t->getID());
  // clean up memory
  delete ea;
  delete eb;
  delete ec;
  delete t;
}


// =======================================================================
// Helper functions for accessing data in the hash table
// =======================================================================

Edge* Mesh::getMeshEdge(Vertex *a, Vertex *b) const {
  edgeshashtype::const_iterator iter = edges.find(std::make_pair(a,b));
  if (iter == edges.end()) return NULL;
  return iter->second;
}

Vertex* Mesh::getChildVertex(Vertex *p1, Vertex *p2) const {
  vphashtype::const_iterator iter = vertex_parents.find(std::make_pair(p1,p2)); 
  if (iter == vertex_parents.end()) return NULL;
  return iter->second; 
}

void Mesh::setParentsChild(Vertex *p1, Vertex *p2, Vertex *child) {
  assert (vertex_parents.find(std::make_pair(p1,p2)) == vertex_parents.end());
  vertex_parents[std::make_pair(p1,p2)] = child; 
}


// =======================================================================
// the load function parses very simple .obj files
// the basic format has been extended to allow the specification 
// of crease weights on the edges.
// =======================================================================

#define MAX_CHAR_PER_LINE 200

void Mesh::Load(const std::string &input_file) {

  std::ifstream istr(input_file.c_str());
  if (!istr) {
    std::cout << "ERROR! CANNOT OPEN: " << input_file << std::endl;
    return;
  }

  char line[MAX_CHAR_PER_LINE];
  std::string token, token2;
  float x,y,z;
  int a,b,c;
  int index = 0;
  int vert_count = 0;
  int vert_index = 1;

  // read in each line of the file
  while (istr.getline(line,MAX_CHAR_PER_LINE)) { 
    // put the line into a stringstream for parsing
    std::stringstream ss;
    ss << line;

    // check for blank line
    token = "";   
    ss >> token;
    if (token == "") continue;

    if (token == std::string("usemtl") ||
	token == std::string("g")) {
      vert_index = 1; 
      index++;
    } else if (token == std::string("v")) {
      vert_count++;
      ss >> x >> y >> z;
      addVertex(glm::vec3(x,y,z));
    } else if (token == std::string("f")) {
      a = b = c = -1;
      ss >> a >> b;
      // handle faces with > 3 vertices
      // assume the face can be triangulated with a triangle fan
      while (ss >> c) {
        int a_ = a-vert_index;
        int b_ = b-vert_index;
        int c_ = c-vert_index;
        assert (a_ >= 0 && a_ < numVertices());
        assert (b_ >= 0 && b_ < numVertices());
        assert (c_ >= 0 && c_ < numVertices());
        addTriangle(getVertex(a_),getVertex(b_),getVertex(c_));
        b = c;
      }
    } else if (token == std::string("e")) {
      a = b = -1;
      ss >> a >> b >> token2;
      // whoops: inconsistent file format, don't subtract 1
      assert (a >= 0 && a <= numVertices());
      assert (b >= 0 && b <= numVertices());
      if (token2 == std::string("inf")) x = 1000000; // this is close to infinity...
      x = atof(token2.c_str());
      Vertex *va = getVertex(a);
      Vertex *vb = getVertex(b);
      Edge *ab = getMeshEdge(va,vb);
      Edge *ba = getMeshEdge(vb,va);
      assert (ab != NULL);
      assert (ba != NULL);
      ab->setCrease(x);
      ba->setCrease(x);
    } else if (token == std::string("vt")) {
    } else if (token == std::string("vn")) {
    } else if (token[0] == '#') {
    } else {
      printf ("LINE: '%s'",line);
    }
  }

  std::cout << "Loaded " << numTriangles() << " triangles." << std::endl;

  assert (numTriangles() > 0);
  num_mini_triangles = 0;
}


// =======================================================================
// DRAWING
// =======================================================================

glm::vec3 ComputeNormal(const glm::vec3 &p1, const glm::vec3 &p2, const glm::vec3 &p3) {
  glm::vec3 v12 = p2;
  v12 -= p1;
  glm::vec3 v23 = p3;
  v23 -= p2;
  glm::vec3 normal = glm::cross(v12,v23);
  normal = glm::normalize(normal);
  return normal;
}


void Mesh::initializeVBOs() {
  HandleGLError("enter initialize VBOs");

  // create a pointer for the vertex & index VBOs
  glGenVertexArrays(1, &mesh_VAO);
  glBindVertexArray(mesh_VAO);
  glGenBuffers(1, &mesh_tri_verts_VBO);
  glGenBuffers(1, &mesh_tri_indices_VBO);
  // and the data to pass to the shaders
  GLCanvas::MatrixID = glGetUniformLocation(GLCanvas::programID, "MVP");
  GLCanvas::LightID = glGetUniformLocation(GLCanvas::programID, "LightPosition_worldspace");
  GLCanvas::ViewMatrixID = glGetUniformLocation(GLCanvas::programID, "V");
  GLCanvas::ModelMatrixID = glGetUniformLocation(GLCanvas::programID, "M");

  GLCanvas::wireframeID = glGetUniformLocation(GLCanvas::programID, "wireframe");

  // call this the first time...
  setupVBOs();
  HandleGLError("leaving initializeVBOs");
}


// boundary edges are red, crease edges are yellow
glm::vec3 EdgeColor(Edge *e) {
  if (e->getOpposite() == NULL) {
    return glm::vec3(1,0,0); 
  } else if (e->getCrease() > 0) {
    return glm::vec3(1,1,0);
  } else {
    return glm::vec3(0,0,0.0);
  }
}


void Mesh::TriVBOHelper( std::vector<glm::vec3> &indexed_verts,
                         std::vector<unsigned int> &mesh_tri_indices,
                         const glm::vec3 &pos_a,
                         const glm::vec3 &pos_b,
                         const glm::vec3 &pos_c,
                         const glm::vec3 &normal_a,
                         const glm::vec3 &normal_b,
                         const glm::vec3 &normal_c,
                         const glm::vec3 &color_ab,
                         const glm::vec3 &color_bc,
                         const glm::vec3 &color_ca) {

  /*
  // To create a wireframe rendering...
  // Each mesh triangle is actually rendered as 3 small triangles
  //           b
  //          /|\
  //         / | \
  //        /  |  \
  //       /   |   \   
  //      /    |    \  
  //     /    .'.    \   
  //    /  .'     '.  \  
  //   /.'           '.\ 
  //  a-----------------c
  //
  */
  
  // the center is white, the colors of the two vertices depend on
  // whether the edge is a boundary edge (red) or crease edge (yellow)
  glm::vec3 center_color(1,1,1);
  // use simple averaging to find centroid & average normal
  glm::vec3 centroid = 1.0f / 3.0f * (pos_a + pos_b + pos_c);
  glm::vec3 normal = normal_a + normal_b + normal_c;
  normal = glm::normalize(normal);

  int i = indexed_verts.size()/3;

  if (args->wireframe) {
    // WIREFRAME

    // make the 3 small triangles
    indexed_verts.push_back(pos_a);
    indexed_verts.push_back(normal_a);
    indexed_verts.push_back(color_ab);
    indexed_verts.push_back(pos_b);
    indexed_verts.push_back(normal_b);
    indexed_verts.push_back(color_ab);
    indexed_verts.push_back(centroid);
    indexed_verts.push_back(normal);
    indexed_verts.push_back(center_color);
    
    indexed_verts.push_back(pos_b);
    indexed_verts.push_back(normal_b);
    indexed_verts.push_back(color_bc);
    indexed_verts.push_back(pos_c);
    indexed_verts.push_back(normal_c);
    indexed_verts.push_back(color_bc);
    indexed_verts.push_back(centroid);
    indexed_verts.push_back(normal);
    indexed_verts.push_back(center_color);
    
    indexed_verts.push_back(pos_c);
    indexed_verts.push_back(normal_c);
    indexed_verts.push_back(color_ca);
    indexed_verts.push_back(pos_a);
    indexed_verts.push_back(normal_a);
    indexed_verts.push_back(color_ca);
    indexed_verts.push_back(centroid);
    indexed_verts.push_back(normal);
    indexed_verts.push_back(center_color);
    
    // add all of the triangle vertices to the indices list
    for (int j = 0; j < 9; j++) {
      mesh_tri_indices.push_back(i+j);
    }
  } else {
    // NON WIREFRAME
    // Note: gouraud shading with the mini triangles looks bad... :(
    
    // make the 3 small triangles
    indexed_verts.push_back(pos_a);
    indexed_verts.push_back(normal_a);
    indexed_verts.push_back(center_color);
    indexed_verts.push_back(pos_b);
    indexed_verts.push_back(normal_b);
    indexed_verts.push_back(center_color);
    indexed_verts.push_back(pos_c);
    indexed_verts.push_back(normal_c);
    indexed_verts.push_back(center_color);
 
    // add all of the triangle vertices to the indices list
    for (int j = 0; j < 3; j++) {
      mesh_tri_indices.push_back(i+j);
    }
  }

}


void Mesh::setupVBOs() {
  HandleGLError("enter setupVBOs");

  std::vector<glm::vec3> indexed_verts;
  std::vector<unsigned int> mesh_tri_indices;
   

  // write the vertex & triangle data
  for (triangleshashtype::iterator iter = triangles.begin();
       iter != triangles.end(); iter++) {

    Triangle *t = iter->second;
    
    // grab the vertex positions
    glm::vec3 a = (*t)[0]->getPos();
    glm::vec3 b = (*t)[1]->getPos();
    glm::vec3 c = (*t)[2]->getPos();
    
    // determine edge colors (when wireframe is enabled)
    glm::vec3 edgecolor_ab = EdgeColor(t->getEdge());
    glm::vec3 edgecolor_bc = EdgeColor(t->getEdge()->getNext());
    glm::vec3 edgecolor_ca = EdgeColor(t->getEdge()->getNext()->getNext());
    
	if (args->gouraud) {
		// =====================================
		// ASSIGNMENT: complete this functionality
		// =====================================
		  //define edges
		Edge* edge_ab = t->getEdge();
		Edge* edge_bc = edge_ab->getNext();
		Edge* edge_ca = edge_bc->getNext();

		//calculating average normal of vertex a
		glm::vec3 normala = ComputeNormal(a, b, c);
		edge_ab = edge_ab->getNext()->getNext()->getOpposite();
		while (edge_ab != NULL && edge_ab->getTriangle() != t) {
			//compute the normal of adjacent triangle
			Triangle* t2 = edge_ab->getTriangle();
			glm::vec3 a2 = (*t2)[0]->getPos();
			glm::vec3 b2 = (*t2)[1]->getPos();
			glm::vec3 c2 = (*t2)[2]->getPos();
			glm::vec3 normala2 = ComputeNormal(a2, b2, c2);
			normala += normala2;

			//go to next edge
			edge_ab = edge_ab->getNext()->getNext()->getOpposite();
		}
		normala /= 6.0f;

		//calculating average normal of vertex b
		glm::vec3 normalb = ComputeNormal(a, b, c);
		edge_bc = edge_bc->getNext()->getNext()->getOpposite();
		while (edge_bc != NULL && edge_bc->getTriangle() != t) {
			//compute the normal of adjacent triangle
			Triangle* t2 = edge_bc->getTriangle();
			glm::vec3 a2 = (*t2)[0]->getPos();
			glm::vec3 b2 = (*t2)[1]->getPos();
			glm::vec3 c2 = (*t2)[2]->getPos();
			glm::vec3 normalb2 = ComputeNormal(a2, b2, c2);
			normalb += normalb2;

			//go to next edge
			edge_bc = edge_bc->getNext()->getNext()->getOpposite();
		}
		normalb /= 6.0f;

		//calculating average normal of vertex c
		glm::vec3 normalc = ComputeNormal(a, b, c);
		edge_ca = edge_ca->getNext()->getNext()->getOpposite();
		while (edge_ca != NULL && edge_ca->getTriangle() != t) {
			//compute the normal of adjacent triangle
			Triangle* t2 = edge_ca->getTriangle();
			glm::vec3 a2 = (*t2)[0]->getPos();
			glm::vec3 b2 = (*t2)[1]->getPos();
			glm::vec3 c2 = (*t2)[2]->getPos();
			glm::vec3 normalc2 = ComputeNormal(a2, b2, c2);
			normalc += normalc2;

			//go to next edge
			edge_ca = edge_ca->getNext()->getNext()->getOpposite();
		}
		normalc /= 6.0f;
		
		TriVBOHelper(indexed_verts, mesh_tri_indices,
			a, b, c,
			normala, normalb, normalc,
			edgecolor_ab, edgecolor_bc, edgecolor_ca);
			
      
    } else {
      // for flat shading, use the triangle normal at each vertex
      // use the normal of the triangl
      glm::vec3 normal = ComputeNormal(a,b,c);
      TriVBOHelper(indexed_verts,mesh_tri_indices,
                   a,b,c,
                   normal,normal,normal,
                   edgecolor_ab,edgecolor_bc,edgecolor_ca);

    }
  }
        
  // the vertex data
  glBindBuffer(GL_ARRAY_BUFFER, mesh_tri_verts_VBO);
  glBufferData(GL_ARRAY_BUFFER, indexed_verts.size() * sizeof(glm::vec3), &indexed_verts[0], GL_STATIC_DRAW);
  // the index data (refers to vertex data)
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mesh_tri_indices_VBO);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, mesh_tri_indices.size() * sizeof(unsigned int), &mesh_tri_indices[0] , GL_STATIC_DRAW);

  num_mini_triangles = mesh_tri_indices.size() / 3.0;

  HandleGLError("leaving setupVBOs");
}


void Mesh::drawVBOs(const glm::mat4 &ProjectionMatrix,const glm::mat4 &ViewMatrix,const glm::mat4 &ModelMatrix) {
  HandleGLError("enter drawVBOs");

  // prepare data to send to the shaders
  glm::mat4 MVP = ProjectionMatrix * ViewMatrix * ModelMatrix;

  glm::vec3 lightPos = glm::vec3(4,4,4);
  glUniform3f(GLCanvas::LightID, lightPos.x, lightPos.y, lightPos.z);
  glUniformMatrix4fv(GLCanvas::MatrixID, 1, GL_FALSE, &MVP[0][0]);
  glUniformMatrix4fv(GLCanvas::ModelMatrixID, 1, GL_FALSE, &ModelMatrix[0][0]);
  glUniformMatrix4fv(GLCanvas::ViewMatrixID, 1, GL_FALSE, &ViewMatrix[0][0]);
  glUniform1i(GLCanvas::wireframeID, args->wireframe);

  // triangle vertex positions
  glBindBuffer(GL_ARRAY_BUFFER, mesh_tri_verts_VBO);
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0,                  // attribute
			3,                  // size
			GL_FLOAT,           // type
			GL_FALSE,           // normalized?
			3*sizeof(glm::vec3),// stride
			(void*)0            // array buffer offset
                        );
// triangle vertex normals
glEnableVertexAttribArray(1);
glVertexAttribPointer(1,                      // attribute
	3,                      // size
	GL_FLOAT,               // type
	GL_FALSE,               // normalized?
	3 * sizeof(glm::vec3),    // stride
	(void*)sizeof(glm::vec3)// array buffer offset
);
// triangle vertex colors
glEnableVertexAttribArray(2);
glVertexAttribPointer(2,                          // attribute
	3,                          // size
	GL_FLOAT,                   // type
	GL_FALSE,                   // normalized?
	3 * sizeof(glm::vec3),        // stride
	(void*)(sizeof(glm::vec3) * 2)// array buffer offset
);
// triangle indices
glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mesh_tri_indices_VBO);
glDrawElements(GL_TRIANGLES,         // mode
	num_mini_triangles * 3, // count
	GL_UNSIGNED_INT,      // type
	(void*)0              // element array buffer offset
);
glDisableVertexAttribArray(0);
glDisableVertexAttribArray(1);
glDisableVertexAttribArray(2);

// =================================
// draw the different types of edges
//  if (args->wireframe) {

HandleGLError("leaving drawVBOs");
}


void Mesh::cleanupVBOs() {
	glDeleteBuffers(1, &mesh_VAO);
	glDeleteBuffers(1, &mesh_tri_verts_VBO);
	glDeleteBuffers(1, &mesh_tri_indices_VBO);
}


// =================================================================
// SUBDIVISION
// =================================================================


void Mesh::LoopSubdivision() {
	printf("Subdivide the mesh!\n");

	// =====================================
	// ASSIGNMENT: complete this functionality
	// =====================================

	//have a vector of initial vertexes
	std::vector<Vertex*> initialvertices = vertices;

	//have a vector of modified positions of these initial vertices
	std::vector<glm::vec3> modifiedpositions;
	for (int i = 0; i < initialvertices.size(); i++) {
		modifiedpositions.push_back(glm::vec3(0, 0, 0));
	}

	//loop through edges
	edgeshashtype::iterator iter = edges.begin();
	while (iter != edges.end()) {
		Edge* THEedge = iter->second;
		Triangle* THEtriangle = THEedge->getTriangle();

		//get the vertexes
		Vertex* vertex1 = THEedge->getStartVertex();
		Vertex* vertex2 = THEedge->getEndVertex();

		//if child has not been created already, create it!
		if (vertex_parents.find(std::make_pair(vertex1, vertex2)) == vertex_parents.end()) {
			Vertex* newvertex;

			//if this is a boundary edge, use 1/2 1/2
			if (THEedge->getOpposite() == NULL) {
				newvertex = addVertex((vertex1->getPos() + vertex2->getPos()) / 2.0f);
			}
			//else use 3/8 3/8 1/8 1/8
			else {
				Vertex* vertex3 = THEedge->getNext()->getEndVertex();
				Vertex* vertex4 = THEedge->getOpposite()->getNext()->getEndVertex();
				newvertex = addVertex((vertex1->getPos() * 0.375f) + (vertex2->getPos() * 0.375f)
					+ (vertex3->getPos() * 0.125f) + (vertex4->getPos() * 0.125f));
			}

			//create parent-child relationship
			setParentsChild(vertex1, vertex2, newvertex);
		}

		//calculate new position of initial vertex
		//get the surrounding vertexes
		std::vector<Vertex*> surroundingvertices;
		surroundingvertices.push_back(vertex2);
		while (THEedge != NULL) {
			THEedge = THEedge->getNext()->getNext();
			surroundingvertices.push_back(THEedge->getStartVertex());

			//break if reached boundary edge
			if (THEedge->getOpposite() != NULL) THEedge = THEedge->getOpposite();
			else break;

			//if it has looped around, the last one was a duplicate
			if (THEedge->getTriangle() == THEtriangle) { surroundingvertices.pop_back(); break; }
		}

		//if encountered a boundary, use different rule
		if (THEedge->getOpposite() == NULL) {
			Vertex* vertex3 = THEedge->getStartVertex();
			
			//go all the way around to the other side
			THEedge = THEedge->getNext();
			while (THEedge->getOpposite() != NULL) THEedge = THEedge->getOpposite()->getNext();

			Vertex* vertex4 = THEedge->getEndVertex();

			//calculate new position
			glm::vec3 newpos((vertex1->getPos() * 0.75f) + (vertex3->getPos() * 0.125f) + (vertex4->getPos() * 0.125f));
			modifiedpositions[vertex1->getIndex()] = newpos;
		}

		/*
		if (THEedge == NULL) {
			THEedge = iter->second;
			THEedge = THEedge->getOpposite();
			while (THEedge != NULL) {
				THEedge = THEedge->getNext();
				surroundingvertices.push_back(THEedge->getEndVertex());
				THEedge = THEedge->getOpposite();
			}
		}
		*/

		else {
			//subdivision rules
			int n = surroundingvertices.size();

			if (n > 3) {
				float beta = (float)3 / (float)(8 * n);
				glm::vec3 newpos(0, 0, 0);

				//calculate new position
				for (int j = 0; j < n; j++) {
					newpos += (surroundingvertices[j]->getPos() * beta);
				}
				newpos += (vertex1->getPos() * (1 - (n*beta)));
				modifiedpositions[vertex1->getIndex()] = newpos;
			}

			else if (n == 3) {
				float beta = (float)3 / (float)16;
				glm::vec3 newpos(0, 0, 0);
				for (int j = 0; j < n; j++) {
					newpos += (surroundingvertices[j]->getPos() * beta);
				}
				
				//calculate new position
				newpos += (vertex1->getPos() * (1 - (n*beta)));
				modifiedpositions[vertex1->getIndex()] = newpos;
			}

			else {
				modifiedpositions[vertex1->getIndex()] = vertex1->getPos();
			}
		}

		iter++;
  }

  //create vector of triangles to loop through
  std::vector<Triangle*> todo;
  for (triangleshashtype::iterator iter = triangles.begin();
	  iter != triangles.end(); iter++) {
	  Triangle *t = iter->second;
	  todo.push_back(t);
  }
  //loop through triangles
  for (int i = 0; i < todo.size(); i++) {
	  Triangle* THEtriangle = todo[i];

	  //get the edges
	  Edge* edge1 = THEtriangle->getEdge();
	  Edge* edge2 = edge1->getNext();
	  Edge* edge3 = edge2->getNext();

	  //get the vertexes
	  Vertex* vertex1 = edge1->getStartVertex();
	  Vertex* vertex2 = edge2->getStartVertex();
	  Vertex* vertex3 = edge3->getStartVertex();
	  Vertex* vertex4 = getChildVertex(vertex1, vertex2);
	  Vertex* vertex5 = getChildVertex(vertex2, vertex3);
	  Vertex* vertex6 = getChildVertex(vertex3, vertex1);

	  //delete the triangle
	  removeTriangle(THEtriangle);

	  //add the new triangles
	  //std::cout << "adding triangle 1\n";
	  addTriangle(vertex1, vertex4, vertex6);
	  //std::cout << "adding triangle 2\n";
	  addTriangle(vertex4, vertex2, vertex5);
	  //std::cout << "adding triangle 3\n";
	  addTriangle(vertex5, vertex3, vertex6);
	  //std::cout << "adding triangle 4\n";
	  addTriangle(vertex4, vertex5, vertex6);
  }

  //loop through the initial vertexes and modify their positions
  for (int i = 0; i < initialvertices.size(); i++) {
	  //initialvertices[i]->setPos(initialvertices[i]->getPos() * 2.0f);
	  initialvertices[i]->setPos(modifiedpositions[i]);
  }
}


// =================================================================
// SIMPLIFICATION
// =================================================================


void Mesh::Simplification(int target_tri_count) {
  // clear out any previous relationships between vertices
  vertex_parents.clear();

  printf ("Simplify the mesh! %d -> %d\n", numTriangles(), target_tri_count);

  // =====================================
  // ASSIGNMENT: complete this functionality
  // =====================================
  while (numTriangles() > target_tri_count) {
	  /*
	  //go to a random edge
	  int index = rand() % edges.size();
	  edgeshashtype::iterator iter = edges.begin();
	  for (int j = 0; j < index; j++) { iter++; }
	  Edge* THEedge = iter->second;
	  Edge* THEoppositeedge = THEedge->getOpposite();
	  */

	  //removeTriangle(THEedge->getTriangle()); continue;
	  
	  //go to the shortest edge
	  edgeshashtype::iterator iter = edges.begin();
	  Edge* THEedge = iter->second;
	  float shortestlength = THEedge->Length();
	  iter++;
	  while (iter != edges.end()) {
		  if (iter->second->Length() < shortestlength) {
			  THEedge = iter->second;
			  shortestlength = THEedge->Length();
		  }
		  iter++;
	  }
	  Edge* THEoppositeedge = THEedge->getOpposite();

	  //create a new vertex that is the average of the two vertexes of the edge
	  Vertex* vertexa = THEedge->getStartVertex();
	  Vertex* vertexb = THEedge->getEndVertex();
	  glm::vec3 newpos = (vertexa->getPos() + vertexb->getPos()) / 2.0f;
	  Vertex* vertexc = addVertex(newpos);

	  //get pointers to the next edges around these vertexes
	  Edge* THEnextedge = THEedge->getNext()->getOpposite();
	  Edge* THEnextedge2 = NULL;
	  if (THEoppositeedge != NULL) THEnextedge2 = THEoppositeedge->getNext()->getOpposite();

	  //remove the two triangles of this edge
	  removeTriangle(THEedge->getTriangle());
	  if (THEoppositeedge != NULL) removeTriangle(THEoppositeedge->getTriangle());

	  while (THEnextedge != NULL) {
		  //temp pointer to next edge
		  Edge* temp = THEnextedge->getNext()->getOpposite();

		  Triangle* t = THEnextedge->getTriangle();
		  Vertex* a = (*t)[0];
		  Vertex* b = (*t)[1];
		  Vertex* c = (*t)[2];

		  //try to figure out which vertex is the one we need
		  //then remove triangle and create new one, replacing this vertex with the new one
		  if (a == vertexb) {
			  removeTriangle(t);
			  addTriangle(vertexc, b, c);
		  }
		  else if (b == vertexb) {
			  removeTriangle(t);
			  addTriangle(a, vertexc, c);
		  }
		  else if (c == vertexb) {
			  removeTriangle(t);
			  addTriangle(a, b, vertexc);
		  }

		  THEnextedge = temp;
	  }
		
	  while (THEnextedge2 != NULL) {
		  //temp pointer to next edge
		  Edge* temp = THEnextedge2->getNext()->getOpposite();

		  Triangle* t = THEnextedge2->getTriangle();
		  Vertex* a = (*t)[0];
		  Vertex* b = (*t)[1];
		  Vertex* c = (*t)[2];

		  //try to figure out which vertex is the one we need
		  //then remove triangle and create new one, replacing this vertex with the new one
		  if (a == vertexa) {
			  removeTriangle(t);
			  addTriangle(vertexc, b, c);
		  }
		  else if (b == vertexa) {
			  removeTriangle(t);
			  addTriangle(a, vertexc, c);
		  }
		  else if (c == vertexa) {
			  removeTriangle(t);
			  addTriangle(a, b, vertexc);
		  }

		  THEnextedge2 = temp;
	  }
  }
}


// =================================================================

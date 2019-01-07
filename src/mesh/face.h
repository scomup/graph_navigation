#ifndef MESH_FACE_H
#define MESH_FACE_H

#include "src/mesh/half_edge.h"
#include "src/mesh/vertex.h"
#include <iostream>

namespace GraphNavigation
{
namespace Mesh
{

template<typename T> class Vertex;


template<typename T>
class Face
{
    public:
	  typedef Vertex<T> VertexT;
	  typedef boost::shared_ptr<VertexT> VertexPtr;

	  typedef HalfEdge<Vertex<T>, Face<T>> HEdge;
	  typedef boost::shared_ptr<HEdge> EdgePtr;

	  // A pointer to a surrounding half edge
	  EdgePtr edge_;

	  // A vector containing the indices of face vertices
	  size_t indices_[3];

	  size_t face_index_;

	  /// The face normal
	  T normal_;


	  void calc_normal()
	  {
		  T vertices[3];
		  
		  VertexPtr start = edge_->start();
		  EdgePtr current_edge = edge_;

		  int c = 0;
		  while (current_edge->end() != start)
		  {
			  vertices[c] = current_edge->start()->position_;
			  current_edge = current_edge->next();
			  c++;
		  }
		  T diff1 = vertices[0] - vertices[1];
		  T diff2 = vertices[0] - vertices[2];
		  normal_ = (diff1.cross(diff2));
		  normal_ = normal_/normal_.norm();
		  if(normal_.z() < 0){
			  normal_ = -normal_;
		  }
		  std::vector<VertexT> v;
	  }

	  void getVertices(std::vector<VertexT> &v)
	  {
		  VertexPtr start = edge_->start();
		  EdgePtr current_edge = edge_;
		  do
		  {
			  v.push_back(current_edge->end()->position_);
			  current_edge = current_edge->next();
		  } while (current_edge->end() != start);
		  v.push_back(current_edge->end()->position_);
	  }

	  

	  void getCentroid()
	  {
		  std::vector<VertexT> vert;
		  getVertices(vert);

		  VertexT centroid;

		  for (size_t i = 0; i < vert.size(); i++)
		  {
			  centroid += vert[i];
		  }

		  if (vert.size() > 0)
		  {
			  centroid.x = centroid.x / vert.size();
			  centroid.y = centroid.y / vert.size();
			  centroid.z = centroid.z / vert.size();
		  }
		  else
		  {
			  std::cout << "Warning: HalfEdgeFace::getCentroid: No vertices found." << std::endl;
			  return VertexT();
		  }

		  return centroid;
	  }
};

} // namespace Mesh
} // namespace GraphNavigation
#endif//MESH_FACE_H
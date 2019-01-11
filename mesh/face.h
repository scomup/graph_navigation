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
	  typedef Face<T> FaceT;
	  typedef VertexT* VertexTPtr;
	  typedef FaceT* FaceTPtr;
	  typedef HalfEdge<VertexT, FaceT> EdgeT;
	  typedef EdgeT* EdgeTPtr;

	  // A pointer to a surrounding half edge
	  EdgeTPtr edge_;

	  // A vector containing the indices of face vertices
	  size_t indices_[3];

	  size_t face_index_;

	  /// The face normal
	  T normal_;


	  void calc_normal()
	  {
		  T vertices[3];
		  
		  VertexTPtr start = edge_->start();
		  EdgeTPtr current_edge = edge_;

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
	  }

	  void getVertices(std::vector<T> &v)
	  {
		  VertexTPtr start = edge_->start();
		  EdgeTPtr current_edge = edge_;
		  do
		  {
			  v.push_back(current_edge->end()->position_);
			  current_edge = current_edge->next();
		  } while (current_edge->end() != start);
		  v.push_back(current_edge->end()->position_);
	  }

	  

	  T getCentroid()
	  {
		  std::vector<T> vert;
		  getVertices(vert);

		  T centroid(0,0,0);

		  for (size_t i = 0; i < vert.size(); i++)
		  {
			  centroid += vert[i];
		  }

		  if (vert.size() > 0)
		  {
			  centroid = centroid / vert.size();
		  }
		  else
		  {
			  std::cout << "Warning: HalfEdgeFace::getCentroid: No vertices found." << std::endl;
			  return T(0,0,0);
		  }

		  return T(0,0,0);
	  }

	  void getAdjacentFaces(std::vector<FaceTPtr> &adj)
	  {

		  EdgeTPtr current = edge_;

		  do
		  {
			  if (current->hasNeighborFace())
			  {
				  adj.push_back(current->pair()->face());
			  }
			  current = current->next();
		  } while (edge_ != current);
	  }
};

} // namespace Mesh
} // namespace GraphNavigation
#endif//MESH_FACE_H

#ifndef MESH_VERTEX_H
#define MESH_VERTEX_H

#include <vector>
#include <boost/shared_ptr.hpp>
#include "src/mesh/half_edge.h"
#include "src/mesh/face.h"


using namespace GraphNavigation::Mesh;

namespace GraphNavigation
{
namespace Mesh
{

template <typename T>
class Vertex
{
  public:
  	typedef HalfEdge< Vertex<T>, Face<T> > HEdge;
	typedef boost::shared_ptr<HEdge> EdgePtr;

	Vertex(T p){
		position_ = p;
	}

	// The vertex's position
	T position_;

	// The vertex's normal
	T normal_;

	// The list incoming edges
	std::vector<EdgePtr> in_;

	// The list of outgoing edges
	std::vector<EdgePtr> out_;

	// The vertex index in the mesh
	size_t index_;
};

} // namespace Mesh
} // namespace GraphNavigation
#endif //MESH_VERTEX_H
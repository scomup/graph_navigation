#ifndef MESH_HALF_EDGE_MESH_H
#define MESH_HALF_EDGE_MESH_H

#include <Eigen/Dense>
#include <stdlib.h>
#include "src/mesh/vertex.h"
#include "src/mesh/face.h"
#include "src/mesh/half_edge.h"

namespace GraphNavigation
{
namespace Mesh
{

template<typename T>
class  HalfEdgeMesh
{
    public:

	typedef Face<T> FaceT;
    typedef Vertex<T> VertexT;
    typedef HalfEdge<VertexT, FaceT> HEdgeT;
    typedef FaceT*    FaceTPtr;
    typedef VertexT*  VertexTPtr;
    typedef HEdgeT*   HEdgeTPtr;

	virtual void addVertex(T v);
	virtual void addNormal(T n);
	virtual void addTriangle(size_t a, size_t b, size_t c);

	HEdgeTPtr halfEdgeToVertex(VertexTPtr v, VertexTPtr next);


protected:
	/// The faces in the half edge mesh
	std::vector<FaceTPtr>  faces_;

	/// The vertices of the mesh
	std::vector<VertexTPtr>  vertices_;

	/// The indexed of the newest inserted vertex
	size_t  vertexs_size_;

};


} // namespace Mesh
} // namespace GraphNavigation
#endif//HALF_EDGE_H

#ifndef GRAPHHALFEDGEMESH_H_
#define GRAPHHALFEDGEMESH_H_

#include <vector>
#include <list>
#include <math.h>
#include <cstdlib>
#include <limits>

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/astar_search.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/properties.hpp>
#include "src/mesh/half_edge_mesh.h"


namespace GraphNavigation
{
namespace Mesh
{

/**
 * @brief A implementation of a half edge triangle mesh.
 */

  // specify some types
  typedef boost::adjacency_list<boost::listS, boost::vecS, boost::undirectedS, boost::no_property,
    boost::property<boost::edge_weight_t, float> > mygraph_t;
  typedef boost::property_map<mygraph_t, boost::edge_weight_t>::type WeightMap;
  typedef mygraph_t::vertex_descriptor vertex_descriptor;
  typedef mygraph_t::edge_descriptor edge_descriptor;
  typedef mygraph_t::vertex_iterator vertex_iterator;
  typedef std::pair<int, int> edge;

template <typename T>
class MeshMap : public HalfEdgeMesh<T>
{
public:
  typedef boost::shared_ptr<MeshMap<T>> Ptr;
  typedef Face<T> FaceT;
  typedef Vertex<T> VertexT;
  typedef HalfEdge<VertexT, FaceT> EdgeT;
  typedef boost::shared_ptr<EdgeT> EdgeTPtr;
  typedef boost::shared_ptr<FaceT> FaceTPtr;
  typedef boost::shared_ptr<VertexT> VertexTPtr;

  MeshMap();

  virtual void addVertex(T v);
  virtual void addTriangle(uint a, uint b, uint c);
  void make_graph();
private:
  mygraph_t graph_;
  size_t face_cnt_;
  size_t vertex_cnt_;

};

} // namespace Mesh
} // namespace GraphNavigation

#include "mesh_map.tcc"
#endif /* GRAPHHALFEDGEMESH_H_ */

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


// Vertex Costs
struct vertex_costs_t {
  typedef boost::vertex_property_tag kind;
};

// Edge distances - Euclidean vertex distances 
struct edge_distance_t {
  typedef boost::edge_property_tag kind;
};


typedef boost::adjacency_list_traits<boost::listS, boost::vecS, boost::undirectedS>::vertex_descriptor vertex_descriptor;

typedef boost::adjacency_list<
    boost::listS, boost::vecS, boost::undirectedS,
    // Vertex properties
    boost::property<vertex_costs_t, float>,
    // Edge properties
    boost::property<edge_distance_t, float>>
    Graph;

typedef boost::graph_traits<Graph>::out_edge_iterator out_edge_iterator;
typedef std::pair<out_edge_iterator, out_edge_iterator> out_edge_iterator_range;
typedef boost::graph_traits < Graph >::adjacency_iterator adjacency_iterator;


typedef boost::property_map<Graph, vertex_costs_t>::type VertexCostMap;
typedef boost::property_map<Graph, edge_distance_t>::type EdgeDistanceMap;


typedef Graph::vertex_descriptor GraphNode;
typedef Graph::edge_descriptor GraphEdge;
typedef Graph::vertex_iterator GraphNode_iterator;
typedef float CostType;

namespace GraphNavigation
{
namespace Mesh
{

/**
 * @brief A implementation of a half edge triangle mesh.
 */
template<typename T>
class MeshMap : public HalfEdgeMesh<T>
{
public:

  typedef boost::shared_ptr< MeshMap<T> > Ptr;
  typedef Face<T> HFace;
  typedef Vertex<T> HVertex;
  typedef HalfEdge< HVertex, HFace > HEdge;

  typedef boost::shared_ptr<HEdge> EdgePtr;
  typedef boost::shared_ptr<HFace> FacePtr;
  typedef boost::shared_ptr<HVertex> VertexPtr;

  MeshMap();
  
  virtual void addVertex(T v); 
  
  virtual void addTriangle(uint a, uint b, uint c);


private:

  Graph face_graph_, vertex_graph_;
  size_t face_cnt_;
  size_t vertex_cnt_;


};



}
}



#endif /* GRAPHHALFEDGEMESH_H_ */

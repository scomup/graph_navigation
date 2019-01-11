#ifndef MESH_MAP_H_
#define MESH_MAP_H_

#include <vector>
#include <list>
#include <math.h>
#include <cstdlib>
#include <limits>

#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>
#include <pcl/surface/gp3.h>


#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/astar_search.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/properties.hpp>
#include "src/mesh/half_edge_mesh.h"

namespace boost{
  enum vertex_weight_t{vertex_weight};
  BOOST_INSTALL_PROPERTY(vertex, weight);
}

namespace GraphNavigation
{
namespace Mesh
{

/**
 * @brief A implementation of a half edge triangle mesh.
 */

// specify some types
typedef boost::adjacency_list<boost::listS,
                              boost::vecS,
                              boost::undirectedS,
                              boost::property<boost::vertex_weight_t, float>,
                              boost::property<boost::edge_weight_t, float>>
    mesh_graph_t;

typedef boost::property_map<mesh_graph_t, boost::vertex_weight_t>::type vertex_weight_map_t; 
typedef boost::property_map<mesh_graph_t, boost::edge_weight_t>::type edge_weight_map_t;
typedef mesh_graph_t::vertex_descriptor vertex_descriptor;
typedef mesh_graph_t::edge_descriptor edge_descriptor;
typedef mesh_graph_t::vertex_iterator vertex_iterator;
typedef boost::property_map<mesh_graph_t, boost::vertex_index_t>::type index_map_t;

template <typename T>
class MeshMap : public HalfEdgeMesh<T>
{
public:
/*
  typedef boost::shared_ptr<MeshMap<T>> Ptr;
  typedef Face<T> FaceT;
  typedef Vertex<T> VertexT;
  typedef HalfEdge<VertexT, FaceT> EdgeT;
  typedef boost::shared_ptr<EdgeT> EdgeTPtr;
  typedef boost::shared_ptr<FaceT> FaceTPtr;
  typedef boost::shared_ptr<VertexT> VertexTPtr;*/

  MeshMap();
  MeshMap(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

  void make_graph();
  bool astar(uint start, uint goal, std::vector<int>& path);

  int  getRandVec(){
    int idx;
    do{
      idx = rand() % cloud_->points.size();
    } while(!traversability_[idx]);
    return idx;
  }
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
  std::vector<uint16_t> traversability_;
  
private:
  mesh_graph_t graph_;
  
  // euclidean distance heuristic
  class distance_heuristic : public boost::astar_heuristic<mesh_graph_t, float>
  {
  public:
    distance_heuristic(MeshMap<T> *mesh, index_map_t &idx, vertex_descriptor goal)
        : mesh_(mesh), indices_(idx) , goal_(goal)  {}
    float operator()(vertex_descriptor u)
    {
      auto goal_position = mesh_->vertices_[indices_[goal_]]->position_;
      auto cur_position = mesh_->vertices_[indices_[u]]->position_;
      return (goal_position - cur_position).norm();
    }

  private:
    MeshMap<T> *mesh_;
    index_map_t indices_;
    vertex_descriptor goal_;
  
  };

  struct found_goal
  {
  }; // exception for termination

  // visitor that terminates when we find the goal
  class astar_goal_visitor : public boost::default_astar_visitor
  {
  public:
    astar_goal_visitor(vertex_descriptor goal)
     : goal_(goal) {}
    void examine_vertex(vertex_descriptor u, const  mesh_graph_t &g)
    {
      if (u == goal_)
        throw found_goal();
    }

  private:
    vertex_descriptor goal_;
  };
};

} // namespace Mesh
} // namespace GraphNavigation

#include "mesh_map.tcc"
#endif /* GRAPHHALFEDGEMESH_H_ */

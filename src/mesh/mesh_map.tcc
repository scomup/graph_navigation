#include "src/mesh/mesh_map.h"
#include <iostream>

#include <boost/graph/visitors.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/breadth_first_search.hpp>
#include <boost/graph/graph_utility.hpp> 

template <typename T>
MeshMap<T>::MeshMap()
    : HalfEdgeMesh<T>(), face_cnt_(0), vertex_cnt_(0)
{
}

template <typename T>
void MeshMap<T>::make_graph(){
  mygraph_t g(100);
  WeightMap weightmap = boost::get(boost::edge_weight, g);
  auto faces = this->faces_;
  for (std::size_t j = 0; j < 10; ++j)
  {
    edge_descriptor e;
    bool inserted;
    int a = (j)%1;
    int b = (j+1)%1;
    std::cout<<"try form "<<a<<" to "<<b<<std::endl;
    if(!boost::edge(boost::vertex(a, g), boost::vertex(b, g), g).second){
        boost::tie(e, inserted) = add_edge(a, b, g);
        weightmap[e] = 1;
        std::cout<<"add form "<<a<<" to "<<b<<"!\n";
    }
  }

      if(!boost::edge(boost::vertex(1, g), boost::vertex(2, g), g).second){
        add_edge(1, 2, g);
        std::cout<<"add form "<<1<<" to "<<2<<"!\n";
      }

    if(!boost::edge(boost::vertex(2, g), boost::vertex(1, g), g).second){
        add_edge(2, 1, g);
        std::cout<<"add form "<<2<<" to "<<1<<"!\n";
    }else{
      std::cout<<"exist!\n";
    }




  //boost::print_graph(g);

}

template <typename T>
void MeshMap<T>::addVertex(T v)
{
  HalfEdgeMesh<T>::addVertex(v);

  //this->vertices_[vertex_cnt_]->index_ = vertex_cnt_;
  //GraphNode vertex_insert = boost::add_vertex(graph_);
  //vertex_cnt_++;
  
}


template <typename T>
void MeshMap<T>::addTriangle(uint a, uint b, uint c)
{
  const int face_index = face_cnt_;
  HalfEdgeMesh<T>::addTriangle(a, b, c);



/*

  // set face index for the graph relationship
  this->faces_[face_index]->face_index_ = face_index;
  
  std::pair<GraphEdge, bool> edge_insert;

  EdgeDistanceMap vertex_graph_distances = boost::get(edge_distance_t(), vertex_graph_);
  EdgeDistanceMap face_graph_distances = boost::get(edge_distance_t(), face_graph_);
  VertexCostMap face_graph_vertex_costmap = boost::get(vertex_costs_t(), face_graph_);

  edge_insert = boost::add_edge(a, b, vertex_graph_);
  if(edge_insert.second){
    vertex_graph_distances[edge_insert.first] = (this->vertices_[a]->position_ - this->vertices_[b]->position_).norm();
  }
  edge_insert = boost::add_edge(b, c, vertex_graph_);
  if(edge_insert.second){
    vertex_graph_distances[edge_insert.first] = (float)(this->vertices_[b]->position_ - this->vertices_[c]->position_).norm();
  }
  edge_insert = boost::add_edge(c, a, vertex_graph_);
  if(edge_insert.second){
    vertex_graph_distances[edge_insert.first] = (float)(this->vertices_[c]->position_ - this->vertices_[a]->position_).norm();
  }*/
}


//MeshMap<Eigen::Vector3d>MeshMap3D;



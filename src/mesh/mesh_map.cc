#include "src/mesh/mesh_map.h"
#include <iostream>

template <typename T>
MeshMap<T>::MeshMap()
    : HalfEdgeMesh<T>(), face_cnt_(0), vertex_cnt_(0)
{
}

template <typename T>
void MeshMap<T>::addVertex(T v)
{
  HalfEdgeMesh<T>::addVertex(v);

  this->vertices_[vertex_cnt_]->index_ = vertex_cnt_;
  GraphNode vertex_insert = boost::add_vertex(vertex_graph_);
  // init vertex costs to zero
  VertexCostMap vertex_graph_vertex_costmap = boost::get(vertex_costs_t(), vertex_graph_);
  vertex_graph_vertex_costmap[vertex_insert] = 0;
  if (vertex_insert != vertex_cnt_)
  {
    std::cerr << "difference in graph and mesh index: " << vertex_insert << ", " << vertex_cnt_ << std::endl;
  }
  vertex_cnt_++;
}


template <typename T>
void MeshMap<T>::addTriangle(uint a, uint b, uint c)
{
  const int face_index = face_cnt_;
  HalfEdgeMesh<T>::addTriangle(a, b, c);


  // set face index for the graph relationship
  this->m_faces[face_index]->m_face_index = face_index;
  
  std::pair<GraphEdge, bool> edge_insert;

  EdgeDistanceMap vertex_graph_distances = boost::get(edge_distance_t(), vertex_graph_);
  EdgeDistanceMap face_graph_distances = boost::get(edge_distance_t(), face_graph_);
  VertexCostMap face_graph_vertex_costmap = boost::get(vertex_costs_t(), face_graph_);

  edge_insert = boost::add_edge(a, b, vertex_graph_);
  if(edge_insert.second){
    vertex_graph_distances[edge_insert.first] = this->vertices_[a]->position_.distance(this->vertices_[b]->position_);
  }
  edge_insert = boost::add_edge(b, c, vertex_graph_);
  if(edge_insert.second){
    vertex_graph_distances[edge_insert.first] = this->vertices_[b]->position_.distance(this->vertices_[c]->position_);
  }
  edge_insert = boost::add_edge(c, a, vertex_graph_);
  if(edge_insert.second){
    vertex_graph_distances[edge_insert.first] = this->vertices_[c]->position_.distance(this->vertices_[a]->position_);
  }
     
  GraphNode vertex_insert = boost::add_vertex(face_graph_);
  // init costs to zero
  face_graph_vertex_costmap[vertex_insert] = 0;
  
  typename Face<T>::FaceVector neighbours;
  typename Face<T>::FaceVector::iterator n_iter;
  this->faces_[face_index]->getAdjacentFaces(neighbours);

  Vertex<T> center = this->faces_[face_index]->getCentroid();
  for(n_iter = neighbours.begin(); n_iter != neighbours.end(); ++n_iter){
    int neighbour_index = (*n_iter)->m_face_index;
    edge_insert = boost::add_edge(face_index, neighbour_index, face_graph_);
    if(edge_insert.second){
      face_graph_distances[edge_insert.first] = center.distance((*n_iter)->getCentroid());
    }
  }
  
  face_cnt_++;
}
/*

template<typename VertexT, typename NormalT>
void GraphHalfEdgeMesh<VertexT, NormalT>::getVertexCostsFaceGraph(std::vector<float>& costs){
  costs.clear();
  VertexCostMap face_graph_vertex_costmap = boost::get(vertex_costs_t(), face_graph);
  for(size_t i=0; i<face_cnt; i++){
    costs.push_back(face_graph_vertex_costmap[i]);
  }
}

template<typename VertexT, typename NormalT>
void GraphHalfEdgeMesh<VertexT, NormalT>::getVetrexCostsVertexGraph(std::vector<float>& costs){
  costs.clear();
  VertexCostMap vertex_graph_vertex_costmap = boost::get(vertex_costs_t(), vertex_graph);
  for(size_t i=0; i<vertex_cnt; i++){
    costs.push_back(vertex_graph_vertex_costmap[i]);
  }
}

template<typename VertexT, typename NormalT>
void GraphHalfEdgeMesh<VertexT, NormalT>::getDistancesFaceGraph(std::vector<float>& costs){
  costs.clear();
  VertexDistanceMap distance_map = boost::get(boost::vertex_distance, face_graph);
  for(size_t i=0; i<face_cnt; i++){
    costs.push_back(distance_map[i]);
  }
}

template<typename VertexT, typename NormalT>
void GraphHalfEdgeMesh<VertexT, NormalT>::getDistancesVertexGraph(std::vector<float>& costs){
  costs.clear();
  VertexDistanceMap distance_map = boost::get(boost::vertex_distance, vertex_graph);
  for(size_t i=0; i<vertex_cnt; i++){
    costs.push_back(distance_map[i]);
  }
}

template<typename VertexT, typename NormalT>
bool GraphHalfEdgeMesh<VertexT, NormalT>::faceGraphAStar(
  const VertexT& start,
  const VertexT& goal,
  std::vector<VertexT>& path,
  std::vector<NormalT>& path_normals)
{
    int start_index, goal_index;
    getNearestVertexIndexFaceGraph(start, start_index);
    getNearestVertexIndexFaceGraph(goal, goal_index);

    std::list<int> path_ids;
    if(faceGraphAStar(start_index, goal_index, path_ids)){
      path.clear();
            for(std::list<int>::iterator iter = path_ids.begin();
                iter != path_ids.end(); ++iter)
            {   
                path.push_back(this->m_faces[*iter]->getCentroid());
                path_normals.push_back(this->m_faces[*iter]->getFaceNormal());
            }
            return true;
        }
    else{
            return false;
    }
}*/


#include "src/mesh/mesh_map.h"
#include <iostream>

#include <boost/graph/visitors.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/breadth_first_search.hpp>
#include <boost/graph/graph_utility.hpp> 
#include <boost/graph/random.hpp>
#include <boost/random.hpp>

template <typename T>
MeshMap<T>::MeshMap(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
    : HalfEdgeMesh<T>(),cloud_(cloud),traversability_(cloud->size())
{    

    // Normal estimation*
    
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);
    n.setInputCloud(cloud);
    n.setSearchMethod(tree);
    n.setKSearch(20);
    n.compute(*normals);
    
    //* normals should not contain the point normals + surface curvatures
    // Concatenate the XYZ and normal fields*
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);

     
    //* cloud_with_normals = cloud + normals
    // Create search tree*
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud(cloud_with_normals);
    // Initialize objects
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
    pcl::PolygonMesh triangles;
    // Set the maximum distance between connected points (maximum edge length)
    gp3.setSearchRadius(0.1);
    // Set typical values for the parameters
    gp3.setMu(25);
    gp3.setMaximumNearestNeighbors(100);
    gp3.setMaximumSurfaceAngle(M_PI / 4); // 45 degrees
    gp3.setMinimumAngle(M_PI / 18);       // 10 degrees
    gp3.setMaximumAngle(2 * M_PI / 3);    // 120 degrees
    gp3.setNormalConsistency(false);
    // Get result
    gp3.setInputCloud(cloud_with_normals);
    gp3.setSearchMethod(tree2);
    gp3.reconstruct(triangles);
    
    

    for (auto &n : *normals)
    {
      if (n.normal[2] < 0)
      {
        n.normal[0] = -n.normal[0];
        n.normal[1] = -n.normal[1];
        n.normal[2] = -n.normal[2];
      }
    }
    
    for (size_t i = 0; i < cloud->size(); i++)
    {
        auto &point = cloud->points[i];
        auto &normal = normals->points[i].normal;
        this->addVertex(Eigen::Vector3d(point.x, point.y, point.z));
        //this->addNormal(Eigen::Vector3d(normal[0], normal[1], normal[2]));
        Eigen::Vector3d norm(normal[0],
                             normal[1],
                             normal[2]);

        Eigen::AngleAxisd angle_axis(Eigen::Quaterniond::FromTwoVectors(
            norm, Eigen::Vector3d::UnitZ()));
        if (angle_axis.angle() > M_PI / 6)
            traversability_[i] = false;
        else
            traversability_[i] = true;
    }

    for (::pcl::Vertices v : triangles.polygons)
    {
        if (v.vertices.size() != 3)
            continue;
        size_t a = v.vertices[0];
        size_t b = v.vertices[1];
        size_t c = v.vertices[2];
        this->addTriangle(a, b ,c);
    }
    //make_graph();
}

template <typename T>
MeshMap<T>::MeshMap()
    : HalfEdgeMesh<T>()
{

    traversability_.resize(18522800);
    printf("size : %d\n", traversability_.size());
}
template <typename T>
bool MeshMap<T>::astar(uint start_idx, uint goal_idx, std::vector<int> &path){

  vertex_descriptor start = boost::vertex(start_idx, graph_);
  vertex_descriptor goal = boost::vertex(goal_idx, graph_);

  std::vector<mesh_graph_t::vertex_descriptor> p(num_vertices(graph_));
  std::vector<float> d(num_vertices(graph_));
  vertex_weight_map_t vertex_weight = boost::get(boost::vertex_weight, graph_);
  index_map_t indices = boost::get(boost::vertex_index, graph_);

  try{
    distance_heuristic h(this, indices, goal);
    astar_goal_visitor v(goal);
    boost::astar_search(graph_, start, h,
                        boost::predecessor_map(&p[0])
                        .distance_map(&d[0])
                        .visitor(v));
  }
  catch (found_goal fg)
  {
    // found a path to the goal
    for (vertex_descriptor v = goal;; v = p[v])
    {
      path.push_back(v);
      if (p[v] == v)
        break;
    }
    return true;
  }
  return false;
}

template <typename T>
void MeshMap<T>::make_graph(){

  auto faces = this->getFaces();
  auto vertices = this->getVertices();
  edge_weight_map_t edge_weight = boost::get(boost::edge_weight, graph_);
  vertex_weight_map_t vertex_weight = boost::get(boost::vertex_weight, graph_);

  for (auto vec : vertices)
  {
    vertex_descriptor v = boost::add_vertex(graph_);
    vertex_weight[v] = 0;
  }
  auto v4 = boost::vertex(4, graph_);
  vertex_weight[v4] = 100;


  for (auto face : faces)
  {
    int a = face->indices_[0];
    int b = face->indices_[1];
    int c = face->indices_[2];
    auto va = boost::vertex(a, graph_);
    auto vb = boost::vertex(b, graph_);
    auto vc = boost::vertex(c, graph_);
    auto pa = vertices[a]->position_;
    auto pb = vertices[b]->position_;
    auto pc = vertices[c]->position_;
    
    edge_descriptor e;
    bool inserted;

    if(!boost::edge(va, vb, graph_).second && traversability_[a] && traversability_[b]){
        boost::tie(e, inserted) = add_edge(a, b, graph_);
        edge_weight[e] = (pa - pb).norm();
    }
    if(!boost::edge(vb, vc, graph_).second && traversability_[b] && traversability_[c]){
        boost::tie(e, inserted) = add_edge(b, c, graph_);
        edge_weight[e] = (pb - pc).norm();;
    }
    if (!boost::edge(vc, va, graph_).second && traversability_[c] && traversability_[a])
    {   boost::tie(e, inserted) = add_edge(c, a, graph_);
        edge_weight[e] = (pc - pa).norm();
    }
  }
  //boost::print_graph(graph_);
}



//MeshMap<Eigen::Vector3d>MeshMap3D;



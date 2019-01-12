/*#include <bits/stdc++.h>
#include "src/graph/dijkstra_graph.h"
#include "src/graph/astar_graph.h"
#include <set>
#include <Eigen/Dense>

int main()
{
    //GraphNavigation::Graph::DijkstraGraph<Eigen::Vector2d, float> g;
    GraphNavigation::Graph::AStarGraph<Eigen::Vector2d, float> g;
    g.AddNode(Eigen::Vector2d(0,0));
    g.AddNode(Eigen::Vector2d(0,1));
    g.AddNode(Eigen::Vector2d(0,2));
    g.AddNode(Eigen::Vector2d(1,0));
    g.AddNode(Eigen::Vector2d(1,1));
    g.AddNode(Eigen::Vector2d(1,2));
    g.AddNode(Eigen::Vector2d(2,0));
    g.AddNode(Eigen::Vector2d(2,1));
    g.AddNode(Eigen::Vector2d(2,2));
    g.AddEdge(0, 1);
    g.AddEdge(1, 2);
    g.AddEdge(0, 3);
    g.AddEdge(1, 4);
    g.AddEdge(2, 5);
    g.AddEdge(3, 4);
    g.AddEdge(4, 5);
    g.AddEdge(3, 6);
    g.AddEdge(4, 7);
    g.AddEdge(5, 8);
    g.AddEdge(6, 7);
    g.AddEdge(7, 8);
    g.AddEdge(7, 3);


    std::vector<uint> path;
    g.FindPath(0,8, path);

    for (int i = 0; i < path.size(); i++)
        std::cout << path[i] << " ";
    std::cout << std::endl;
}*/


#include <chrono>
#include <thread>
#include <memory>
#include <iostream>
#include <utility>
#include <boost/functional/hash.hpp>
#include <liblas/liblas.hpp>
#include <time.h>
#include "src/playable_bag.h"
#include "sensor_msgs/PointCloud2.h"
#include "viewer/viewer.h"
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>
#include <pcl/surface/gp3.h>

#include "rrt/BiRRT.h"
#include "rrt/CloudStateSpace.h"
#include "cloud_analyzer.h"
#include "viewer/cloud_analyzer_handle.h"
#include "viewer/planner_handle.h"

#include "src/graph/dijkstra_graph.h"
#include "src/graph/astar_graph.h"
#include <flann/algorithms/dist.h>
#include <flann/algorithms/kdtree_single_index.h>
#include <flann/flann.hpp>



using namespace GraphNavigation::Graph;

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &input)
{
    // ... do data processing
    pcl::PointCloud<pcl::PointXYZ>::Ptr local_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*input, *local_cloud);
    size_t j = 0;
    for (size_t i = 0; i < local_cloud->points.size(); ++i)
    {
        if (!pcl_isfinite(local_cloud->points[i].x) ||
            !pcl_isfinite(local_cloud->points[i].y) ||
            !pcl_isfinite(local_cloud->points[i].z))
            continue;
        local_cloud->points[j] = local_cloud->points[i];
        j++;
    }
    if (j != local_cloud->points.size())
    {
        // Resize to the correct size
        local_cloud->points.resize(j);
    }
    cloud = local_cloud;
    cloud->width = cloud->size();
}

int main(int argc, char **argv)
{/*
    int N = 10;
    int dim = 3;
    double targetData[N * dim];
    for (uint i = 0; i < N; ++i)
    {
        for (int j = 0; j < dim; ++j)
            targetData[i * dim + j] = 1;
    }
    flann::Matrix<double> dataset(targetData, N, dim);
    flann::Index<flann::L2<double>> index(dataset, flann::KDTreeIndexParams(8));
    index.buildIndex();
    double queryData[3];
    queryData[0] = 1;
    queryData[1] = 2;
    queryData[2] = 3;

    flann::Matrix<double> query(queryData, 1, 3);
    flann::Matrix<int> indices(new int[3], query.rows, 1);
    flann::Matrix<double> dists(new double[3], query.rows, 1);

    index.knnSearch(query, indices, dists, 1, flann::SearchParams(128));

    return 0;*/
    ros::init(argc, argv, "test_traversability");
    ros::NodeHandle nh;
    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe("/map3d", 1, cloud_cb);

    std::cout << "wait map3d topic\n";
    while (cloud == nullptr && ros::ok())
    {
        std::this_thread::sleep_for(std::chrono::microseconds(100));
        ros::spinOnce();
    }
    std::cout << "map3d loaded.\n";
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
    gp3.setMu(30);
    gp3.setMaximumNearestNeighbors(100);
    gp3.setMaximumSurfaceAngle(M_PI / 4); // 45 degrees
    gp3.setMinimumAngle(M_PI / 18);       // 10 degrees
    gp3.setMaximumAngle(2 * M_PI / 3);    // 120 degrees
    gp3.setNormalConsistency(false);
    // Get result
    gp3.setInputCloud(cloud_with_normals);
    gp3.setSearchMethod(tree2);
    gp3.reconstruct(triangles);

    std::function<size_t(Eigen::Vector3d)> f = [](Eigen::Vector3d state) {
             size_t seed = 0;
            boost::hash_combine(seed, state.x());
            boost::hash_combine(seed, state.y());
            boost::hash_combine(seed, state.z());
            return seed; };

    DijkstraGraph<Eigen::Vector3d, float> graph(
        [](Eigen::Vector3d state) {
             size_t seed = 0;
            boost::hash_combine(seed, state.x());
            boost::hash_combine(seed, state.y());
            boost::hash_combine(seed, state.z());
            return seed; });
    for (size_t i = 0; i < cloud->size(); i++)
    {
        auto &point = cloud->points[i];
        graph.AddNode(Eigen::Vector3d(point.x, point.y, point.z));
    }

    for (::pcl::Vertices v : triangles.polygons)
    {
        if (v.vertices.size() != 3)
            continue;
        size_t a = v.vertices[0];
        size_t b = v.vertices[1];
        size_t c = v.vertices[2];
        graph.AddEdge(a, b);
        graph.AddEdge(b, c);
        graph.AddEdge(c, a);
    }
    graph.initKDTree();
    double d;
    graph.FindNearest(Eigen::Vector3d(0,0,0),&d);
    std::cout <<d<< " graph was created!\n";
    auto viewer = new Viewer();
    auto viewer_thread = std::thread(&Viewer::Run, viewer);

    viewer->SetMesh(&graph);
    viewer_thread.join();
}


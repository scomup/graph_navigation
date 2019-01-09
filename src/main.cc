
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

#include "src/mesh/mesh_map.h"

using namespace GraphNavigation::Mesh;

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
{

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


    MeshMap<Eigen::Vector3d> mesh(cloud);

    std::cout<<"My mesh was created!\n";

/*
    MeshMap<Eigen::Vector3d> mesh;

    mesh.addVertex(Eigen::Vector3d(0, 0, 0));//0
    mesh.addVertex(Eigen::Vector3d(0, 1, 0));//1
    mesh.addVertex(Eigen::Vector3d(0, 2, 0));//2

    mesh.addVertex(Eigen::Vector3d(1, 0, 0));//3
    mesh.addVertex(Eigen::Vector3d(1, 1, 0));//4
    mesh.addVertex(Eigen::Vector3d(1, 2, 0));//5

    mesh.addVertex(Eigen::Vector3d(2, 0, 0));//6
    mesh.addVertex(Eigen::Vector3d(2, 1, 0));//7
    mesh.addVertex(Eigen::Vector3d(2, 2, 0));//8

    mesh.addTriangle(0, 3 ,4);
    mesh.addTriangle(0, 1 ,4);
    mesh.addTriangle(1, 2 ,5);
    mesh.addTriangle(1, 4 ,5);
    mesh.addTriangle(3, 4 ,7);
    mesh.addTriangle(3, 6 ,7);
    mesh.addTriangle(4, 5 ,8);
    mesh.addTriangle(4, 7 ,8);
    mesh.make_graph();*/

    auto viewer = new Viewer();
    auto viewer_thread = std::thread(&Viewer::Run, viewer);

    viewer->SetMesh(&mesh);
    viewer_thread.join();
}
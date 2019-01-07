#include <iostream>
#include <string>
#include <utility>
#include <vector>
#include <deque>
#include <boost/graph/adjacency_list.hpp>
//A*寻路算法
#include <boost/graph/astar_search.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/graphviz.hpp>
#include <boost/variant/get.hpp>

#include "src/mesh/half_edge_mesh.h"
#include <Eigen/Dense>

using namespace std;
using namespace boost;


int main()
{



    GraphNavigation::Mesh::HalfEdgeMesh<Eigen::Vector3d> mesh;
    mesh.addVertex(Eigen::Vector3d(0,0,0));
    mesh.addVertex(Eigen::Vector3d(0,1,0));
    mesh.addVertex(Eigen::Vector3d(1,1,0));




    //定义图的种类
    typedef adjacency_list<listS, vecS, directedS, no_property, property<edge_weight_t, double>> graph_t;
    //定义相关类型
    typedef graph_traits<graph_t>::vertex_descriptor vertex_desciptor;
    typedef graph_traits<graph_t>::edge_descriptor edge_descriptor;
    typedef pair<int, int> Edge;

    //定义结点和边的相关对象和属性
    enum nodes { A, B, C, D, E, F, G, H, N };
    char name[] = "ABCDEFGH";
    //创建边
    Edge edge_array[] = { Edge(A,B),Edge(A,C),Edge(B,D),Edge(B,E),Edge(C,E),
    Edge(C,F),Edge(F,G),Edge(G,H),Edge(E,H),Edge(D,E),Edge(D,H) };
    //定义边的权重
    double weights[] = { 5,1,1.3,3,10,2,1.2,0.5,1.3,0.4,6.3 };
    //边的数量
    int num_arcs = sizeof(edge_array) / sizeof(Edge);

    //生成被建模的图对象
    //     边数组头地址      边数组尾地址       定义权重     结点数量
    graph_t g(edge_array, edge_array + num_arcs, weights,       N);

    //p用于放置最短路径生成树的各个顶点的下一个节点
    std::vector<vertex_desciptor> p(num_vertices(g));
    //d用于放置从近到远的路径距离
    std::vector<double> d(num_vertices(g));
    //待求最短路径的源顶点
    vertex_desciptor s = vertex(A, g);

    //对图g的A顶点(s为它的描述器,即从哪个点开始)应用dijkstra算法
    //作为结果的距离矢量保存在d数组中
    //最短路径树上的父节点保存在p数组中
    dijkstra_shortest_paths(g, s, predecessor_map(&p[0]).distance_map(&d[0]));
    std::cout << "最短路径延时(ms)" << "\t最短路径树的父节点:" << std::endl;
    //输出结果到屏幕
    graph_traits <graph_t>::vertex_iterator vi, vend;
    for (tie(vi, vend) = vertices(g); vi != vend; vi++)
    {
        std::cout << "路径距离(" << name[*vi] << ") = " << d[*vi] << ",\t";
        std::cout << "父节点(" << name[*vi] << ") = " << name[p[*vi]] << endl;
    }
    cout << endl;
    std::ofstream file("test.dot");

    boost::write_graphviz(file, g,
            boost::make_label_writer(name));



    system("pause");
}

/*

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
#include "rrt/BiRRT.h"
#include "rrt/CloudStateSpace.h"
#include "cloud_analyzer.h"
#include "viewer/cloud_analyzer_handle.h"
#include "viewer/planner_handle.h"
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>


using namespace sample_carto;


void ReadLas (const std::string &file_name, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    std::ifstream ifs(file_name.c_str(), std::ios::in | std::ios::binary);

    if(ifs.fail()) 
	{
        std::cerr << "ERROR : Impossible to open the file : " << file_name <<std::endl;
        abort();
    }


    liblas::ReaderFactory f;
    liblas::Reader reader = f.CreateWithStream(ifs);
    unsigned long int nbPoints=reader.GetHeader().GetPointRecordsCount();
    std::cout<<"Cloud size:"<<nbPoints<<std::endl;

	// Fill in the cloud data
	cloud->width    = nbPoints;				// This means that the point cloud is "unorganized"
	cloud->height   = 1;						// (i.e. not a depth map)
	cloud->is_dense = false;
	cloud->points.resize (cloud->width * cloud->height);


	int i=0;				// counter

    double scale = 10;
	//double bias_x = (reader.GetPoint().GetX())/scale;
	//double bias_y = (reader.GetPoint().GetY())/scale;
	//double bias_z = (reader.GetPoint().GetZ())/scale;					

    Eigen::Vector3d xyz_centroid(0,0,0);
	while(reader.ReadNextPoint()) 
	{
		// get XYZ information
		cloud->points[i].x = (reader.GetPoint().GetX())/scale;
	    cloud->points[i].y = (reader.GetPoint().GetY())/scale;
	    cloud->points[i].z = (reader.GetPoint().GetZ())/scale;
        xyz_centroid.x() += cloud->points[i].x;
        xyz_centroid.y() += cloud->points[i].y;	
        xyz_centroid.z() += cloud->points[i].z;	



		i++; // ...moving on
	}
    xyz_centroid = xyz_centroid/cloud->points.size();

    //Eigen::Vector4f xyz_centroid;
    //pcl::compute3DCentroid(*cloud, xyz_centroid); //重心を計算

    for (size_t i = 0; i < cloud->points.size(); i++)
    {
        cloud->points[i].x = cloud->points[i].x - xyz_centroid[0];
        cloud->points[i].y = cloud->points[i].y - xyz_centroid[1];
        cloud->points[i].z = cloud->points[i].z - xyz_centroid[2];
    }
}

pcl::PointCloud<pcl::PointXYZ>::Ptr gcloud;

void 
cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input)
{
  // ... do data processing
  pcl::PointCloud<pcl::PointXYZ>::Ptr local_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg (*input, *local_cloud);  

  gcloud = local_cloud;
}


int main(int argc, char **argv)
{
    
 ros::init (argc, argv, "test_traversability");
  ros::NodeHandle nh;
  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/map3d", 1, cloud_cb);

  std::cout<<"wait map3d topic\n";
  while (gcloud == nullptr && ros::ok())
  {
      std::this_thread::sleep_for(std::chrono::microseconds(100));
      ros::spinOnce();
  }
  std::cout<<"map3d loaded.\n";


  pcl::io::savePCDFileASCII("/home/liu/workspace/traversability_detection/build/f.pcd", *gcloud);
  pcl::io::loadPCDFile("/home/liu/workspace/traversability_detection/build/f.pcd", *gcloud);

  std::vector< int > idx;
  pcl::removeNaNFromPointCloud(*gcloud, *gcloud, idx); 
  

  auto viewer = new Viewer();
  auto viewer_thread = std::thread(&Viewer::Run, viewer);



  // Normal estimation*
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (gcloud);
  n.setInputCloud (gcloud);
  n.setSearchMethod (tree);
  n.setKSearch (20);
  n.compute (*normals);
  //* normals should not contain the point normals + surface curvatures

  // Concatenate the XYZ and normal fields*
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
  pcl::concatenateFields (*gcloud, *normals, *cloud_with_normals);
  //* cloud_with_normals = cloud + normals

  // Create search tree*
  pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
  tree2->setInputCloud (cloud_with_normals);

  // Initialize objects
  pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
  pcl::PolygonMesh triangles;

  // Set the maximum distance between connected points (maximum edge length)
  gp3.setSearchRadius (0.1);

  // Set typical values for the parameters
  gp3.setMu (25);
  gp3.setMaximumNearestNeighbors (100);
  gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
  gp3.setMinimumAngle(M_PI/18); // 10 degrees
  gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
  gp3.setNormalConsistency(false);

  // Get result
  gp3.setInputCloud (cloud_with_normals);
  gp3.setSearchMethod (tree2);
  gp3.reconstruct (triangles);

  // Additional vertex information
  std::vector<int> parts = gp3.getPartIDs();
  std::vector<int> states = gp3.getPointStates();

  //viewer->SetRRTHandler(planner_handle);

  viewer->SetMesh(triangles);
  //auto cloud_analyzer = std::make_shared<CloudAnalyzer<pcl::PointXYZ>>(gcloud);
  //auto cloud_analyzer_handle = std::make_shared<CloudAnalyzerHandle>(cloud_analyzer);
  //viewer->SetCloudDrawer(cloud_analyzer_handle);
  //auto state_space = std::make_shared<RRT::CloudStateSpace>(cloud_analyzer);
  //auto planner_handle = std::make_shared<PlannerHandle>(state_space);
  //viewer->SetRRTHandler(planner_handle);

  viewer_thread.join();
    
}

*/
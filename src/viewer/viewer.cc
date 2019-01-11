/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include "viewer.h"
#include <pangolin/pangolin.h>
#include "myhandler.h"

#include "draw_object_helper.h"
#include "frustum_culling.h"

#include "Eigen/Core"
#include "Eigen/Geometry"

#include <iostream>
#include <chrono>
#include <thread>

Viewer::Viewer()
{
    t_ = 1e3 / 30;
    image_width_ = 640;
    image_height_ = 480;
    view_point_x_ = 20;
    view_point_y_ = 20;
    view_point_z_ = 10;
    view_point_f_ = 2000;

    pose_ = Eigen::Matrix4d::Identity();
}

void Viewer::Run()
{
    finish_ = false;

    pangolin::CreateWindowAndBind("rrt", 1024, 768);

    pangolin::CreatePanel("menu").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(175));
    pangolin::Var<bool> menuOriginalMap("menu.show original map", false, true);

    // Define Camera Render Object (for view / scene browsing)
    pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(1024, 768, view_point_f_, view_point_f_, 512, 389, 0.1, 1000),
        pangolin::ModelViewLookAt(view_point_x_, view_point_y_, view_point_z_, 0, 0, 0, 0.1, 0.0, 1.0));

    // Add named OpenGL viewport to window and provide 3D Handler
    auto handler = new pangolin::MyHandler3D(s_cam);
    pangolin::View &d_cam = pangolin::CreateDisplay()
                                .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
                                .SetHandler(handler);

    //glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_COLOR_MATERIAL);

    GLfloat lightColor0[] = {1.0f, 1.0f, 1.0f, 1.0f};
    GLfloat lightPos0[] = {0.0f, 0.0f, 100.0f, 1.0f};
    glLightfv(GL_LIGHT0, GL_DIFFUSE, lightColor0);
    glLightfv(GL_LIGHT0, GL_POSITION, lightPos0);

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    while (!pangolin::ShouldQuit() && !finish_)
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        d_cam.Activate(s_cam);
        pangolin::glDrawAxis(1);
        
        


        //DrawGrid(200,1);
        /*
        if (cloud_analyzer_handle_ != nullptr)
        {
            if (menuOriginalMap == true)
            {
                cloud_analyzer_handle_->DrawPoint(0);
            }
            else
            {
                cloud_analyzer_handle_->DrawPoint(1);
            }
            //cloud_analyzer_handle_->DrawObj(pose_);
        }

        if (handler_for_rrt_ != nullptr)
        {
            if (handler->getSign())
            {
                handler_for_rrt_->run();
            }

            handler_for_rrt_->setStart(*handler->getStart());
            handler_for_rrt_->setGoal(*handler->getGoal());
            handler_for_rrt_->DrawStart();
            handler_for_rrt_->DrawGoal();
            handler_for_rrt_->DrawPath();
        }
        */

        /*
        pcl::PointCloud<pcl::PointXYZ>::Ptr vertices( new pcl::PointCloud<pcl::PointXYZ> );
        pcl::fromPCLPointCloud2( mesh_.cloud, *vertices ); 
        for (::pcl::Vertices v : mesh_.polygons)
        {

            pangolin::glColorHSV(std::abs(vertices->points[v.vertices[0]].z * 100));
            glBegin(GL_TRIANGLES);
            for (size_t i = 0; i < v.vertices.size(); ++i)
            {
                int idx = v.vertices[i];
                pcl::PointXYZ v = vertices->points[idx];
                float x = v.x;
                float y = v.y;
                float z = v.z;
                glVertex3f(x, y, z);
                //std::cout<<x<<std::endl;
                //std::cout<<y<<std::endl;
                //std::cout<<z<<std::endl;
            }
            glEnd();
            
       }
       */

      
        glColor3f(0.0f, 0.0f, 0.0f);
        glPointSize(10);

        const auto& positions = graph_->positions();

        if (handler->getSign())
        {
            path_.clear();
            s_ = rand()%positions.size();
            g_ = rand()%positions.size();
            graph_->FindPath(s_, g_, path_);
        }

        glBegin(GL_POINTS);
        auto start = positions[s_];
        glVertex3f(start.x(), start.y(), start.z());
        glEnd();

        glBegin(GL_POINTS);
        auto goal = positions[g_];
        glVertex3f(goal.x(), goal.y(), goal.z());
        glEnd();
        glColor3f(0.0f, 0.0f, 0.0f);
        glBegin(GL_LINE_STRIP);
        for (auto p : path_)
        {
            auto point = positions[p];
            glVertex3f(point.x(), point.y(), point.z());
        }
        glEnd();

/*
        for (auto face : faces)
        {
            auto n = face->normal_;
            auto a = vertices[face->indices_[0]]->position_;
            auto b = vertices[face->indices_[1]]->position_;
            auto c = vertices[face->indices_[2]]->position_;
            glBegin(GL_TRIANGLES);
            pangolin::glColorHSV(std::abs(a.z() * 100));
            glNormal3f(n.x(), n.y(), n.z());
            glVertex3f(a.x(), a.y(), a.z());
            glVertex3f(b.x(), b.y(), b.z());
            glVertex3f(c.x(), c.y(), c.z());
            glEnd();
        }
  */     
 /*
        const auto& graph = graph_->graph();
        for (uint u = 0 ; u < graph.size(); u++)
        {
            for(auto& e: graph[u]){
                uint v = e.second;
                auto uu = positions[u];
                auto vv = positions[v];
                pangolin::glColorHSV(std::abs((uu.z()+vv.z()) * 50));
                glBegin(GL_LINES);
                glVertex3f(uu.x(), uu.y(), uu.z());
                glVertex3f(vv.x(), vv.y(), vv.z());
                glEnd();
            }
        }*/
        
       glPointSize(1);
        for (uint i = 0; i < positions.size(); i++)
        {
            auto a = positions[i];
            pangolin::glColorHSV(std::abs(a.z() * 100));
            glBegin(GL_POINTS);
            glVertex3f(a.x(), a.y(), a.z());
            glEnd();
        }
        
        pangolin::FinishFrame();
    }
    SetFinish();
}

void Viewer::SetFinish()
{
    finish_ = true;
}

void Viewer::SetCloudDrawer(std::shared_ptr<CloudAnalyzerHandle> cloud_analyzer_handle)
{
    cloud_analyzer_handle_ = cloud_analyzer_handle;
}

void Viewer::SetMesh(NaviGraph<Eigen::Vector3d, float>* mesh)
{
    graph_ = mesh;
    graph_->FindPath(s_,g_,path_);
}

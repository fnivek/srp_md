//
// @file progress/PoseToSceneGraph.cpp
// @brief convert from object poses to scene graph
// @author Zhen Zeng
// University of Michigan, 2017
//

#pragma once
#include <iostream>
#include <fstream>
#include <iterator>
#include <cmath>

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/imgproc/imgproc.hpp>

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>

#include "srp_md/scene_graph.h"
#include "srp_md_msgs/PoseToSceneGraph.h"

class PoseToSceneGraph
{
  public:
    PoseToSceneGraph();
    ~PoseToSceneGraph()
    {
    }

    void WriteSceneGraph(std::string file_path);

    scene_graph::SceneGraph get_scene_graph()
    {
        return scene_graph_;
    }

  private:
    ros::ServiceServer server_;
    scene_graph::SceneGraph scene_graph_;
    std::vector<scene_graph::Object> clear_objects_;

    bool CalcSceneGraph(srp_md_msgs::PoseToSceneGraph::Request& req, srp_md_msgs::PoseToSceneGraph::Response& resp);

    float CalcMinAngle(Eigen::Vector3f v1, Eigen::Vector3f v2);

    int GetGravitationalAxis(Eigen::Matrix4f transform, float& angle);

    std::vector<Eigen::Vector3f> ProjectObjectBoudingBox(scene_graph::Object object, std::string surface);

    void DrawPolygon(cv::Mat& image, std::vector<Eigen::Vector3f> points);

    bool CheckOverlap(scene_graph::Object object1, scene_graph::Object object2);
};

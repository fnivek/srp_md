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
#include <set>
#include <map>
#include <algorithm>
#include <iterator>

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

    void SetProximityThreshold(float threshold) {proximity_threshold_ = threshold;};

  private:
    ros::ServiceServer server_;
    scene_graph::SceneGraph scene_graph_;
    std::vector<scene_graph::Object> clear_objects_;
    float proximity_threshold_;

    bool CalcSceneGraph(srp_md_msgs::PoseToSceneGraph::Request& req, srp_md_msgs::PoseToSceneGraph::Response& resp);

    float CalcMinAngle(Eigen::Vector3d v1, Eigen::Vector3d v2);

    int GetGravitationalAxis(Eigen::Affine3d transform, float& angle);

    std::vector<Eigen::Vector3d> ProjectObjectBoudingBox(scene_graph::Object object, std::string surface);

    void DrawPolygon(cv::Mat& image, std::vector<Eigen::Vector3d> points);

    bool CheckOverlap(scene_graph::Object object1, scene_graph::Object object2);
    bool CheckCenterOfMassOn(scene_graph::Object top_obj, scene_graph::Object bot_obj);
    bool CheckProximity(scene_graph::Object object1, scene_graph::Object object2);

    void Get2DConvexHull(scene_graph::Object obj, std::vector<cv::Point2f>* hull);
};

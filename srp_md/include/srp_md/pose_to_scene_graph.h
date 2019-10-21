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
#include "srp_md/object.h"

class PoseToSceneGraph
{
  public:
    PoseToSceneGraph(std::string file_path) : pos_file_(file_path), tray_id_(-1)
    {
    }
    ~PoseToSceneGraph()
    {
    }

    void CalcSceneGraph();

    void WriteSceneGraph(std::string file_path)
    {
        std::ofstream fout;
        fout.open(file_path);

        for (size_t i = 0; i < scene_graph_.relList.size(); i++)
            fout << "on " << scene_graph_.relList[i].name1 << " " << scene_graph_.relList[i].name2 << "\n";

        for (const auto& object : clear_objects_)
            fout << "clear " << object.name_ << "\n";

        fout << std::endl;
    }

    renderer::SceneGraph get_scene_graph()
    {
        return scene_graph_;
    }

  private:
    std::string pos_file_;
    renderer::SceneGraph scene_graph_;
    std::vector<renderer::Object> clear_objects_;
    int tray_id_;

    float CalcMinAngle(Eigen::Vector3f v1, Eigen::Vector3f v2);

    int GetGravitationalAxis(Eigen::Matrix4f transform, float& angle);

    std::vector<Eigen::Vector3f> ProjectObjectBoudingBox(renderer::Object object, std::string surface);

    void DrawPolygon(cv::Mat& image, std::vector<Eigen::Vector3f> points);

    bool CheckOverlap(renderer::Object object1, renderer::Object object2);
};

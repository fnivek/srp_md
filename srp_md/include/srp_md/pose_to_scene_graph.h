//
// @file progress/poseToSceneGraph.cpp
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

class poseToSceneGraph
{
  public:
    poseToSceneGraph(std::string file_path) : pos_file(file_path), tray_id(-1)
    {
    }
    ~poseToSceneGraph()
    {
    }

    void calcSceneGraph();

    void writeSceneGraph(std::string file_path)
    {
        std::ofstream fout;
        fout.open(file_path);

        for (size_t i = 0; i < scene_graph.relList.size(); i++)
            fout << "on " << scene_graph.relList[i].name1 << " " << scene_graph.relList[i].name2 << "\n";

        for (const auto& object : clear_objects)
            fout << "clear " << object.name_ << "\n";

        fout << std::endl;
    }

    renderer::SceneGraph getSceneGraph()
    {
        return scene_graph;
    }

  private:
    std::string pos_file;
    renderer::SceneGraph scene_graph;
    std::vector<renderer::Object> clear_objects;
    int tray_id;

    float calcMinAngle(Eigen::Vector3f v1, Eigen::Vector3f v2);

    int getGravitationalAxis(Eigen::Matrix4f transform, float& angle);

    std::vector<Eigen::Vector3f> projectObjectBoudingBox(renderer::Object object, std::string surface);

    void drawPolygon(cv::Mat& image, std::vector<Eigen::Vector3f> points);

    bool checkOverlap(renderer::Object object1, renderer::Object object2);
};

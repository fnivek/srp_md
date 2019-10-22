//
// @file progress/PoseToSceneGraph.cpp
// @brief convert from object poses to scene graph
// @author Zhen Zeng
// University of Michigan, 2017
//

#include "srp_md/pose_to_scene_graph.h"

// Helper functions not needed elsewhere
bool ObjectCompByHeight(const scene_graph::Object& s1, const scene_graph::Object& s2)
{
    return s1.pose.pos_[2] > s2.pose.pos_[2];
}

bool PointCompByHeight(const Eigen::Vector4f& p1, const Eigen::Vector4f& p2)
{
    return p1[2] < p2[2];
}

// PoseToSceneGraph
PoseToSceneGraph::PoseToSceneGraph() : tray_id_(-1)
{
    // Start ros service
    ros::NodeHandle nh;
    server_ = nh.advertiseService("pose_to_scene_graph", &PoseToSceneGraph::CalcSceneGraph, this);
}

void PoseToSceneGraph::WriteSceneGraph(std::string file_path)
{
    std::ofstream fout;
    fout.open(file_path);

    for (size_t i = 0; i < scene_graph_.rel_list.size(); i++)
        fout << "on " << scene_graph_.rel_list[i].name1 << " " << scene_graph_.rel_list[i].name2 << "\n";

    for (const auto& object : clear_objects_)
        fout << "clear " << object.name << "\n";

    fout << std::endl;
}

bool PoseToSceneGraph::CalcSceneGraph(srp_md_msgs::PoseToSceneGraph::Request& req,
                                      srp_md_msgs::PoseToSceneGraph::Response& resp)
{
    float table_height = 0.76;
    int object_id = 0;

    std::string object_folder_path = "/home/logan/Documents/betty_models/light/";

    scene_graph::ObjectList object_list;

    // while (std::getline(infile, line))
    // {
    //     std::size_t colon = line.find(":");

    //     if (colon != std::string::npos)
    //     {
    //         std::istringstream iss(line);
    //         std::vector<std::string> tokens;
    //         std::copy(std::istream_iterator<std::string>(iss), std::istream_iterator<std::string>(),
    //                   std::back_inserter(tokens));

    //         scene_graph::Object object;
    //         object.name = tokens[0];
    //         object.name = object.name.substr(0, std::strlen(object.name.c_str()) - 1);
    //         object.pose.pos_ = glm::vec3(std::stof(tokens[1]), std::stof(tokens[2]), std::stof(tokens[3]));
    //         object.pose.euler_ = glm::vec3(std::stof(tokens[4]), std::stof(tokens[5]), std::stof(tokens[6]));
    //         object.id = object_id++;

    //         if (object.name == "tray")
    //             tray_id_ = object.id;

    //         // read in object dimensions
    //         std::string obj_dim_file = object_folder_path + "/" + object.name + ".txt";
    //         std::ifstream dim_reader(obj_dim_file);
    //         dim_reader >> object.dim[0];  // x
    //         dim_reader >> object.dim[1];  // y
    //         dim_reader >> object.dim[2];  // z

    //         object_list.push_back(object);

    //         if (object.name != "tray")
    //             clear_objects_.push_back(object);

    //         std::cout << object.name << ": " << object.pose;
    //         printf("object dim: %f %f %f\n", object.dim[0], object.dim[1], object.dim[2]);
    //     }
    // }

    scene_graph_.object_list = object_list;
    assert(tray_id_ != -1);

    // determine objects being supported
    scene_graph::ObjectList supported_objects;
    for (int i = object_list.size() - 1; i >= 0; i--)
    {
        if (object_list[i].name == "tray")
        {
            object_list.erase(object_list.begin() + i);
            continue;
        }

        // determine object axis that is aligned with gravitational axis
        float angle;
        int axis_ind = GetGravitationalAxis(object_list[i].pose.TransformationFromPose(), angle);
        printf("%s axis %d aligns with grav axis, angle = %f\n", object_list[i].name.c_str(), axis_ind, angle);

        // determine whether object is being supported
        float occupied_height = object_list[i].dim[axis_ind] * cos(angle);
        float threshold = 0.01;

        if (object_list[i].pose.pos_[2] - occupied_height / 2.0 > table_height + threshold)
        {
            printf("%s is being supported by other objects\n", object_list[i].name.c_str());

            supported_objects.push_back(object_list[i]);
            object_list.erase(object_list.begin() + i);
        }
        else
        {
            // add relation on(obj, table) to scene graph
            // NOTE: not distinguishing between table and tray, because the planner needs to be able to put multiple
            // objects onto tray
            scene_graph::Relation rel(scene_graph::RelationType::kOn, object_list[i].id, tray_id_, object_list[i].name,
                                      "tra"
                                      "y");
            scene_graph_.rel_list.push_back(rel);
        }

        std::cout << std::endl;
    }

    // sort in descending order for z
    std::sort(supported_objects.begin(), supported_objects.end(), ObjectCompByHeight);
    std::cout << "all objects being supported : ";
    for (const auto& object : supported_objects)
        printf("%s ", object.name.c_str());
    std::cout << std::endl;
    std::cout << "all rest objects : ";
    for (const auto& object : object_list)
        printf("%s ", object.name.c_str());
    std::cout << std::endl;

    // check supporting relationship
    for (int i = supported_objects.size() - 1; i >= 0; i--)
    {
        int supporting_object_index = -1;
        float min_dist = INFINITY;
        for (int j = 0; j < object_list.size(); j++)
        {
            // find overlap between objects if any
            if (CheckOverlap(supported_objects[i], object_list[j]))
            {
                // calculate distances between projected center
                float dist = (Eigen::Vector2f(supported_objects[i].pose.pos_[0], supported_objects[i].pose.pos_[1]) -
                              Eigen::Vector2f(object_list[j].pose.pos_[0], object_list[j].pose.pos_[1]))
                                 .norm();
                printf("2d dist = %f\n", dist);

                if (dist < min_dist)
                {
                    min_dist = dist;
                    supporting_object_index = j;
                }
            }
        }

        // move the supported object to rest objects lists
        object_list.push_back(supported_objects[i]);
        supported_objects.erase(supported_objects.begin() + i);

        if (supporting_object_index != -1)
        {
            printf("on(%s, %s)\n", object_list.back().name.c_str(), object_list[supporting_object_index].name.c_str());

            for (int j = 0; j < clear_objects_.size(); j++)
            {
                if (clear_objects_[j].id == object_list[supporting_object_index].id)
                {
                    clear_objects_.erase(clear_objects_.begin() + j);
                    break;
                }
            }

            // add relation to scene graph
            scene_graph::Relation rel(scene_graph::RelationType::kOn, object_list.back().id,
                                      object_list[supporting_object_index].id, object_list.back().name,
                                      object_list[supporting_object_index].name);
            scene_graph_.rel_list.push_back(rel);
        }
    }
    return true;
}

float PoseToSceneGraph::CalcMinAngle(Eigen::Vector3f v1, Eigen::Vector3f v2)
{
    float cos_angle = v1.dot(v2) / (v1.norm() * v2.norm());
    float angle = acos(cos_angle);

    std::cout << "angle = " << angle * 180.0 / M_PI;
    if (angle > M_PI / 2.0)
    {
        angle = M_PI - angle;
        std::cout << " -> " << angle * 180.0 / M_PI;
    }

    std::cout << std::endl;

    return angle;
}

int PoseToSceneGraph::GetGravitationalAxis(Eigen::Matrix4f transform, float& angle)
{
    // axis ind: 0, 1, 2 corresponds to x, y, z axis
    // gravitational axis is (0, 0, 1) in world frame
    Eigen::Vector4f origin(0, 0, 0, 1);
    origin = transform * origin;

    // x-axis
    Eigen::Vector4f x_axis(1, 0, 0, 1);
    x_axis = transform * x_axis;
    x_axis = x_axis - origin;

    // y-axis
    Eigen::Vector4f y_axis(0, 1, 0, 1);
    y_axis = transform * y_axis;
    y_axis = y_axis - origin;

    // z-axis
    Eigen::Vector4f z_axis(0, 0, 1, 1);
    z_axis = transform * z_axis;
    z_axis = z_axis - origin;

    // compare with gravitational axis
    Eigen::Vector3f grav_axis(0, 0, 1);
    Eigen::Vector3f x_axis3(x_axis[0], x_axis[1], x_axis[2]);
    Eigen::Vector3f y_axis3(y_axis[0], y_axis[1], y_axis[2]);
    Eigen::Vector3f z_axis3(z_axis[0], z_axis[1], z_axis[2]);

    float angle_x = CalcMinAngle(x_axis3, grav_axis);
    float angle_y = CalcMinAngle(y_axis3, grav_axis);
    float angle_z = CalcMinAngle(z_axis3, grav_axis);

    if ((angle_x <= angle_y) && (angle_x <= angle_z))
    {
        angle = angle_x;
        return 0;
    }
    else if ((angle_y <= angle_x) && (angle_y <= angle_z))
    {
        angle = angle_y;
        return 1;
    }
    else if ((angle_z <= angle_y) && (angle_z <= angle_x))
    {
        angle = angle_z;
        return 2;
    }
    else
    {
        std::cerr << "should not get there\n";
        exit(1);
    }
}

std::vector<Eigen::Vector3f> PoseToSceneGraph::ProjectObjectBoudingBox(scene_graph::Object object, std::string surface)
{
    Eigen::Matrix4f transform = object.pose.TransformationFromPose();

    // object vertices
    Eigen::Vector4f p0(object.dim[0] / 2.0, -object.dim[1] / 2.0, -object.dim[2] / 2.0, 1);
    Eigen::Vector4f p1(object.dim[0] / 2.0, object.dim[1] / 2.0, -object.dim[2] / 2.0, 1);
    Eigen::Vector4f p2(-object.dim[0] / 2.0, object.dim[1] / 2.0, -object.dim[2] / 2.0, 1);
    Eigen::Vector4f p3(-object.dim[0] / 2.0, -object.dim[1] / 2.0, -object.dim[2] / 2.0, 1);
    Eigen::Vector4f p4(object.dim[0] / 2.0, -object.dim[1] / 2.0, object.dim[2] / 2.0, 1);
    Eigen::Vector4f p5(object.dim[0] / 2.0, object.dim[1] / 2.0, object.dim[2] / 2.0, 1);
    Eigen::Vector4f p6(-object.dim[0] / 2.0, object.dim[1] / 2.0, object.dim[2] / 2.0, 1);
    Eigen::Vector4f p7(-object.dim[0] / 2.0, -object.dim[1] / 2.0, object.dim[2] / 2.0, 1);

    std::vector<Eigen::Vector4f> vertices{ p0, p1, p2, p3, p4, p5, p6, p7 };
    for (auto& vertice : vertices)
        vertice = transform * vertice;

    // sort in ascending order for z
    std::sort(vertices.begin(), vertices.end(), PointCompByHeight);

    std::vector<Eigen::Vector3f> points;
    if (surface == "upper")
    {
        // take highest 4 points
        for (int i = 4; i < vertices.size(); i++)
            points.push_back(Eigen::Vector3f(vertices[i][0], vertices[i][1], vertices[i][2]));
    }
    else if (surface == "bottom")
    {
        // take lowest 4 points
        for (int i = 0; i < 4; i++)
            points.push_back(Eigen::Vector3f(vertices[i][0], vertices[i][1], vertices[i][2]));
    }
    else
    {
        std::cerr << "unrecognized surface option\n";
        exit(1);
    }

    return points;
}

void PoseToSceneGraph::DrawPolygon(cv::Mat& image, std::vector<Eigen::Vector3f> points)
{
    std::vector<cv::Point> cv_points(4);
    for (int i = 0; i < points.size(); i++)
    {
        cv_points[i].x = points[i][0];
        cv_points[i].y = points[i][1];
    }

    std::vector<cv::Point> polygon_points;
    cv::convexHull(cv::Mat(cv_points), polygon_points);

    cv::Point polygon_array[polygon_points.size()];
    for (int i = 0; i < polygon_points.size(); i++)
    {
        polygon_array[i].x = polygon_points[i].x;
        polygon_array[i].y = polygon_points[i].y;
    }

    cv::fillConvexPoly(image, polygon_array, polygon_points.size(), cv::Scalar(255, 255, 255));

    // cv::imshow("polygon",image);
    // cv::waitKey(-1);
}

bool PoseToSceneGraph::CheckOverlap(scene_graph::Object object1, scene_graph::Object object2)
{
    printf("check overlap (%s, %s): ", object1.name.c_str(), object2.name.c_str());

    std::vector<Eigen::Vector3f> object1_points = ProjectObjectBoudingBox(object1, "bottom");
    std::vector<Eigen::Vector3f> object2_points = ProjectObjectBoudingBox(object2, "upper");

    // find min, max of x, y coordinates in all projected points
    float min_x = INFINITY, min_y = INFINITY;
    float max_x = -INFINITY, max_y = -INFINITY;
    std::vector<Eigen::Vector3f> all_points;
    all_points.insert(all_points.end(), object1_points.begin(), object1_points.end());
    all_points.insert(all_points.end(), object2_points.begin(), object2_points.end());

    for (const auto& point : all_points)
        if (point[0] < min_x)
            min_x = point[0];
    for (const auto& point : all_points)
        if (point[1] < min_y)
            min_y = point[1];
    for (const auto& point : all_points)
        if (point[0] > max_x)
            max_x = point[0];
    for (const auto& point : all_points)
        if (point[1] > max_y)
            max_y = point[1];

    // move and scale points to [0, 100] in both x, y directions, then offset by 10
    // image with dimension 120x120
    for (auto& point : object1_points)
    {
        point = point - Eigen::Vector3f(min_x, min_y, 0);
        point[0] = point[0] * 100.0 / (max_x - min_x) + 10.0;
        point[1] = point[1] * 100.0 / (max_y - min_y) + 10.0;
    }

    for (auto& point : object2_points)
    {
        point = point - Eigen::Vector3f(min_x, min_y, 0);
        point[0] = point[0] * 100.0 / (max_x - min_x) + 10.0;
        point[1] = point[1] * 100.0 / (max_y - min_y) + 10.0;
    }

    // draw filled polygons and check overlap in opencv
    cv::Mat image1(120, 120, CV_8UC3, cv::Scalar(0, 0, 0));
    DrawPolygon(image1, object1_points);
    cv::Mat image1_bw;
    cv::cvtColor(image1, image1_bw, CV_RGB2GRAY);
    cv::threshold(image1_bw, image1_bw, 0, 255, CV_THRESH_BINARY);

    cv::Mat image2(120, 120, CV_8UC3, cv::Scalar(0, 0, 0));
    DrawPolygon(image2, object2_points);
    cv::Mat image2_bw;
    cv::cvtColor(image2, image2_bw, CV_RGB2GRAY);
    cv::threshold(image2_bw, image2_bw, 0, 255, CV_THRESH_BINARY);

    cv::Mat overlap_image;
    cv::bitwise_and(image1_bw, image2_bw, overlap_image);
    int overlap = cv::countNonZero(overlap_image);
    printf("overlap %d pixels in 120x120 scaled image\n", overlap);

    if (overlap > 0)
        return true;
    else
        return false;
}

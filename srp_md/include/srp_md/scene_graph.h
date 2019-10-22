#pragma once
//
// @file src/scene_graph/object.h
// @brief General object description
// @author Zhiqiang Sui, Zhen Zeng
// University of Michigan, 2016
//

#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>
#include <string>
#include <vector>
#include <eigen3/Eigen/Geometry>

namespace scene_graph
{
class Pose
{
  public:
    glm::vec3 pos_;
    glm::vec3 euler_;

    Pose()
    {
        pos_ = glm::vec3(0.0);
        euler_ = glm::vec3(0.0);
    }

    void PoseFromTransformation(Eigen::Matrix4f transform)
    {
        Eigen::Matrix3f m = transform.block<3, 3>(0, 0);
        Eigen::Vector3f euler = m.eulerAngles(0, 1, 2);
        Eigen::Vector3f trans = transform.block<3, 1>(0, 3);

        pos_.x = trans[0];
        pos_.y = trans[1];
        pos_.z = trans[2];
        euler_.x = euler[0];
        euler_.y = euler[1];
        euler_.z = euler[2];
    }

    Eigen::Matrix4f TransformationFromPose()
    {
        Eigen::Matrix4f transform;
        transform.setIdentity();

        Eigen::Matrix3f m;
        m = Eigen::AngleAxisf(euler_.x, Eigen::Vector3f::UnitX()) *
            Eigen::AngleAxisf(euler_.y, Eigen::Vector3f::UnitY()) *
            Eigen::AngleAxisf(euler_.z, Eigen::Vector3f::UnitZ());
        transform.block<3, 3>(0, 0) = m;

        Eigen::Vector3f trans(pos_.x, pos_.y, pos_.z);
        transform.block<3, 1>(0, 3) = trans;

        return transform;
    }

    friend std::ostream& operator<<(std::ostream& os, const Pose& pose)
    {
        os << pose.pos_.x << " " << pose.pos_.y << " " << pose.pos_.z << " " << pose.euler_.x << " " << pose.euler_.y
           << " " << pose.euler_.z << "\n";
        return os;
    }
};

struct Object
{
    Pose pose;
    glm::vec3 dim;

    std::string name;
    int id;
    int pose_label;

    Object() : pose_label(-1)
    {
    }
};

typedef std::vector<Object> ObjectList;

enum RelationType
{
    kOn
};

struct Relation
{
    int n1;  // object id 1
    int n2;  // object id 2
    std::string name1;
    std::string name2;
    RelationType type;  // relation type

    Relation(RelationType relationType, int object1, int object2, std::string object_name1, std::string object_name2)
      : n1(object1), n2(object2), name1(object_name1), name2(object_name2), type(relationType)
    {
    }

    // void operator=(const Relation& relation){
    // 	n1 = relation.n1;
    // 	n2 = relation.n2;
    // 	type = relation.type;
    // }
};

struct SceneGraph
{
    std::vector<Object> object_list;
    std::vector<Relation> rel_list;

    // void operator=(const SceneGraph& scene_graph){
    // 	object_list.clear();
    // 	for(size_t i = 0; i < scene_graph.object_list.size(); i++){
    // 		Object object = scene_graph.object_list[i];
    // 		object_list.push_back(object);
    // 	}

    // 	rel_list.clear();
    // 	for(size_t i =0; i < scene_graph.rel_list.size(); i++){
    // 		Relation rel = scene_graph.rel_list[i];
    // 		rel_list.push_back(rel);
    // 	}

    // 	objIndMap.clear();
    // 	typedef std::map<ObjectName, int>::const_iterator it_type;
    // 	for(it_type it = scene_graph.objIndMap.begin(); it != scene_graph.objIndMap.end(); it++){
    // 		objIndMap.insert(std::make_pair(it->first, it->second));
    // 	}
    // 	//objIndMap = scene_graph.objIndMap;
    // }

    // void display(){
    // 	 for(size_t i = 0; i < object_list.size(); i++){
    // 		Object object = object_list[i];
    // 	//	std::cout << object.name << ' ' << object.type << ' ' << object.id << ' ' << object.length << ' ' <<
    // object.width << object.height << std::endl;
    // 		std::cout << nameObjHash[object.name] << " pose: " << object.pos.x << ' ' << object.pos.y << ' ' <<
    // object.pos.z << ' ' << object.pos.rot_x << ' ' << object.pos.rot_y << ' ' << object.pos.rot_z << " dimension: "<<
    // object.length << ' ' << object.width << ' ' << object.height << " support_surface:" << object.support_surface <<
    // " normal_x: " << object.norm_x.x << ' ' << object.norm_x.y << ' ' << object.norm_x.z << " normal_y:" <<
    // object.norm_y.x << ' ' << object.norm_y.y << ' ' << object.norm_y.z << " normal_z:" << object.norm_z.x << ' ' <<
    // object.norm_z.y << ' ' << object.norm_z.z << std::endl;
    // 	}

    // 	std::cout << "relations: ";
    // 	for(size_t i = 0; i < rel_list.size(); i++){
    // 		//Relation rel = rel_list[i];
    // 		std::cout <<  "on" << "(" << nameObjHash[rel_list[i].n1] << "," << nameObjHash[rel_list[i].n2] << ") ";
    // 	}
    // 	std::cout << std::endl;

    // }
};

}  // namespace scene_graph

#pragma once
//
// @file src/scene_graph/object.h
// @brief General object description
// @author Zhiqiang Sui, Zhen Zeng
// University of Michigan, 2016
//

#include <string>
#include <vector>
#include <eigen3/Eigen/Geometry>

namespace scene_graph
{

struct Object
{
    Eigen::Affine3d pose;
    Eigen::Vector3d dim;

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
    kOn,
    kSupport
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

    std::string get_type_str()
    {
        std::string type_str;
        switch (type)
        {
            case kOn:
                type_str = "on";
                break;
            case kSupport:
                type_str = "support";
                break;
        }
        return type_str;
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

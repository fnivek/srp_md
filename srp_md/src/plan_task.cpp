//
// @file progress/planSRPTask.cpp
// @brief generate symbolic plan for SRP task
// @author Zhen Zeng
// University of Michigan, 2017
//

#include <fstream>
#include <iostream>
#include <cstdlib>
#include <map>
#include <sstream>
#include <eigen3/Eigen/Geometry>
#include <math.h>
#include <iterator>

#include "srp_md/object.h"
#include "srp_md/pose_to_scene_graph.h"

using namespace std;

struct TriggerTriplet
{
    std::string obj1;
    std::string obj2;
    int trigger_id;
    int poseLabel;
};

class TaskPlanner
{
  public:
    TaskPlanner()
    {
        planner_path = "/home/logan/workspace/library/pbd/programing_by_demonstration/res/planner/";
        poseLabel = 1;
        triggerLabel = 1;
    }

    ~TaskPlanner()
    {
    }

    void displayResult(std::string prefix = "fetch")
    {
        std::string resultFileName = planner_path + prefix + "_task.pddl.soln";

        std::ifstream result_file(resultFileName.c_str(), std::ifstream::in);
        std::string line;

        while (std::getline(result_file, line))
            std::cout << line << std::endl;
    }

    void generateObjPosLabelHash(renderer::SceneGraph goal_scene_graph, renderer::SceneGraph current_scene_graph)
    {
        for (size_t i = 0; i < goal_scene_graph.rel_list.size(); i++)
        {
            renderer::Relation rel = goal_scene_graph.rel_list[i];
            if (rel.type == renderer::RelationType::kOn && rel.name1 != "tray")
            {
                if (rel.name2 == "table" || rel.name2 == "tray")
                {
                    // find object pose add push to hash table
                    for (size_t j = 0; j < goal_scene_graph.object_list.size(); j++)
                    {
                        if (goal_scene_graph.object_list[j].id == rel.n1)
                        {
                            goal_ObjHash.insert(std::make_pair(rel.n1, goal_scene_graph.object_list[j]));

                            printf("#GOAL: object %s on table or tray\n", goal_scene_graph.object_list[j].name.c_str());
                            break;
                        }
                    }
                }
            }
        }

        std::map<int, renderer::Object> current_ObjHash;
        for (size_t i = 0; i < current_scene_graph.rel_list.size(); i++)
        {
            renderer::Relation rel = current_scene_graph.rel_list[i];
            if (rel.type == renderer::RelationType::kOn && rel.name1 != "tray")
            {
                if (rel.name2 == "table" || rel.name2 == "tray")
                {
                    // find object pose add push to hash table
                    for (size_t j = 0; j < current_scene_graph.object_list.size(); j++)
                    {
                        if (current_scene_graph.object_list[j].id == rel.n1)
                        {
                            current_ObjHash.insert(std::make_pair(rel.n1, current_scene_graph.object_list[j]));

                            printf("$CURR: object %s on table or tray\n",
                                   current_scene_graph.object_list[j].name.c_str());
                            break;
                        }
                    }
                }
            }
        }

        // go through objects that stay at the same place first
        for (std::map<int, renderer::Object>::iterator x = goal_ObjHash.begin(); x != goal_ObjHash.end(); ++x)
        {
            for (std::map<int, renderer::Object>::iterator y = current_ObjHash.begin(); y != current_ObjHash.end(); ++y)
            {
                if (x->second.name == y->second.name)
                {
                    renderer::Pose pose1 = x->second.pose;  // goal
                    renderer::Pose pose2 = y->second.pose;  // current

                    // float distance = (pose1.pos_.x-pose2.position.x)*(pose1.position.x-pose2.position.x)
                    //             + (pose1.position.y-pose2.position.y)*(pose1.position.y-pose2.position.y)
                    //             + (pose1.position.z-pose2.position.z)*(pose1.position.z-pose2.position.z);
                    float distance = glm::distance(pose1.pos_, pose2.pos_);

                    // assign same label pose if the change of position is less than 0.1m
                    if (distance < 0.1)
                    {
                        printf("%s-%s stays at the same place\n", x->second.name.c_str(), y->second.name.c_str());

                        x->second.pose_label = poseLabel;
                        goal_occupied_pose.push_back(poseLabel);
                        goal_objNamePoseLabelHash.insert(std::make_pair(x->second.name, poseLabel));

                        y->second.pose_label = poseLabel;
                        current_occupied_pose.push_back(poseLabel);
                        current_objNamePoseLabelHash.insert(std::make_pair(y->second.name, poseLabel));

                        poseLabel++;
                    }
                }
                else
                    continue;
            }
        }

        // DEBUG: current_occupied_pose
        // 	for(size_t i=0; i<current_occupied_pose.size(); i++)
        // 	{
        // 		printf("current_occupied_pose[%d]=%d\n", i, current_occupied_pose[i]);
        // 	}

        // go through objects that change positions and see if it is taking up places that are being occupied now
        for (std::map<int, renderer::Object>::iterator x = goal_ObjHash.begin(); x != goal_ObjHash.end(); ++x)
        {
            for (std::map<int, renderer::Object>::iterator y = current_ObjHash.begin(); y != current_ObjHash.end(); ++y)
            {
                // std::map<string, int>::iterator goal_it;
                // goal_it = goal_objNamePoseLabelHash.find(x->second.name);
                // std::map<string, int>::iterator current_it;
                // current_it = current_objNamePoseLabelHash.find(nameObjHash[y->first]);
                if ((x->second.pose_label == -1) && (y->second.pose_label == -1))
                {
                    renderer::Pose pose1 = x->second.pose;  // goal
                    renderer::Pose pose2 = y->second.pose;  // current

                    float distance = (pose1.pos_[0] - pose2.pos_[0]) * (pose1.pos_[0] - pose2.pos_[0]) +
                                     (pose1.pos_[1] - pose2.pos_[1]) * (pose1.pos_[1] - pose2.pos_[1]);
                    distance = sqrt(distance);

                    // assign same label pose if 2d distance in x-y plane is less than 0.1m
                    if (distance < 0.1)
                    {
                        printf("%s goal loc is similar to current %s loc\n", x->second.name.c_str(),
                               y->second.name.c_str());

                        if (x->second.name == y->second.name)
                        {
                            // same object but drastically different pose
                            x->second.pose_label = poseLabel;
                            goal_occupied_pose.push_back(poseLabel);
                            goal_objNamePoseLabelHash.insert(std::make_pair(x->second.name, poseLabel));

                            poseLabel++;

                            y->second.pose_label = poseLabel;
                            current_occupied_pose.push_back(poseLabel);
                            current_objNamePoseLabelHash.insert(std::make_pair(y->second.name, poseLabel));
                        }
                        else
                        {
                            x->second.pose_label = poseLabel;
                            goal_occupied_pose.push_back(poseLabel);
                            goal_objNamePoseLabelHash.insert(std::make_pair(x->second.name, poseLabel));

                            y->second.pose_label = poseLabel;
                            current_occupied_pose.push_back(poseLabel);
                            current_objNamePoseLabelHash.insert(std::make_pair(y->second.name, poseLabel));
                        }

                        poseLabel++;
                    }
                }
            }
        }

        // DEBUG: current_occupied_pose
        // 	for(size_t i=0; i<current_occupied_pose.size(); i++)
        // 	{
        // 		printf("current_occupied_pose[%d]=%d\n", i, current_occupied_pose[i]);
        // 	}

        // go through objets that needs to be moved to a new empty place
        for (std::map<int, renderer::Object>::iterator x = goal_ObjHash.begin(); x != goal_ObjHash.end(); ++x)
            if (x->second.pose_label == -1)
            {
                printf("%s needs to be moved to a new empty place\n", x->second.name.c_str());

                x->second.pose_label = poseLabel;
                goal_occupied_pose.push_back(poseLabel);
                goal_objNamePoseLabelHash.insert(std::make_pair(x->second.name, poseLabel));

                poseLabel++;
            }

        // go through objects that are now at a place that will become empty
        for (std::map<int, renderer::Object>::iterator x = current_ObjHash.begin(); x != current_ObjHash.end(); ++x)
            if (x->second.pose_label == -1)
            {
                printf("%s is at a place that will become empty\n", x->second.name.c_str());

                x->second.pose_label = poseLabel;
                current_occupied_pose.push_back(poseLabel);
                current_objNamePoseLabelHash.insert(std::make_pair(x->second.name, poseLabel));

                poseLabel++;
            }

        // objects that are in the current scene graph (on table) but not in the goal scene graph are supposed to be
        // moved into bin
        for (size_t i = 0; i < current_scene_graph.object_list.size(); i++)
        {
            bool extra_object = true;
            std::string extra_object_name = current_scene_graph.object_list[i].name;
            for (size_t j = 0; j < goal_scene_graph.object_list.size(); j++)
            {
                if (current_scene_graph.object_list[i].name == goal_scene_graph.object_list[j].name)
                {
                    extra_object = false;
                    printf("%s currently present is in goal scene\n", current_scene_graph.object_list[i].name.c_str());
                    break;
                }
            }

            if (extra_object)
            {
                printf("%s currently present is not in goal scene\n", current_scene_graph.object_list[i].name.c_str());
                extra_objNamePoseLabelHash.insert(std::make_pair(current_scene_graph.object_list[i].name, poseLabel));
                goal_occupied_pose.push_back(poseLabel);
                poseLabel++;
            }
        }

        // DEBUG: current_occupied_pose
        // 	for(size_t i=0; i<current_occupied_pose.size(); i++)
        // 	{
        // 		printf("current_occupied_pose[%d]=%d\n", i, current_occupied_pose[i]);
        // 	}
    }

    void generateObjTrigger(renderer::SceneGraph goal_scene_graph)
    {
        // find close locations of objects on table
        for (std::map<int, renderer::Object>::iterator x = goal_ObjHash.begin(); x != goal_ObjHash.end(); ++x)
        {
            assert(x->second.name != "tray");

            for (std::map<int, renderer::Object>::iterator y = std::next(x); y != goal_ObjHash.end(); ++y)
            {
                assert(y->second.name != "tray");

                printf("comparing %s goal pose and %s goal pose\n", x->second.name.c_str(), y->second.name.c_str());

                renderer::Pose pose1 = x->second.pose;
                renderer::Pose pose2 = y->second.pose;

                float distance = (pose1.pos_[0] - pose2.pos_[0]) * (pose1.pos_[0] - pose2.pos_[0]) +
                                 (pose1.pos_[1] - pose2.pos_[1]) * (pose1.pos_[1] - pose2.pos_[1]);
                distance = sqrt(distance);

                // assign same label pose if the 2d distance in x-y plane is less than 0.15m
                if (distance < 0.15)
                {
                    printf("%s goal pose is close to %s goal pose\n", x->second.name.c_str(), y->second.name.c_str());
                    TriggerTriplet trigger;
                    trigger.trigger_id = triggerLabel++;
                    trigger.obj1 = x->second.name;
                    trigger.obj2 = y->second.name;
                    // find a clear pose (loc 1-8 available)
                    for (int i = 1; i < 8; i++)
                    {
                        if (std::find(current_occupied_pose.begin(), current_occupied_pose.end(), i) ==
                                current_occupied_pose.end() &&
                            std::find(goal_occupied_pose.begin(), goal_occupied_pose.end(), i) ==
                                goal_occupied_pose.end())
                        {
                            trigger.poseLabel = i;
                            goal_occupied_pose.push_back(i);
                            break;
                        }
                    }

                    triggers_list.push_back(trigger);
                }
            }
        }
    }

    void setExtendedTask(std::string goal_scene_graph_path, std::string current_scene_graph_path,
                         std::string prefix = "fetch")
    {
        std::string taskFileName = planner_path + prefix + "_task.pddl";

        std::ofstream task_file;
        task_file.open(taskFileName.c_str());

        // task objects, locations
        task_file << "(define (problem srp)" << std::endl;
        task_file << "  (:domain srp)" << std::endl;
        task_file << "  (:objects" << std::endl;
        task_file << "   arm - gripper" << std::endl;
        task_file << "   ";
        task_file << "red_bowl spray_bottle_a toy tide downy pringle clorox sugar shampoo scotch_brite blue_cup tray";
        task_file << " - object" << std::endl;
        task_file << "   pos1 pos2 pos3 pos4 pos5 pos6 pos7 pos8 - location" << std::endl << "   ";

        for (int i = 1; i < triggerLabel; i++)
            task_file << "trig" << i << " ";

        task_file << "- trigger)" << std::endl << std::endl;

        // task: initial state
        std::cout << "load current scene graph file" << std::endl;
        std::ifstream infile(current_scene_graph_path.c_str(), std::ifstream::in);
        std::string line;

        task_file << "  (:init (free arm)";
        for (int i = 1; i <= 8; i++)
        {
            if (std::find(current_occupied_pose.begin(), current_occupied_pose.end(), i) == current_occupied_pose.end())
            {
                task_file << " (clear_pos pos" << i << ")";
            }
        }

        // either at obj pos_id, or on obj1 obj2 (should not have on obj tray)
        while (std::getline(infile, line))
        {
            std::size_t found_on = line.find("on");
            std::size_t found_tray = line.find("tray");

            if (found_on != std::string::npos && found_tray != std::string::npos)
            {
                std::istringstream iss(line);
                std::vector<std::string> tokens;
                std::copy(std::istream_iterator<std::string>(iss), std::istream_iterator<std::string>(),
                          std::back_inserter(tokens));

                task_file << " (at " << tokens[1] << " pos" << current_objNamePoseLabelHash[tokens[1]] << ")";
            }
            else if (line != "")
                task_file << " (" << line << ")";
        }

        // add extended trigger relationships
        for (size_t i = 0; i < triggers_list.size(); i++)
        {
            task_file << " (has " << triggers_list[i].obj1 << " trig" << triggers_list[i].trigger_id << ")";
            task_file << " (in pos" << triggers_list[i].poseLabel << " trig" << triggers_list[i].trigger_id << ")";
        }

        for (size_t i = 0; i < current_scene_graph_copy.object_list.size(); i++)
        {
            if (current_scene_graph_copy.object_list[i].name != "tray")
                task_file << " (temp " << current_scene_graph_copy.object_list[i].name << ")";
        }

        task_file << ")" << std::endl << std::endl;

        // task: goal state
        std::cout << "load goal scene graph file" << std::endl;
        task_file << "(:goal (AND";
        std::ifstream goalInFile(goal_scene_graph_path.c_str(), std::ifstream::in);
        while (std::getline(goalInFile, line))
        {
            std::size_t found_on = line.find("on");
            std::size_t found_tray = line.find("tray");

            if (found_on != std::string::npos && found_tray != std::string::npos)
            {
                bool no_trigger = true;
                for (size_t i = 0; i < triggers_list.size(); i++)
                {
                    if (line.find(triggers_list[i].obj2) != std::string::npos)
                        no_trigger = false;
                }

                if (no_trigger)
                {
                    std::istringstream iss(line);
                    std::vector<std::string> tokens;
                    std::copy(std::istream_iterator<std::string>(iss), std::istream_iterator<std::string>(),
                              std::back_inserter(tokens));

                    task_file << " (at " << tokens[1] << " pos" << goal_objNamePoseLabelHash[tokens[1]] << ")";
                }
            }
            else if (line != "")
                task_file << " (" << line << ")";
        }

        // add extra objects in current scene graph
        for (std::map<string, int>::iterator x = extra_objNamePoseLabelHash.begin();
             x != extra_objNamePoseLabelHash.end(); ++x)
        {
            task_file << " (at " << x->first << " pos" << x->second << ")";
        }

        // add extended trigger relationships
        for (size_t i = 0; i < triggers_list.size(); i++)
        {
            task_file << " (contains " << triggers_list[i].obj2 << " trig" << triggers_list[i].trigger_id << ")";
        }

        for (size_t i = 0; i < goal_scene_graph_copy.object_list.size(); i++)
        {
            if (goal_scene_graph_copy.object_list[i].name != "tray")
                task_file << " (final " << goal_scene_graph_copy.object_list[i].name << ")";
        }

        for (std::map<string, int>::iterator x = extra_objNamePoseLabelHash.begin();
             x != extra_objNamePoseLabelHash.end(); ++x)
        {
            task_file << " (temp " << x->first << ")";
        }

        task_file << "))" << std::endl;

        task_file << ")" << std::endl;

        task_file.close();
    }

    void plan_extended(renderer::SceneGraph goal_scene_graph, renderer::SceneGraph current_scene_graph,
                       std::string goal_scene_graph_path, std::string current_scene_graph_path,
                       std::string prefix = "fetch")
    {
        current_scene_graph_copy = current_scene_graph;
        goal_scene_graph_copy = goal_scene_graph;
        generateObjPosLabelHash(goal_scene_graph, current_scene_graph);
        generateObjTrigger(goal_scene_graph);
        setExtendedTask(goal_scene_graph_path, current_scene_graph_path, prefix);

        std::string command = planner_path + "pyperplan/pyperplan.py " + planner_path + "extended_domain.pddl " +
                              planner_path + prefix + "_task.pddl";
        printf("%s\n", command.c_str());
        system(command.c_str());
    }

  private:
    int poseLabel;
    int triggerLabel;
    std::string planner_path;
    std::map<int, renderer::Object> goal_ObjHash;
    std::map<std::string, int> goal_objNamePoseLabelHash;  // from object name to pose label, pose label # starts from 1
    std::map<std::string, int> current_objNamePoseLabelHash;
    std::map<std::string, int> extra_objNamePoseLabelHash;

    std::vector<int> current_occupied_pose;
    std::vector<int> goal_occupied_pose;
    std::vector<TriggerTriplet> triggers_list;

    renderer::SceneGraph current_scene_graph_copy;
    renderer::SceneGraph goal_scene_graph_copy;
};

int main(int argc, char** argv)
{
    TaskPlanner planner;
    std::string goal_pose_path = std::string(argv[1]);
    std::string current_pose_path = std::string(argv[2]);

    renderer::SceneGraph goal_scene_graph;
    renderer::SceneGraph current_scene_graph;

    // PoseToSceneGraph goal_scene_generator(goal_pose_path);
    // goal_scene_generator.CalcSceneGraph();
    // goal_scene_generator.WriteSceneGraph("goal_scene_graph.txt");

    // PoseToSceneGraph current_scene_generator(current_pose_path);
    // current_scene_generator.CalcSceneGraph();
    // current_scene_generator.WriteSceneGraph("current_scene_graph.txt");

    // planner.plan_extended(goal_scene_generator.get_scene_graph(), current_scene_generator.get_scene_graph(),
    //                       "goal_scene_graph.txt", "current_scene_graph.txt");

    std::cout << "\n\n";
    planner.displayResult();

    return 0;
}

#ifndef SRP_MD_ACTIONS_H_
#define SRP_MD_ACTIONS_H_

#include <fstream>
#include <thread>
#include <chrono>
#include <future>
#include <mutex>
#include <condition_variable>
#include <random>

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseArray.h>

#include <control_msgs/GripperCommandAction.h>
#include <control_msgs/GripperCommand.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/PointHeadAction.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <moveit_msgs/GetPlanningScene.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <visualization_msgs/Marker.h>

class Act;

template<typename Goal>
class BaseFunc {
protected:
    Goal goal;
    Act *act;
    std::atomic<bool> isPreempted;
    std::future<void> f1, f2;
    std::mutex preempt_mutex;
    std::condition_variable cond_var;
    void preempt_helper();

public:
    BaseFunc(Goal &g, Act *a);
    bool is_finished();
    bool preempt();
    void start();
    void wait_for_result();
    virtual void func() = 0;
};


template<typename Goal>
class TestFunc : public BaseFunc<Goal> {
public:
    TestFunc(Goal &goal, Act *a);
    void func() override;
};


class Act {
    template<typename Goal> friend class BaseFunc;
protected:
    ros::NodeHandle nh_;
    std::map<std::string, moveit::planning_interface::MoveGroupInterface::Plan> predefined_plan_map;
    std::map<std::string, std::map<std::string, double>> predefined_joint_angles_;
    moveit::planning_interface::MoveGroupInterface move_group_;

    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac_arm_joints_;
    actionlib::SimpleActionClient<control_msgs::GripperCommandAction> ac_gripper_;
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac_torso_ctrl_;
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac_head_pan_tilt_;
    actionlib::SimpleActionClient<control_msgs::PointHeadAction> ac_head_point_;

    // std::map<int, geometry_msgs::Pose> shelf_id_to_retrieve_pose_;
    ros::ServiceClient get_scene_client_;
    ros::Publisher planning_scene_diff_pub_;
    robot_model_loader::RobotModelLoader robot_model_loader_;
    robot_model::RobotModelPtr kinematic_model_;
    robot_state::RobotStatePtr kinematic_state_;
    const robot_state::JointModelGroup* joint_model_group_;

    const double PI;
    const double MAX_HEAD_PAN, MIN_HEAD_PAN, MAX_HEAD_TILT, MIN_HEAD_TILT;
    const double MAX_TORSO_HEIGHT, MIN_TORSO_HEIGHT;
    const double MAX_GRIPPER_VAL, MIN_GRIPPER_VAL;

public:
    // default constructor that starts all the action client
    Act(ros::NodeHandle& nh);

    // // load 7 predefined poses, new poses may be added later.
    // void load_predefined_poses();

    // wrapper function for "action_client.waitForServer()"
    template <typename Client>
    void wait_for_server(Client &ac, const std::string &topic_name);

    // include functions!
    bool generate_init_trajectory();
    bool load_trajectory(moveit::planning_interface::MoveGroupInterface::Plan &plan,
                         const std::string &trajectory_name);
    bool record_trajectory(const moveit::planning_interface::MoveGroupInterface::Plan& plan, std::string name);

    std::pair<bool, moveit::planning_interface::MoveGroupInterface::Plan>
    plan(const std::string &pose_name, int max_try = 1);
    std::pair<bool, moveit::planning_interface::MoveGroupInterface::Plan>
    plan(const std::map<std::string, double> &joint_angles, int max_try = 1);
    std::pair<bool, moveit::planning_interface::MoveGroupInterface::Plan>
    plan(const geometry_msgs::Pose &pose, int max_try = 1);

    bool boolean_interface(const std::string &action);

    bool grasploc_pick(std::vector<geometry_msgs::Pose> grasp_poses, std::vector<geometry_msgs::Vector3> normals, std::vector<geometry_msgs::Vector3> principals, bool usual_way);

    bool cartesian_grasp(const std::vector<geometry_msgs::Pose> &waypoints, int max_try /* = 3 */);
    std::pair<bool, moveit::planning_interface::MoveGroupInterface::Plan>
    cartesian_move(const geometry_msgs::Pose &end_pose, int max_try = 3);
    std::pair<bool, moveit::planning_interface::MoveGroupInterface::Plan>
    cartesian_move(const std::vector<geometry_msgs::Pose> &waypoints, int max_try = 3);
    std::pair<bool, moveit_msgs::RobotTrajectory>
    cartesian_plan(const geometry_msgs::Pose &end_pose, int max_try = 1);
    std::pair<bool, moveit_msgs::RobotTrajectory>
    cartesian_plan(const std::vector<geometry_msgs::Pose> &waypoints, int max_try = 1);

    bool control_gripper(double pos);
    bool open_gripper();
    bool close_gripper();
};


// //template implementation==========================================================================
// template <typename Client>
// void Act::wait_for_server(Client &ac, const std::string &topic_name)
// {
//     ROS_INFO("Waiting for \"%s\" action server...", topic_name.c_str());
//     ac.waitForServer();
//     ROS_INFO("Connected to \"%s\" action server!", topic_name.c_str());
// }


// template<typename Goal>
// std::unique_ptr<BaseFunc<Goal>> Act::test_func(Goal &goal)
// {
//     std::unique_ptr<BaseFunc<Goal>> ptr(new TestFunc<Goal>(goal, this));
//     return ptr;
// }


// template<typename Goal>
// BaseFunc<Goal>::BaseFunc(Goal &g, Act *a) : goal(g), act(u), isPreempted(false)
// {
// }


// template<typename Goal>
// void BaseFunc<Goal>::preempt_helper()
// {
//     {
//         std::unique_lock<std::mutex> ul(preempt_mutex);
//         cond_var.wait(ul, [this] { return isPreempted.load() || is_finished() || !ros::ok(); });
//     }
//     if (!is_finished()) {
//         ROS_INFO("Stopping move_group!");
//         act->move_group_.stop();
//     }
// }


// template<typename Goal>
// bool BaseFunc<Goal>::is_finished()
// {
//     return f1.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
// }


// template<typename Goal>
// bool BaseFunc<Goal>::preempt()
// {
//     if (is_finished()) {
//         ROS_INFO("The function has already ended, no need to preempt!");
//         return false;
//     }
//     this->isPreempted.store(true);
//     ROS_INFO("Preempt movement!");
//     cond_var.notify_one();
//     return true;
// }


// template<typename Goal>
// void BaseFunc<Goal>::start()
// {
//     f1 = std::async(std::launch::async, &BaseFunc::func, this);
//     f2 = std::async(std::launch::async, &BaseFunc::preempt_helper, this);
// }


// template<typename Goal>
// void BaseFunc<Goal>::wait_for_result() {
//     f1.wait();
//     cond_var.notify_one();
// }

// #include "TestFunc.inl"

#endif
// Project
#include "srp_md/actions.h"
#include "srp_md/action_servers/grasploc_pick_as.h"
#include "srp_md/action_servers/move_to_pose_as.h"
#include "srp_md/action_servers/move_to_relative_pose_as.h"
#include "srp_md_msgs/GrasplocPickAction.h"
#include "srp_md_msgs/MoveToPoseAction.h"
#include "srp_md_msgs/MoveToRelativePoseAction.h"

// ROS
#include <ros/ros.h>

int main(int argc, char** argv)
{
    // Start ROS
    ROS_INFO("Initialize ROS");
    ros::init(argc, argv, "action_node");
    ros::NodeHandle nh;
    Act act(nh);

    // Include necessary action servers
    GrasplocPickAS grasplocPickAS(nh, act, "grasploc_pick");
    ROS_INFO("Created GrasplocPickAS.");
    MoveToPoseAS moveToPoseAS(nh, act, "move");
    ROS_INFO("Created MoveToPoseAS.");
    MoveToRelativePoseAS moveToRelativePoseAS(nh, act, "relative_move");
    ROS_INFO("Created MoveToRelativePoseAS.");

    // Loop forever
    ros::Rate sleep_rate(100);
    while (ros::ok())
    {
        ros::spinOnce();
        sleep_rate.sleep();
    }
    return 0;
}

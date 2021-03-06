// Project
#include "srp_md/actions.h"
#include "srp_md/action_servers/move_to_pose_as.h"
#include "srp_md/action_servers/move_to_relative_pose_as.h"
#include "srp_md/action_servers/get_table_as.h"
#include "srp_md/action_servers/point_cloud_as.h"
#include "srp_md/action_servers/free_space_finder_as.h"
#include "srp_md/action_servers/relative_cartesian_move_as.h"
#include "srp_md/action_servers/get_stack_pose_as.h"
#include "srp_md/action_servers/AttachObjectAS.h"
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
    MoveToPoseAS moveToPoseAS(nh, act, "move");
    ROS_INFO("Created MoveToPoseAS.");
    MoveToFirstPoseAS moveToFirstPoseAS(nh, act, "move_to_first_pose");
    ROS_INFO("Created MoveToFirstPoseAS.");
    MoveToRelativePoseAS moveToRelativePoseAS(nh, act, "relative_move");
    ROS_INFO("Created MoveToRelativePoseAS.");
    GetTableAS getTableAS(nh, act, "get_table");
    ROS_INFO("Created GetTableAS.");
    CropPCAS cropPCAS(nh, act, "crop_pc");
    ROS_INFO("Created cropPCAS.");
    TFPCAS tfPCAS(nh, act, "tf_pc");
    ROS_INFO("Created tfPCAS.");
    FreeSpaceFinderAS freeSpaceFindereAS(nh, act, "free_space_finder");
    ROS_INFO("Created FreeSpaceFinderAS.");
    RelativeCartesianMoveAS RelativeCartesianMoveAS(nh, act, "relative_cartesian_move");
    ROS_INFO("Created RelativeCartesianMoveAS.");
    GetStackPoseAS GetStackPoseAS(nh, act, "GetStackPose");
    ROS_INFO("Created GetStackPoseAS.");
    AttachObjectAS attachObjectAS(nh, act, "attach_object");
    ROS_INFO("Created AttachObjectAS.");
    // Loop forever
    ros::Rate sleep_rate(100);
    while (ros::ok())
    {
        ros::spinOnce();
        sleep_rate.sleep();
    }
    return 0;
}

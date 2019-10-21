// Project
#include "srp_md/pose_to_scene_graph.h"

// ROS
#include "ros/ros.h"

int main(int argc, char** argv)
{
    // Start ROS
    ros::init(argc, argv, "pose_to_scene_graph");
    ros::NodeHandle nh;

    // Make a factor graph worker
    PoseToSceneGraph worker;

    // Loop forever
    ros::Rate sleep_rate(100);
    while (ros::ok())
    {
        ros::spinOnce();
        sleep_rate.sleep();
    }
    return 0;
}

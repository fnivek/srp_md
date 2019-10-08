// Project
#include "srp_md/plane_detector.h"

// ROS
#include <ros/ros.h>

int main(int argc, char** argv)
{
    // Start ROS
    ros::init(argc, argv, "plane_detector_node");
    ros::NodeHandle nh;

    // Make the plane detector
    PlaneDetector detector;

    // Loop forever
    ros::Rate sleep_rate(100);
    while (ros::ok())
    {
        ros::spinOnce();
        sleep_rate.sleep();
    }
    return 0;
}

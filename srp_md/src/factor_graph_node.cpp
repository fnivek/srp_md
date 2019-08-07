// Project
#include <srp_md/factor_graph_worker.h>

// ROS
#include <ros/ros.h>


int main(int argc, char** argv) {
    // Make a factor graph worker
    FactorGraphWorker worker;

    // ROS
    ros::init(argc, argv, "fg_worker");
    ros::NodeHandle nh;
    ros::Rate sleep_rate(10);
    while(ros::ok()) {
        ros::spinOnce();
        sleep_rate.sleep();
    }
    return 0;
}

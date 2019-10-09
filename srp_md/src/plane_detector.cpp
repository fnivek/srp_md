#include "srp_md/plane_detector.h"

PlaneDetector::PlaneDetector() : server_(nh_, "plane_detector", boost::bind(&PlaneDetector::Callback, this, _1), false)
{
    server_.start();
}

void PlaneDetector::Callback(const srp_md_msgs::DetectPlaneGoalConstPtr& goal)
{
    // Get point cloud
    sensor_msgs::PointCloud2ConstPtr ros_pc =
        ros::topic::waitForMessage<sensor_msgs::PointCloud2>(goal->pc_topic, ros::Duration(5));
    if (!ros_pc)
    {
        ROS_WARN("Did not get a point cloud on %s in 5s", goal->pc_topic.c_str());
        server_.setAborted();
        return;
    }

    srp_md_msgs::DetectPlaneResult result;
    server_.setSucceeded(result);
}

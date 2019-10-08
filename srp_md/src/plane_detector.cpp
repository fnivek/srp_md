#include "srp_md/plane_detector.h"

PlaneDetector::PlaneDetector() : server_(nh_, "plane_detector", boost::bind(&PlaneDetector::Callback, this, _1), false)
{
    server_.start();
}

void PlaneDetector::Callback(const srp_md_msgs::DetectPlaneGoalConstPtr& goal)
{
    srp_md_msgs::DetectPlaneResult result;
    server_.setSucceeded(result);
}

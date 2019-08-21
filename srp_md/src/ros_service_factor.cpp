#include "srp_md/ros_service_factor.h"

// Constructor
RosServiceFactor::RosServiceFactor(const std::string& service_name, const dai::VarSet& vars)
  : service_name_(service_name)
{

}

// Setup ros
// Returns false if something goes wrong otherwise true
bool RosServiceFactor::Setup()
{
    ros::NodeHandle nh;
    client_ = nh.serviceClient<srp_md::EvalFactor>(service_name_);
    if(!client_.waitForExistence(ros::Duration(1)))
    {
        ROS_ERROR("Failed to find %s service after waiting 1s", service_name_.c_str());
        return false;
    }
}

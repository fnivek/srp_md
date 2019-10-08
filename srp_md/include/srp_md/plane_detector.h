#ifndef SRP_MD_PLANE_DETECTOR_H_
#define SRP_MD_PLANE_DETECTOR_H_

// ROS
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

// Project
#include <srp_md_msgs/DetectPlaneAction.h>

class PlaneDetector
{
  public:
    PlaneDetector();

  private:
    void Callback(const srp_md_msgs::DetectPlaneGoalConstPtr& goal);

  private:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<srp_md_msgs::DetectPlaneAction> server_;
};

#endif

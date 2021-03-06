#ifndef SRP_MD_FG_WORKER_H_
#define SRP_MD_FG_WORKER_H_

// ROS includes
#include <ros/ros.h>

// DAI includes
#include <dai/scenegraph.h>

// C++
#include <vector>

// Project includes
#include "srp_md_msgs/GetGoal.h"

class FactorGraphWorker
{
  public:
    FactorGraphWorker();
    void Setup();

  private:
    bool GetGoal(srp_md_msgs::GetGoalRequest& req, srp_md_msgs::GetGoalResponse& resp);

  private:
    ros::ServiceServer goal_server_;
};

#endif

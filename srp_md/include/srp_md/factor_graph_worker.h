#ifndef SRP_MD_FG_WORKER_H_
#define SRP_MD_FG_WORKER_H_

// ROS includes
#include <ros/ros.h>
#include <std_msgs/String.h>

// DAI includes
#include <dai/scenegraph.h>

// C++
#include <vector>

// Project includes
#include "srp_md/GetGoal.h"

class FactorGraphWorker
{
  public:
    FactorGraphWorker();
    void Setup();

  private:
    bool GetGoal(srp_md::GetGoalRequest& req, srp_md::GetGoalResponse& resp);

  private:
    ros::ServiceServer goal_server_;
};

#endif

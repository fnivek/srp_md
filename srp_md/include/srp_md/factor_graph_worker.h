#ifndef SRP_MD_FG_WORKER_H_
#define SRP_MD_FG_WORKER_H_

// ROS includes
#include <ros/ros.h>

// DAI includes
#include <dai/scenegraph.h>

class FactorGraphWorker {
  public:
    FactorGraphWorker();

  private:
    dai::sceneGraph scene_graph_;
};

#endif

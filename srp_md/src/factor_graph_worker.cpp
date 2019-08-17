#include <srp_md/factor_graph_worker.h>

FactorGraphWorker::FactorGraphWorker() : scene_graph_()
{
}

void FactorGraphWorker::Setup()
{
    ros::NodeHandle nh;
    goal_server_ = nh.advertiseService("get_goal", &FactorGraphWorker::GetGoal, this);
}

bool FactorGraphWorker::GetGoal(srp_md::GetGoalRequest& req, srp_md::GetGoalResponse& resp)
{
    resp.result.data = "This is only a test. Repeat this is only a test";
    return true;
}

#ifndef SRP_MD_ACTION_SERVERS_POINT_CLOUD_H_
#define SRP_MD_ACTION_SERVERS_POINT_CLOUD_H_

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

#include "srp_md/actions.h"
#include "srp_md_msgs/CropPCAction.h"

#include <string>

class CropPCAS {
    typedef srp_md_msgs::CropPCAction ActionMsg;
    typedef srp_md_msgs::CropPCGoal GoalMsg;
    typedef srp_md_msgs::CropPCFeedback FeedbackMsg;
    typedef srp_md_msgs::CropPCResult ResultMsg;

public:
    CropPCAS(ros::NodeHandle& nh, Act& act, const std::string& name)
    : as_(nh, name, boost::bind(&CropPCAS::goalCB, this, _1), false)
    , act_(act)
    , name_(name) {
        as_.start();
    }

    void goalCB(const typename GoalMsg::ConstPtr& goal) {
        act_.crop_box_filt_pc(goal->in_pc, goal->crop_box, result_.out_pc);
        as_.setSucceeded(result_);
    }

protected:
    actionlib::SimpleActionServer<ActionMsg> as_;
    Act & act_;
    const std::string name_;
    FeedbackMsg feedback_;
    ResultMsg result_;
};

#endif

#ifndef GET_STACK_POSE_AS_H
#define GET_STACK_POSE_AS_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

#include "srp_md/actions.h"
#include "srp_md_msgs/GetStackPoseAction.h"

#include <string>

class GetStackPoseAS {
    typedef srp_md_msgs::GetStackPoseAction ActionMsg;
    typedef srp_md_msgs::GetStackPoseGoal GoalMsg;
    typedef srp_md_msgs::GetStackPoseFeedback FeedbackMsg;
    typedef srp_md_msgs::GetStackPoseResult ResultMsg;

public:
    GetStackPoseAS(ros::NodeHandle & nh, Act & act, const std::string & name)
    : as_(nh, name, boost::bind(&GetStackPoseAS::goalCB, this, _1), false)
    , act_(act)
    , name_(name) {
        as_.start();
    }

    void goalCB(const typename GoalMsg::ConstPtr & goal) {
        ROS_INFO("GetStackPoseAS goalCB");
        result_.pose = act_.GetStackPose(goal->bot_pose, goal->bot_dim, goal->dim);
        feedback_.feedback = "";
        as_.publishFeedback(feedback_);
        as_.setSucceeded(result_);
        ROS_INFO("Result sent.");
    }

protected:
    actionlib::SimpleActionServer<ActionMsg> as_;
    Act & act_;
    const std::string name_;
    FeedbackMsg feedback_;
    ResultMsg result_;
};

#endif

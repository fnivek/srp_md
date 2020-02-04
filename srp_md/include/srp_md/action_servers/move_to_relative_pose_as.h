#ifndef MOVE_TO_RELATIVE_POSE_AS_H
#define MOVE_TO_RELATIVE_POSE_AS_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

#include "srp_md/actions.h"
#include "srp_md_msgs/MoveToRelativePoseAction.h"

#include <string>

class MoveToRelativePoseAS {
    typedef srp_md_msgs::MoveToRelativePoseAction ActionMsg;
    typedef srp_md_msgs::MoveToRelativePoseGoal GoalMsg;
    typedef srp_md_msgs::MoveToRelativePoseFeedback FeedbackMsg;
    typedef srp_md_msgs::MoveToRelativePoseResult ResultMsg;

public:
    MoveToRelativePoseAS(ros::NodeHandle & nh, Act & act, const std::string & name)
    : as_(nh, name, boost::bind(&MoveToRelativePoseAS::goalCB, this, _1), false)
    , act_(act)
    , name_(name) {
        as_.start();
    }

    void goalCB(const typename GoalMsg::ConstPtr & goal) {
        ROS_INFO("MoveToRelativePoseAS goalCB");
        result_.success = act_.relative_move(goal->transform);
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

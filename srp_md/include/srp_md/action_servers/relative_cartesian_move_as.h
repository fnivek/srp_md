#ifndef RELATIVE_CARTESIAN_MOVE_H
#define RELATIVE_CARTESIAN_MOVE_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

#include "srp_md/actions.h"
#include "srp_md_msgs/RelativeCartesianMoveAction.h"

#include <string>

class RelativeCartesianMoveAS {
    typedef srp_md_msgs::RelativeCartesianMoveAction ActionMsg;
    typedef srp_md_msgs::RelativeCartesianMoveGoal GoalMsg;
    typedef srp_md_msgs::RelativeCartesianMoveFeedback FeedbackMsg;
    typedef srp_md_msgs::RelativeCartesianMoveResult ResultMsg;

public:
    RelativeCartesianMoveAS(ros::NodeHandle & nh, Act & act, const std::string & name)
    : as_(nh, name, boost::bind(&RelativeCartesianMoveAS::goalCB, this, _1), false)
    , act_(act)
    , name_(name) {
        as_.start();
    }

    void goalCB(const typename GoalMsg::ConstPtr& goal) {
        ROS_INFO("RelativeCartesianMoveAS goalCB");
        auto result_temp = act_.relative_cartesian_move(goal->pose_diff_msg);
        result_.success = result_temp.first;
        feedback_.feedback = "";
        if (!result_.success) {
            as_.setAborted(result_);
            return;
        }
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
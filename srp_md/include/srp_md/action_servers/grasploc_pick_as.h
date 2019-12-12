#ifndef GRASPLOC_PICK_AS_H
#define GRASPLOC_PICK_AS_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

#include "srp_md/actions.h"
#include "srp_md_msgs/GrasplocPickAction.h"

// #include "fetch_manipulation_pipeline/GrasplocPickAction.h" <- file doesn't exist?

#include <string>

class GrasplocPickAS {
    typedef srp_md_msgs::GrasplocPickAction ActionMsg;
    typedef srp_md_msgs::GrasplocPickGoal GoalMsg;
    typedef srp_md_msgs::GrasplocPickFeedback FeedbackMsg;
    typedef srp_md_msgs::GrasplocPickResult ResultMsg;

public:
    GrasplocPickAS(ros::NodeHandle & nh, Act & act, const std::string & name)
    : as_(nh, name, boost::bind(&GrasplocPickAS::goalCB, this, _1), false)
    , act_(act)
    , name_(name) {
        as_.start();
    }

    void goalCB(const typename GoalMsg::ConstPtr & goal) {
        ROS_INFO("GrasplocPickAS goalCB");
        result_.success = act_.grasploc_pick(goal->grasp_poses, goal->normal, goal->principal, goal->filter_off);
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
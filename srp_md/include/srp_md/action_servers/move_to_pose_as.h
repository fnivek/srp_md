#ifndef MOVE_TO_POSE_AS_H
#define MOVE_TO_POSE_AS_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

#include "srp_md/actions.h"
#include "srp_md_msgs/MoveToPoseAction.h"
#include "srp_md_msgs/MoveToFirstPoseAction.h"

#include <string>

class MoveToPoseAS {
    typedef srp_md_msgs::MoveToPoseAction ActionMsg;
    typedef srp_md_msgs::MoveToPoseGoal GoalMsg;
    typedef srp_md_msgs::MoveToPoseFeedback FeedbackMsg;
    typedef srp_md_msgs::MoveToPoseResult ResultMsg;

public:
    MoveToPoseAS(ros::NodeHandle & nh, Act & act, const std::string & name)
    : as_(nh, name, boost::bind(&MoveToPoseAS::goalCB, this, _1), false)
    , act_(act)
    , name_(name) {
        as_.start();
    }

    void goalCB(const typename GoalMsg::ConstPtr & goal) {
        ROS_INFO("MoveToPoseAS goalCB");
        result_.success = act_.move(goal->pose);
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

class MoveToFirstPoseAS {
    typedef srp_md_msgs::MoveToFirstPoseAction ActionMsg;
    typedef srp_md_msgs::MoveToFirstPoseGoal GoalMsg;
    typedef srp_md_msgs::MoveToFirstPoseFeedback FeedbackMsg;
    typedef srp_md_msgs::MoveToFirstPoseResult ResultMsg;

public:
    MoveToFirstPoseAS(ros::NodeHandle & nh, Act & act, const std::string & name)
    : as_(nh, name, boost::bind(&MoveToFirstPoseAS::goalCB, this, _1), false)
    , act_(act)
    , name_(name) {
        as_.start();
    }

    void goalCB(const typename GoalMsg::ConstPtr & goal) {
        ROS_INFO("MoveToFirstPoseAS goalCB");
        int i = 0;
        for(auto&& pose : goal->poses)
        {
            result_.success = act_.move(pose);
            if(!result_.success)
            {
                feedback_.feedback = "Pose " + std::to_string(i) + "failed";
                as_.publishFeedback(feedback_);
            }
            else
            {
                as_.setSucceeded(result_);
                ROS_INFO("Result sent.");
                return;
            }
            ++i;
        }

        as_.setAborted(result_);
        ROS_WARN("Failed to go to any pose");
    }

protected:
    actionlib::SimpleActionServer<ActionMsg> as_;
    Act & act_;
    const std::string name_;
    FeedbackMsg feedback_;
    ResultMsg result_;
};

#endif

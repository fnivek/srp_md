#ifndef GET_TABLE_AS_H
#define GET_TABLE_AS_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

#include "srp_md/actions.h"
#include "srp_md_msgs/GetTableAction.h"

#include <string>

class GetTableAS {
    typedef srp_md_msgs::GetTableAction ActionMsg;
    typedef srp_md_msgs::GetTableGoal GoalMsg;
    typedef srp_md_msgs::GetTableFeedback FeedbackMsg;
    typedef srp_md_msgs::GetTableResult ResultMsg;

public:
    GetTableAS(ros::NodeHandle & nh, Act & act, const std::string & name)
    : as_(nh, name, boost::bind(&GetTableAS::goalCB, this, _1), false)
    , act_(act)
    , name_(name) {
        as_.start();
    }

    void goalCB(const typename GoalMsg::ConstPtr& goal) {
        ROS_INFO("GetTableAS goalCB");
        sensor_msgs::PointCloud2::ConstPtr points(new sensor_msgs::PointCloud2(goal->points));
        act_.get_table(points, result_.poses, result_.sizes);
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
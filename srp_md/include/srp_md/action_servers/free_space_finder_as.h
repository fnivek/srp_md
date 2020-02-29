#ifndef FREE_SPACE_FINDER_AS_H
#define FREE_SPACE_FINDER_AS_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

#include "srp_md/actions.h"
#include "srp_md_msgs/FreeSpaceFinderAction.h"

#include <string>

class FreeSpaceFinderAS {
    typedef srp_md_msgs::FreeSpaceFinderAction ActionMsg;
    typedef srp_md_msgs::FreeSpaceFinderGoal GoalMsg;
    typedef srp_md_msgs::FreeSpaceFinderFeedback FeedbackMsg;
    typedef srp_md_msgs::FreeSpaceFinderResult ResultMsg;

public:
    FreeSpaceFinderAS(ros::NodeHandle & nh, Act & act, const std::string & name)
    : as_(nh, name, boost::bind(&FreeSpaceFinderAS::goalCB, this, _1), false)
    , act_(act)
    , name_(name) {
        as_.start();
    }

    void goalCB(const typename GoalMsg::ConstPtr& goal) {
        ROS_INFO("FreeSpaceFinderAS goalCB");
        sensor_msgs::PointCloud2::ConstPtr points(new sensor_msgs::PointCloud2(goal->points));
        act_.free_space_finder(points, goal->plane_bbox, goal->obj_bbox, goal->relation, goal->relative_obj_bbox,
                               result_.pose, goal->distance);
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
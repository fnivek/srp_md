#ifndef ATTACH_OBJECT_AS_H
#define ATTACH_OBJECT_AS_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

#include "srp_md/actions.h"

#include "srp_md_msgs/AttachObjectAction.h"

#include <string>

class AttachObjectAS {
	typedef srp_md_msgs::AttachObjectAction ActionMsg;
	typedef srp_md_msgs::AttachObjectGoal GoalMsg;
	typedef srp_md_msgs::AttachObjectFeedback FeedbackMsg;
	typedef srp_md_msgs::AttachObjectResult ResultMsg;

public:
	AttachObjectAS(ros::NodeHandle & nh, Act & act, const std::string & name)
	: as_(nh, name, boost::bind(&AttachObjectAS::goalCB, this, _1), false)
	, act_(act)
	, name_(name) {
		as_.start();
	}

	void goalCB(const typename GoalMsg::ConstPtr & goal) {
        ROS_INFO("AttachObjectAS goalCB");
		if (goal->toAttach) {
			act_.attach_object_to_gripper(goal->object_name);
			ROS_INFO("Attach %s to gripper", goal->object_name.c_str());
		} else {
			act_.detach_object(goal->object_name);
			ROS_INFO("Detach %s", goal->object_name.c_str());
		}
		feedback_.feedback = "";
        result_.success = true;
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

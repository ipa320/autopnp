/*
 * autopnp_tool_change_client.h
 *
 *  Created on: Aug 27, 2013
 *      Author: rmb-om
 */

#ifndef AUTOPNP_TOOL_CHANGE_CLIENT_H
#define AUTOPNP_TOOL_CHANGE_CLIENT_H

#include "ros/ros.h"
// actions
#include <actionlib/client/simple_action_client.h>
#include <autopnp_tool_change/MoveToWagonAction.h>


class MoveToWagonClient
{
public:
	MoveToWagonClient(ros::NodeHandle nh);
	bool init();
	void run();

protected:
	void doneCb(const actionlib::SimpleClientGoalState& state, const autopnp_tool_change::MoveToWagonResultConstPtr& result);
	void activeCb();
	void feedbackCb(const autopnp_tool_change::MoveToWagonFeedbackConstPtr& feedback);

	ros::NodeHandle node_;

	actionlib::SimpleActionClient<autopnp_tool_change::MoveToWagonAction> move_to_wagon_client;
};

#endif /* AUTOPNP_TOOL_CHANGE_CLIENT_*/

/*
 * autopnp_tool_change_client.h
 *
 *  Created on: Aug 27, 2013
 *      Author: rmb-om
 */

#ifndef AUTOPNP_TOOL_CHANGE_CLIENT_H_
#define AUTOPNP_TOOL_CHANGE_CLIENT_H_

#include "ros/ros.h"
// actions
#include <actionlib/client/simple_action_client.h>
#include <autopnp_tool_change/MoveToSlotAction.h> // here you have to include the header file with exactly the same name as your message in the /action folder (the Message.h is automatically generated from your Message.action file during compilation)

// this typedef just establishes the abbreviation SquareActionServer for the long data type
typedef actionlib::SimpleActionClient<autopnp_tool_change::MoveToSlotAction> MoveToSlotActionClient;

class MoveToSlotClient
{
public:
	MoveToSlotClient(ros::NodeHandle nh);
	bool init();
	void run();

protected:
	void doneCb(const actionlib::SimpleClientGoalState& state, const autopnp_tool_change::MoveToSlotResultConstPtr& result);
	void activeCb();
	void feedbackCb(const autopnp_tool_change::MoveToSlotFeedbackConstPtr& feedback);

	ros::NodeHandle node_;

	MoveToSlotActionClient move_to_slot_client;
};

#endif /* AUTOPNP_TOOL_CHANGE_CLIENT_H_ */

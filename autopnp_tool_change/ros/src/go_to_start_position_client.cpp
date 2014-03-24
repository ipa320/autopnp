/*
 * go_to_start_position_client.cpp
 *
 *  Created on: Mar 18, 2014
 *      Author: rmb-om
 */


#include "ros/ros.h"
#include <autopnp_tool_change/GoToStartPositionAction.h>
// actions
#include <actionlib/client/simple_action_client.h>



class GoToStartPositionClient
{
public:
	GoToStartPositionClient(ros::NodeHandle nh) :go_to_start_position_client("go_to_start_position", true)
{
		node_ = nh;
		ROS_INFO("initialize go_to_start_position_client");
}

	bool init()
	{
		std::cout << "Waiting for action server go_to_start_position to become available..." << std::endl;
		return go_to_start_position_client.waitForServer();
	}

	void run()
	{
		// prepare the goal message
		autopnp_tool_change::GoToStartPositionGoal goal;
		ROS_INFO("Sending goal to go_to_start_position_server");


		go_to_start_position_client.sendGoal(goal,
				boost::bind(&GoToStartPositionClient::doneCb, this, _1, _2),
				boost::bind(&GoToStartPositionClient::activeCb, this),
				boost::bind(&GoToStartPositionClient::feedbackCb, this, _1));

	}

	ros::NodeHandle node_;

	actionlib::SimpleActionClient<autopnp_tool_change::GoToStartPositionAction> go_to_start_position_client;
	// Called once when the goal completes.
	void doneCb(
			const actionlib::SimpleClientGoalState& state, const autopnp_tool_change::GoToStartPositionResultConstPtr& result)
	{
		ROS_INFO("Finished in state [%s]", state.toString().c_str());
		ROS_INFO("Result accomplished.");
	}
	// Called once when the goal becomes active.
	void activeCb()
	{
		ROS_INFO("Goal just went active");
	}

	// Called every time feedback is received for the goal.
	void feedbackCb(const autopnp_tool_change::GoToStartPositionFeedbackConstPtr& feedback)
	{
		ROS_INFO("Computation accomplished. Feedback.");
	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "go_to_start_position_client");
	ros::NodeHandle n;

	GoToStartPositionClient client(n);
	ROS_INFO("client started");
	// only proceed if the action server is available
	bool serverAvailable = client.init();

	if (serverAvailable == false)
	{
		std::cout << "The connection to the action server could not be established.\n" << std::endl;
		return -1;
	}
	std::cout << "The action server was found.\n" << std::endl;

	// start the client
	client.run();

	return 0;
}




#include "ros/ros.h"
#include "autopnp_tool_change/autopnp_tool_change_client.h"
#include "vector"

MoveToSlotClient::MoveToSlotClient(ros::NodeHandle nh)
:move_to_slot_client("autopnp_tool_change", true)
{
	node_ = nh;
}


bool MoveToSlotClient::init()
{
	// here we wait until the service is available;
	//please use the same service name as the one in the server; you may define a timeout if the service does not show up
	std::cout << "Waiting for action server to become available..." << std::endl;
	return move_to_slot_client.waitForServer(ros::Duration(7.0));
}


void MoveToSlotClient::run()
{
	// prepare the goal message
	autopnp_tool_change::MoveToSlotGoal goal;
	ROS_INFO("Sending goal");

	while(1) {
	//translation
	geometry_msgs::Vector3 vec;
	vec.x = 2.;
	vec.y = 3.;
	vec.z = 4.;
	//ROS_INFO("processing vec");
	//rotation
	geometry_msgs::Quaternion quat;
	//ROS_INFO("processing quat");
	geometry_msgs::Transform trans;
	//ROS_INFO("processing trans");
	trans.translation = vec;
	trans.rotation = quat;

	goal.transform = trans;

	// this calls the action server to process our goal message and send result message which will cause the execution of the doneCb callback function
	// this call is not blocking, i.e. this program can proceed immediately after the action call
	//ROS_INFO("processing goal");

	move_to_slot_client.sendGoal(goal,
			boost::bind(&MoveToSlotClient::doneCb, this, _1, _2),
			boost::bind(&MoveToSlotClient::activeCb, this),
			boost::bind(&MoveToSlotClient::feedbackCb, this, _1));

	}
}

// Called once when the goal completes
void MoveToSlotClient::doneCb(
		const actionlib::SimpleClientGoalState& state, const autopnp_tool_change::MoveToSlotResultConstPtr& result)
{
	ROS_INFO("Finished in state [%s]", state.toString().c_str());
	ROS_INFO("Result accomplished.");
}

// Called once when the goal becomes active
void MoveToSlotClient::activeCb()
{
	ROS_INFO("Goal just went active");
}

// Called every time feedback is received for the goal
void MoveToSlotClient::feedbackCb(const autopnp_tool_change::MoveToSlotFeedbackConstPtr& feedback)
{
	ROS_INFO("Computation accomplished. Feedback.");
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "autopnp_tool_change_client");
	ros::NodeHandle n;

	MoveToSlotClient client(n);

	// only proceed if the action server is available
	bool serverAvailable = client.init();

	if (serverAvailable == false)
	{
		std::cout << "The connection to the action server could not be established.\n" << std::endl;
		return -1;
	}
	std::cout << "The action server was found.\n" << std::endl;

	// start the interactive client
	client.run();

	return 0;
}



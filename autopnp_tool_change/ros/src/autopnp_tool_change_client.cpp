#include "ros/ros.h"
#include "autopnp_tool_change/autopnp_tool_change_client.h"
#include "vector"

MoveToWagonClient::MoveToWagonClient(ros::NodeHandle nh)
:move_to_wagon_client("tool_change", true)
{
	node_ = nh;
}

/*
 * Wait till the service is available.
 */
bool MoveToWagonClient::init()
{
	std::cout << "Waiting for action server to become available..." << std::endl;
	return move_to_wagon_client.waitForServer();
}

/*
 * Send the goal message to server.
 */
void MoveToWagonClient::run()
{
	// prepare the goal message
	autopnp_tool_change::MoveToWagonGoal goal;
	ROS_INFO("Sending goal");


	//translation
	/*geometry_msgs::Vector3 vec;
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
*/
	move_to_wagon_client.sendGoal(goal,
			boost::bind(&MoveToWagonClient::doneCb, this, _1, _2),
			boost::bind(&MoveToWagonClient::activeCb, this),
			boost::bind(&MoveToWagonClient::feedbackCb, this, _1));


}

// Called once when the goal completes.
void MoveToWagonClient::doneCb(
		const actionlib::SimpleClientGoalState& state, const autopnp_tool_change::MoveToWagonResultConstPtr& result)
{
	ROS_INFO("Finished in state [%s]", state.toString().c_str());
	ROS_INFO("Result accomplished.");
}

// Called once when the goal becomes active.
void MoveToWagonClient::activeCb()
{
	ROS_INFO("Goal just went active");
}

// Called every time feedback is received for the goal.
void MoveToWagonClient::feedbackCb(const autopnp_tool_change::MoveToWagonFeedbackConstPtr& feedback)
{
	ROS_INFO("Computation accomplished. Feedback.");
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "autopnp_tool_change_client");
	ros::NodeHandle n;

	MoveToWagonClient client(n);
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



/*
 * arm_fiducial_calibration_client.cpp
 *
 *  Created on: Mar 13, 2014
 *      Author: rmb-om
 */


#include "ros/ros.h"
// actions
#include <actionlib/client/simple_action_client.h>
#include <autopnp_tool_change/ArmFiducialCalibrationAction.h>


class ArmFiducialCalibrationClient
{
public:
	ArmFiducialCalibrationClient(ros::NodeHandle nh) :arm_fiducial_calibration_client("arm_fiducial_calibration", true)
{
		node_ = nh;
}

	bool init()
	{
		std::cout << "Waiting for action server to become available..." << std::endl;
		return arm_fiducial_calibration_client.waitForServer();
	}

	void run()
	{
		// prepare the goal message
		autopnp_tool_change::ArmFiducialCalibrationGoal goal;
		ROS_INFO("Sending goal");


		arm_fiducial_calibration_client.sendGoal(goal,
				boost::bind(&ArmFiducialCalibrationClient::doneCb, this, _1, _2),
				boost::bind(&ArmFiducialCalibrationClient::activeCb, this),
				boost::bind(&ArmFiducialCalibrationClient::feedbackCb, this, _1));


	}

	ros::NodeHandle node_;

	actionlib::SimpleActionClient<autopnp_tool_change::ArmFiducialCalibrationAction> arm_fiducial_calibration_client;
	// Called once when the goal completes.
	void doneCb(
			const actionlib::SimpleClientGoalState& state, const autopnp_tool_change::ArmFiducialCalibrationResultConstPtr& result)
	{
		ROS_INFO("Finished in state [%s]", state.toString().c_str());
		ROS_INFO("Result accomplished.");
	}
	/*
	 * A helper function to print out
	 * the PoseStamped msg.
	 */
	void printMsg(const geometry_msgs::PoseStamped pose)
	{
		std::cout<<" pose : "
				<<pose.pose.position.x<<","
				<<pose.pose.position.y<<","
				<<pose.pose.position.z<<","

				<<pose.pose.orientation.x<<","
				<<pose.pose.orientation.y<<","
				<<pose.pose.orientation.z<<","
				<<pose.pose.orientation.w<<","
				<<std::endl;

	}
	// Called once when the goal becomes active.
	void activeCb()
	{

		ROS_INFO("Goal just went active");
	}

	// Called every time feedback is received for the goal.
	void feedbackCb(const autopnp_tool_change::ArmFiducialCalibrationFeedbackConstPtr& feedback)
	{
		ROS_INFO("Computation accomplished. Feedback.");
	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "arm_fiducial_calibration_client");
	ros::NodeHandle n;

	ArmFiducialCalibrationClient client(n);
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



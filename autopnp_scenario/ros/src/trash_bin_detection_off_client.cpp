#include "ros/ros.h"
#include <autopnp_scenario/TrashBinDetection.h>
#include <autopnp_scenario/trash_bin_detection.h>

int main(int argc, char **argv)
{
	/**
	* The ros::init() function needs to see argc and argv so that it can perform
	* any ROS arguments and name remapping that were provided at the command line. For programmatic
	* remappings you can use a different version of init() which takes remappings
	* directly, but for most command-line programs, passing argc and argv is the easiest
	* way to do it.  The third argument to init() is the name of the node.
	*
	* You must call one of the versions of ros::init() before using any other
	* part of the ROS system.
	*/
	ros::init(argc, argv, "map_point_accessibility_check_client");

	/**
	* NodeHandle is the main access point to communications with the ROS system.
	* The first NodeHandle constructed will fully initialize this node, and the last
	* NodeHandle destructed will close down the node.
	*/
	ros::NodeHandle n;

	std::string trash_bin_detection_service_name = "trash_bin_detection_check";

	// here we wait until the service is available; please use the same service name as the one in the server; you may define a timeout if the service does not show up
	std::cout << "Waiting for service server to become available....." << std::endl;
	bool serviceAvailable = ros::service::waitForService(trash_bin_detection_service_name , 5000);

	// only proceed if the service is available
	if (serviceAvailable == false)
	{
		std::cout << "The services could not be found.\n" << std::endl;
		return -1;
	}
	std::cout << "The service servers are advertised.\n" << std::endl;


	// ===== example call to trash bin detection service =====
	autopnp_scenario::TrashBinDetection::Request turn_on_request;
	autopnp_scenario::TrashBinDetection::Response res_pose;

	turn_on_request.service_on_off_switch = false;

	// this calls the service server to process our request message and put the result into the response message
	// this call is blocking, i.e. this program will not proceed until the service server sends the response
	bool success = ros::service::call(trash_bin_detection_service_name, turn_on_request, res_pose);

	if (success == true)
	{
		ROS_INFO("trash bin detection off service is available");
	}
	else
		ROS_INFO("trash bin detection off service is not available");

	return 0;
}

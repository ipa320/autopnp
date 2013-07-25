#include "ros/ros.h"
#include "tf/tf.h"

// services - here you have to include the header file with exactly the same name as your message in the /srv folder (the Message.h is automatically generated from your Message.srv file during compilation)
#include <autopnp_dirt_detection/ActivateDirtDetection.h>
#include <autopnp_dirt_detection/DeactivateDirtDetection.h>
#include <autopnp_dirt_detection/GetDirtMap.h>
#include <autopnp_dirt_detection/ValidateCleaningResult.h>


void activateDirtDetection()
{
	// prepare the request and response messages
	autopnp_dirt_detection::ActivateDirtDetection::Request req;
	autopnp_dirt_detection::ActivateDirtDetection::Response res;

	// this calls the service server to process our request message and put the result into the response message
	// this call is blocking, i.e. this program will not proceed until the service server sends the response
	bool success = ros::service::call("/dirt_detection/activate_dirt_detection", req, res);

	if (success == true)
		std::cout << "Dirt detection successfully activated.\n" << std::endl;
	else
		std::cout << "The service call was not successful.\n" << std::endl;
}

void deactivateDirtDetection()
{
	// prepare the request and response messages
	autopnp_dirt_detection::DeactivateDirtDetection::Request req;
	autopnp_dirt_detection::DeactivateDirtDetection::Response res;

	// this calls the service server to process our request message and put the result into the response message
	// this call is blocking, i.e. this program will not proceed until the service server sends the response
	bool success = ros::service::call("/dirt_detection/deactivate_dirt_detection", req, res);

	if (success == true)
		std::cout << "Dirt detection successfully deactivated.\n" << std::endl;
	else
		std::cout << "The service call was not successful.\n" << std::endl;
}

void getDirtMap()
{
	// prepare the request and response messages
	autopnp_dirt_detection::GetDirtMap::Request req;
	autopnp_dirt_detection::GetDirtMap::Response res;

	// this calls the service server to process our request message and put the result into the response message
	// this call is blocking, i.e. this program will not proceed until the service server sends the response
	bool success = ros::service::call("/dirt_detection/get_dirt_map", req, res);

	if (success == true)
	{
		std::cout << "Received a dirt map with dirt spots at (x,y) in [m]:\n" << std::endl;

		double resolution = res.dirtMap.info.resolution;
		double origin_x = res.dirtMap.info.origin.position.x;
		double origin_y = res.dirtMap.info.origin.position.y;
		for (unsigned int v=0; v<res.dirtMap.info.height; ++v)
			for (unsigned int u=0; u<res.dirtMap.info.width; ++u)
				if (res.dirtMap.data[v*res.dirtMap.info.width + u] == 100)
					std::cout << "(" << u*resolution + origin_x << ", " << v*resolution + origin_y << ", 0.)\n";
	}
	else
		std::cout << "The service call was not successful.\n" << std::endl;
}

void validateCleaningResult()
{
	// prepare the request and response messages
	autopnp_dirt_detection::ValidateCleaningResult::Request req;
	autopnp_dirt_detection::ValidateCleaningResult::Response res;

	std::vector<tf::Vector3> dirtList;
	dirtList.push_back(tf::Vector3(3.1, -9.9, 0.));
	dirtList.push_back(tf::Vector3(2, -9.2, 0.));
	dirtList.push_back(tf::Vector3(2, -9.1, 0.));
	dirtList.push_back(tf::Vector3(2.4, -8.8, 0.));
	dirtList.push_back(tf::Vector3(2.9, -8.7, 0.));
	dirtList.push_back(tf::Vector3(2.9, -8.6, 0.));
	dirtList.push_back(tf::Vector3(2.8, -8.6, 0.));
	dirtList.push_back(tf::Vector3(3.3, -8.5, 0.));
	dirtList.push_back(tf::Vector3(3.2, -8.4, 0.));
	dirtList.push_back(tf::Vector3(3.3, -8.4, 0.));
	dirtList.push_back(tf::Vector3(2.8, -8.2, 0.));
	dirtList.push_back(tf::Vector3(2.9, -8.2, 0.));
	dirtList.push_back(tf::Vector3(2.9, -8.1, 0.));
	dirtList.push_back(tf::Vector3(3.4, -8.1, 0.));
	dirtList.push_back(tf::Vector3(2.9, -8, 0.));
	dirtList.push_back(tf::Vector3(3.2, -8, 0.));
	dirtList.push_back(tf::Vector3(3.3, -8, 0.));
	dirtList.push_back(tf::Vector3(3.4, -8, 0.));
	dirtList.push_back(tf::Vector3(3.5, -8, 0.));
	dirtList.push_back(tf::Vector3(2.9, -7.9, 0.));
	dirtList.push_back(tf::Vector3(3.2, -7.9, 0.));

	for (unsigned int i=7; i<10; ++i)
	{
		geometry_msgs::Point point;
		point.x = dirtList[i].getX();
		point.y = dirtList[i].getY();
		point.z = dirtList[i].getZ();
		req.validationPositions.push_back(point);
	}

	// this calls the service server to process our request message and put the result into the response message
	// this call is blocking, i.e. this program will not proceed until the service server sends the response
	bool success = ros::service::call("/dirt_detection/validate_cleaning_result", req, res);

	if (success == true)
	{
		std::cout << "Service call successful, still dirty spots are (x,y,z) in [m]:\n" << std::endl;
		for (unsigned int i=0; i<res.dirtyPositions.size(); ++i)
			std::cout << "(" << res.dirtyPositions[i].x << ", " << res.dirtyPositions[i].y << ", " << res.dirtyPositions[i].z << ")\n";
	}
	else
		std::cout << "The service call was not successful.\n" << std::endl;
}


/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
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
	ros::init(argc, argv, "dirt_detection_service_client");

	/**
	* NodeHandle is the main access point to communications with the ROS system.
	* The first NodeHandle constructed will fully initialize this node, and the last
	* NodeHandle destructed will close down the node.
	*/
	ros::NodeHandle n;

	// here we wait until the service is available; please use the same service name as the one in the server; you may define a timeout if the service does not show up
	std::cout << "Waiting for service server to become available..." << std::endl;
	bool serviceAvailable = true;
	serviceAvailable &= ros::service::waitForService("/dirt_detection/activate_dirt_detection", 5000);
	serviceAvailable &= ros::service::waitForService("/dirt_detection/deactivate_dirt_detection", 5000);
	serviceAvailable &= ros::service::waitForService("/dirt_detection/get_dirt_map", 5000);
	serviceAvailable &= ros::service::waitForService("/dirt_detection/validate_cleaning_result", 5000);

	// only proceed if the service is available
	if (serviceAvailable == false)
	{
		std::cout << "The service could not be found.\n" << std::endl;
		return -1;
	}
	std::cout << "The service server is advertised.\n" << std::endl;

	// show menu
	char key = '0';
	while(key != 'q')
	{
		std::cout << "\n\nDirt detection\n\n 1. dirt detection on\n 2. dirt detection off\n 3. get dirt map\n 4. validate cleaning results\n q. quit\n\nChoose for an option: ";
		std::cin >> key;
		std::cout << "\n\n";

		if (key == '1')
			activateDirtDetection();
		else if (key == '2')
			deactivateDirtDetection();
		else if (key == '3')
			getDirtMap();
		else if (key == '4')
			validateCleaningResult();
	}


	return 0;
}

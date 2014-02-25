#include <ros/ros.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <cob_srvs/Trigger.h>

class ToolchangePnPManager
{
private:
	ros::NodeHandle node_handle_;
	ros::Subscriber diagnostics_sub_;

public:
	ToolchangePnPManager(ros::NodeHandle nh)
	: node_handle_(nh)
	{
		diagnostics_sub_ = node_handle_.subscribe("/diagnostics", 1, &ToolchangePnPManager::diagnosticsCallback, this);
	}

	void diagnosticsCallback(const diagnostic_msgs::DiagnosticArray::ConstPtr& diagnostics_msg)
	{
		for (unsigned int i=0; i<diagnostics_msg->status.size(); ++i)
		{
			const diagnostic_msgs::DiagnosticStatus& status =  diagnostics_msg->status[i];
			if (status.name.compare("/sdh_controller") == 0)
			{
				if (status.message.compare("sdh initialized and running")==0 || status.level==diagnostic_msgs::DiagnosticStatus::OK)
				{
					std::cout << "SDH attached and ready." << std::endl;
				}
				else if (status.message.compare("sdh not initialized")==0 || status.level==diagnostic_msgs::DiagnosticStatus::WARN)
				{
					std::cout << "SDH not attached or not initialized." << std::endl;
				}
				else
					std::cout << "Unknown SDH status." << std::endl;
			}
		}
	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "toolchange_pnp_manager");

	ros::NodeHandle nh;
	ToolchangePnPManager tPnP(nh);

	// services
	std::string sdh_init_service_name = "/sdh_controller/init";

	// here we wait until the service is available; please use the same service name as the one in the server; you may define a timeout if the service does not show up
	std::cout << "Waiting for service servers to become available..." << std::endl;
	bool serviceAvailable = true;
	serviceAvailable &= ros::service::waitForService(sdh_init_service_name, 5000);
	if (serviceAvailable == false)
	{
		std::cout << "A service could not be found.\n" << std::endl;
		return -1;
	}
	std::cout << "The service servers are advertised.\n" << std::endl;

	// try to initialize possible attached devices periodically
	ros::Rate rate(0.5);	// in Hz
	while (ros::ok() == true)
	{
		// check for SDH
		cob_srvs::Trigger::Request req;
		cob_srvs::Trigger::Response res;
		bool success = ros::service::call(sdh_init_service_name, req, res);
		if (success == false)
			std::cout << "SDH init service call failed.\n" << std::endl;

		ros::spinOnce();
		rate.sleep();
	}
	return 0;
}


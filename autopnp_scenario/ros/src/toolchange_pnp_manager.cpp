#include <ros/ros.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <cob_srvs/Trigger.h>
#include <visualization_msgs/Marker.h>

class ToolchangePnPManager
{
private:
	ros::NodeHandle node_handle_;
	ros::Subscriber diagnostics_sdh_sub_;
	ros::Subscriber diagnostics_vacuum_sub_;
	ros::Publisher vis_pub_;
	bool vis_pub_first_;
	int hand_status_;			// -1=not attached, 0=initialized and attached
	int vacuum_status_;

public:
	ToolchangePnPManager(ros::NodeHandle nh)
	: node_handle_(nh)
	{
		vis_pub_first_ = true;
		hand_status_ = -1;
		vacuum_status_ = -1;
		//diagnostics_sdh_sub_ = node_handle_.subscribe("/diagnostics", 1, &ToolchangePnPManager::diagnosticsSDHCallback, this);
		diagnostics_vacuum_sub_ = node_handle_.subscribe("/diagnostics_vacuum_cleaner", 1, &ToolchangePnPManager::diagnosticsVacuumCallback, this);
		vis_pub_ = node_handle_.advertise<visualization_msgs::Marker>( "/pnp_manager", 0 );
	}

	void diagnosticsSDHCallback(const diagnostic_msgs::DiagnosticArray::ConstPtr& diagnostics_msg)
	{
		std::stringstream message_stream;
		for (unsigned int i=0; i<diagnostics_msg->status.size(); ++i)
		{
			const diagnostic_msgs::DiagnosticStatus& status =  diagnostics_msg->status[i];

			// check hand
			if (status.name.compare("/sdh_controller") == 0)
			{
				if (status.message.compare("sdh initialized and running")==0 || status.level==diagnostic_msgs::DiagnosticStatus::OK)
				{
					std::cout << "SDH attached and ready." << std::endl;
					message_stream << "Hand attached and ready." << std::endl;
					hand_status_ = 0;
				}
				else if (status.message.compare("sdh not initialized")==0 || status.level==diagnostic_msgs::DiagnosticStatus::WARN)
				{
					std::cout << "SDH not attached or not initialized." << std::endl;
					hand_status_ = -1;
				}
				else
				{
					std::cout << "Unknown SDH status." << std::endl;
					hand_status_ = -1;
				}
			}
		}

		visualization_msgs::Marker marker;
		marker.header.frame_id = "/base_link";
		marker.header.stamp = ros::Time();
		marker.ns = "pnp_manager";
		marker.id = 0;
		marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
		if (vis_pub_first_ == true)
			marker.action = visualization_msgs::Marker::ADD;
		else
			marker.action = visualization_msgs::Marker::MODIFY;
		marker.pose.position.x = 0;
		marker.pose.position.y = 0;
		marker.pose.position.z = 0.8;
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0.0;
		marker.pose.orientation.w = 1.0;
		marker.scale.x = 1;
		marker.scale.y = 0.1;
		marker.scale.z = 0.1;
		marker.color.a = 1.0;
		marker.color.r = 0.0;
		marker.color.g = 1.0;
		marker.color.b = 0.0;
		if (message_stream.str().compare("")==0)
		{
			marker.text = "No device attached";
			marker.color.r = 1.0;
			marker.color.g = 0.0;
			marker.color.b = 0.0;
		}
		else
			marker.text = message_stream.str();
		vis_pub_.publish( marker );
	}

	void diagnosticsVacuumCallback(const diagnostic_msgs::DiagnosticArray::ConstPtr& diagnostics_msg)
	{
		std::stringstream message_stream;
		for (unsigned int i=0; i<diagnostics_msg->status.size(); ++i)
		{
			const diagnostic_msgs::DiagnosticStatus& status =  diagnostics_msg->status[i];

			// check vacuum cleaner
			if (status.name.compare("vacuum_cleaner_controller") == 0)
			{
				if (status.message.compare("canopen_staubsauger chain initialized and running")==0 || status.level==diagnostic_msgs::DiagnosticStatus::OK)
				{
					std::cout << "Vacuum cleaner attached and ready." << std::endl;
					message_stream << "Vacuum cleaner attached and ready." << std::endl;
					vacuum_status_ = 0;
				}
				else if (status.message.compare("Fault occured.")==0 || status.level==diagnostic_msgs::DiagnosticStatus::ERROR)
				{
					std::cout << "Vacuum cleaner not attached or not initialized." << std::endl;
					vacuum_status_ = -1;
				}
				else
				{
					std::cout << "Unknown SDH status." << std::endl;
					vacuum_status_ = -1;
				}
			}
		}

		visualization_msgs::Marker marker;
		marker.header.frame_id = "/base_link";
		marker.header.stamp = ros::Time();
		marker.ns = "pnp_manager";
		marker.id = 0;
		marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
		if (vis_pub_first_ == true)
			marker.action = visualization_msgs::Marker::ADD;
		else
			marker.action = visualization_msgs::Marker::MODIFY;
		marker.pose.position.x = 0;
		marker.pose.position.y = 0;
		marker.pose.position.z = 0.8;
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0.0;
		marker.pose.orientation.w = 1.0;
		marker.scale.x = 1;
		marker.scale.y = 0.1;
		marker.scale.z = 0.1;
		marker.color.a = 1.0;
		marker.color.r = 0.0;
		marker.color.g = 1.0;
		marker.color.b = 0.0;
		if (message_stream.str().compare("")==0)
		{
			marker.text = "No device attached";
			marker.color.r = 1.0;
			marker.color.g = 0.0;
			marker.color.b = 0.0;
		}
		else
			marker.text = message_stream.str();
		vis_pub_.publish( marker );
	}


	int getHandStatus()
	{
		return hand_status_;
	}

	int setHandStatus(int val)
	{
		hand_status_ = val;
	}

	int getVacuumStatus()
	{
		return vacuum_status_;
	}

	int setVacuumStatus(int val)
	{
		vacuum_status_ = val;
	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "toolchange_pnp_manager");

	ros::NodeHandle nh;
	ToolchangePnPManager tPnP(nh);

	// services
	std::string sdh_init_service_name = "/sdh_controller/init";
	std::string vacuum_init_service_name = "/vacuum_cleaner_controller/init";

	// here we wait until the service is available; please use the same service name as the one in the server; you may define a timeout if the service does not show up
	std::cout << "Waiting for service servers to become available..." << std::endl;
	bool serviceAvailable = true;
	//serviceAvailable &= ros::service::waitForService(sdh_init_service_name, 5000);
	serviceAvailable &= ros::service::waitForService(vacuum_init_service_name, 5000);
	if (serviceAvailable == false)
	{
		std::cout << "A service could not be found.\n" << std::endl;
		return -1;
	}
	std::cout << "The service servers are advertised.\n" << std::endl;

	// try to initialize possible attached devices periodically
	ros::Rate rate(0.5);	// in Hz
	int lastCheckedDevice = 0;
	while (ros::ok() == true)
	{
		// check for vacuum cleaner
		if (tPnP.getVacuumStatus()==-1 && tPnP.getHandStatus()==-1 && lastCheckedDevice==0)
		{
			cob_srvs::Trigger::Request req;
			cob_srvs::Trigger::Response res;
			bool success = ros::service::call(vacuum_init_service_name, req, res);
			if (success == false)
				std::cout << "Vacuum cleaner init service call failed.\n" << std::endl;
			tPnP.setVacuumStatus(1);
			//lastCheckedDevice = 1;
		}

		// check for SDH
//		if (tPnP.getHandStatus()== -1 && tPnP.getVacuumStatus()== -1 && lastCheckedDevice==1)
//		{
//			cob_srvs::Trigger::Request req;
//			cob_srvs::Trigger::Response res;
//			bool success = ros::service::call(sdh_init_service_name, req, res);
//			if (success == false)
//				std::cout << "SDH init service call failed.\n" << std::endl;
//			tPnP.setHandStatus(1);
//			lastCheckedDevice = 0;
//		}

		ros::spinOnce();
		rate.sleep();
	}
	return 0;
}


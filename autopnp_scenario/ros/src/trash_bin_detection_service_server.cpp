#include <autopnp_scenario/trash_bin_detection.h>

//Initialize Trash bin detection to receive necessary raw data
void trash_bin_detection::fiducialsInit(ros::NodeHandle& nh)
{
	{
			boost::mutex::scoped_lock lock(mutex_subscription_data_);
//	fiducials_data_recieved_ = false;
	ROS_INFO("TrashBinDetectionServer: Waiting to receive data...");
	fiducials_msg_sub_ = nh.subscribe<cob_object_detection_msgs::Detection>("/fiducials/detect_fiducials", 1, &trash_bin_detection::fiducialsDataCallback,this);
//	while (fiducials_data_recieved_ == false)
//		ros::spinOnce();
	ROS_INFO("TrashBinDetectionCheck: data received.");
	}
	trash_bin_detection_check_server_ = nh.advertiseService("trash_bin_detection_check", &trash_bin_detection::TrashBinDetectionCallback, this);
}

//fiducials topic data call-back
void trash_bin_detection::fiducialsDataCallback(const cob_object_detection_msgs::Detection::ConstPtr& fiducials_msg_data)
{
	fiducials_frame_id_ = fiducials_msg_data->header.frame_id;
	image_detection_label_ = fiducials_msg_data->label;
	pose_with_respect_to_fiducials_frame_id_ = fiducials_msg_data->pose;
//	fiducials_data_recieved_ = true;
//	fiducials_msg_sub_.shutdown();
}

//This function process the data to calculate the trash bin location
bool trash_bin_detection::TrashBinDetectionCallback(autopnp_scenario::TrashBinDetection::Request &req, autopnp_scenario::TrashBinDetection::Response &res)
{
	if (req.service_on_off_switch == true)
	{
		ROS_INFO("Received request to turn-on trash bin detection.....");
		if (image_detection_label_ == "0")
		{
			ROS_INFO("Trash bin detection is turned-on.");
			tf::TransformListener listener_;
			tf::Stamped<tf::Pose> original_pose(
					tf::Pose(
							tf::Quaternion(
									pose_with_respect_to_fiducials_frame_id_.pose.orientation.x,
									pose_with_respect_to_fiducials_frame_id_.pose.orientation.y,
									pose_with_respect_to_fiducials_frame_id_.pose.orientation.z,
									pose_with_respect_to_fiducials_frame_id_.pose.orientation.w),
							tf::Vector3(
									pose_with_respect_to_fiducials_frame_id_.pose.position.x,
									pose_with_respect_to_fiducials_frame_id_.pose.position.y,
									pose_with_respect_to_fiducials_frame_id_.pose.position.z)),
					ros::Time(0), fiducials_frame_id_);
			tf::Stamped<tf::Pose> transformed_pose;
			listener_.transformPose("map", original_pose, transformed_pose);

			pose_with_respect_to_map_.pose.position.x = transformed_pose.getOrigin().x();
			pose_with_respect_to_map_.pose.position.y = transformed_pose.getOrigin().y();
			pose_with_respect_to_map_.pose.position.z = transformed_pose.getOrigin().z();
			pose_with_respect_to_map_.pose.orientation.x = transformed_pose.getRotation().x();
			pose_with_respect_to_map_.pose.orientation.y = transformed_pose.getRotation().y();
			pose_with_respect_to_map_.pose.orientation.z = transformed_pose.getRotation().z();
			pose_with_respect_to_map_.pose.orientation.w = transformed_pose.getRotation().w();
			pose_array_.push_back(pose_with_respect_to_map_);
			unsigned int container_value_checker = pose_array_.size() - 1;
			if (container_value_checker > 1)
			{
				if (similarity_checker(pose_array_[container_value_checker], pose_array_[container_value_checker - 1]) == true)
				{
					pose_array_[container_value_checker] = average_calculator(pose_array_[container_value_checker], pose_array_[container_value_checker - 1]);
					res.trash_bin_locations.push_back(pose_array_[container_value_checker]);
				}
			}
			else res.trash_bin_locations.push_back(pose_array_[container_value_checker]);
		}
	}
	else
	{
		ROS_INFO("Received request to turn-off trash bin detection.....");
		ROS_INFO("Trash bin detection is turned-off.");
		pose_array_.clear();
	}

	return true;
}

//This function checks the similarity between the two consecutive pose
bool trash_bin_detection::similarity_checker(geometry_msgs::PoseStamped &present_value, geometry_msgs::PoseStamped &past_value)
{
	if ((abs(present_value.pose.position.x - past_value.pose.position.x)) < 0.09 && (abs(present_value.pose.position.y - past_value.pose.position.y)) < 0.09
			&& (abs(present_value.pose.position.z - past_value.pose.position.z)) < 0.09 && (abs(present_value.pose.orientation.x - past_value.pose.orientation.x)) < 0.09
			&& (abs(present_value.pose.orientation.y - past_value.pose.orientation.y)) < 0.09 && (abs(present_value.pose.orientation.z - past_value.pose.orientation.z)) < 0.09
			&& (abs(present_value.pose.orientation.w - past_value.pose.orientation.w)) < 0.09)
	{
		return true;
	}
	else
		return false;
}

geometry_msgs::PoseStamped trash_bin_detection::average_calculator(geometry_msgs::PoseStamped &present_value, geometry_msgs::PoseStamped &past_value)
{
	geometry_msgs::PoseStamped average_value;
	average_value.pose.position.x = (present_value.pose.position.x + past_value.pose.position.x) / 2;
	average_value.pose.position.y = (present_value.pose.position.x + past_value.pose.position.x) / 2;
	average_value.pose.position.z = (present_value.pose.position.x + past_value.pose.position.x) / 2;
	average_value.pose.orientation.x = (present_value.pose.orientation.x + past_value.pose.orientation.x) / 2;
	average_value.pose.orientation.y = (present_value.pose.orientation.y + past_value.pose.orientation.y) / 2;
	average_value.pose.orientation.z = (present_value.pose.orientation.z + past_value.pose.orientation.z) / 2;
	average_value.pose.orientation.w = (present_value.pose.orientation.w + past_value.pose.orientation.w) / 2;
	return average_value;
}

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
	ros::init(argc, argv, "trash_bin_detection_server");
	/**
	 * NodeHandle is the main access point to communications with the ROS system.
	 * The first NodeHandle constructed will fully initialize this node, and the last
	 * NodeHandle destructed will close down the node.
	 */
	ros::NodeHandle nh;
	trash_bin_detection bin_detection_object;
	bin_detection_object.fiducialsInit(nh);
	//trash_bin_detection_check_server_ = nh.advertiseService("trash_bin_detection_check", &trash_bin_detection::TrashBinDetectionCallback, this);
	ROS_INFO("Trash Bin detection Server initialized.....");
	ros::spin();
	return 0;
}

#include <autopnp_scenario/trash_bin_detection.h>

#define __DEBUG_DISPLAYS__

//constructor Initialization
trash_bin_detection::trash_bin_detection()
{
	trash_bin_detection_active_ = false ;
}

//Initialize Trash bin detection to receive necessary raw data
void trash_bin_detection::fiducials_init_(ros::NodeHandle& nh)
{
	{
		boost::mutex::scoped_lock lock(mutex_subscription_data_);
		ROS_INFO("FiducialsDetectionServer: Waiting to receive data.....");
		fiducials_msg_sub_ = nh.subscribe<cob_object_detection_msgs::DetectionArray>("/fiducials/detect_fiducials", 1, &trash_bin_detection::fiducials_data_callback_,this);
		ROS_INFO("FiducialsDetectionCheck: data received.");
	}
	activate_trash_bin_detection_server_ = nh.advertiseService("activate_trash_bin_detection_service", &trash_bin_detection::activate_trash_bin_detection_callback_, this);
	deactivate_trash_bin_detection_server_ = nh.advertiseService("deactivate_trash_bin_detection_service", &trash_bin_detection::deactivate_trash_bin_detection_callback_, this);
	//trash_bin_poses-> So you have to take the data from this topic
	trash_bin_location_publisher_ = nh.advertise<autopnp_scenario::TrashBinDetection>("trash_bin_poses", 1000 , this);
}

//fiducials topic data call-back
void trash_bin_detection::fiducials_data_callback_(const cob_object_detection_msgs::DetectionArray::ConstPtr& fiducials_msg_data)
{
	for (unsigned int loop_counter=0; loop_counter<fiducials_msg_data->detections.size(); loop_counter++)
	{
		if (fiducials_msg_data->detections.size() == 0)
		{
			ROS_INFO("No markers detected.\n");
			return;
		}
		else if(fiducials_msg_data->detections[loop_counter].label == "tag_0" )
		{
			fiducials_frame_id_ = fiducials_msg_data->detections[loop_counter].header.frame_id;
			pose_with_respect_to_fiducials_frame_id_= fiducials_msg_data->detections[loop_counter].pose;
			trash_bin_pose_estimator_();
		}
	}
}

//This function process the data to calculate the trash bin location
void trash_bin_detection::trash_bin_pose_estimator_()
{
	if (trash_bin_detection_active_ == false)
	{
		pose_array_.clear();
		trash_bin_location_storage_.trash_bin_locations.clear();
		return;
	}

	else
	{
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
		//todo: Have to give correct frame id->
		listener_.transformPose(fiducials_frame_id_, original_pose, transformed_pose);
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
			if (similarity_checker_(pose_array_[container_value_checker], pose_array_[container_value_checker - 1],0.09) == true)
			{
				pose_array_[container_value_checker] = average_calculator_(pose_array_[container_value_checker], pose_array_[container_value_checker - 1]);
				trash_bin_location_storage_.trash_bin_locations.push_back(pose_array_[container_value_checker]);
#ifdef __DEBUG_DISPLAYS__
	ROS_INFO("trash_bin_locations size: %d",trash_bin_location_storage_.trash_bin_locations.size());
#endif
			}
		}
		else
		{
			trash_bin_location_storage_.trash_bin_locations.push_back(pose_array_[container_value_checker]);
#ifdef __DEBUG_DISPLAYS__
	ROS_INFO("trash_bin_locations size: %d",trash_bin_location_storage_.trash_bin_locations.size());
#endif
		}

		trash_bin_location_publisher_.publish(trash_bin_location_storage_);
	}
}

//This function activate the trash bin detection service
bool trash_bin_detection::activate_trash_bin_detection_callback_(
		autopnp_scenario::ActivateTrashBinDetection::Request &req,
		autopnp_scenario::ActivateTrashBinDetection::Response &res) {

	ROS_INFO("Received request to turn-on trash bin detection.....");
	ROS_INFO("Trash bin detection is turned-on.");

	trash_bin_detection_active_ = true;

	return true;
}

//This function deactivate the trash bin detection service
bool trash_bin_detection::deactivate_trash_bin_detection_callback_(
		autopnp_scenario::DeactivateTrashBinDetection::Request &req,
		autopnp_scenario::DeactivateTrashBinDetection::Response &res) {

	ROS_INFO("Received request to turn-off trash bin detection.....");
	ROS_INFO("Trash bin detection is turned-off.");

	trash_bin_detection_active_ = false;

	return true;
}


//This function checks the similarity between the two consecutive pose
bool trash_bin_detection::similarity_checker_(geometry_msgs::PoseStamped &present_value, geometry_msgs::PoseStamped &past_value , double difference_value )
{
	if ((abs(present_value.pose.position.x - past_value.pose.position.x))
			< difference_value
			&& (abs(present_value.pose.position.y - past_value.pose.position.y))
					< difference_value
			&& (abs(present_value.pose.position.z - past_value.pose.position.z))
					< difference_value
			&& (abs(
					present_value.pose.orientation.x
							- past_value.pose.orientation.x)) < difference_value
			&& (abs(
					present_value.pose.orientation.y
							- past_value.pose.orientation.y)) < difference_value
			&& (abs(
					present_value.pose.orientation.z
							- past_value.pose.orientation.z)) < difference_value
			&& (abs(
					present_value.pose.orientation.w
							- past_value.pose.orientation.w))
					< difference_value)
	{
		return true;
	}
	else
		return false;
}

//This function calculate the average between the two pose
geometry_msgs::PoseStamped trash_bin_detection::average_calculator_(geometry_msgs::PoseStamped &present_value, geometry_msgs::PoseStamped &past_value)
{
	geometry_msgs::PoseStamped average_value;
	average_value.pose.position.x = (present_value.pose.position.x
			+ past_value.pose.position.x) / 2;
	average_value.pose.position.y = (present_value.pose.position.y
			+ past_value.pose.position.y) / 2;
	average_value.pose.position.z = (present_value.pose.position.z
			+ past_value.pose.position.z) / 2;
	average_value.pose.orientation.x = (present_value.pose.orientation.x
			+ past_value.pose.orientation.x) / 2;
	average_value.pose.orientation.y = (present_value.pose.orientation.y
			+ past_value.pose.orientation.y) / 2;
	average_value.pose.orientation.z = (present_value.pose.orientation.z
			+ past_value.pose.orientation.z) / 2;
	average_value.pose.orientation.w = (present_value.pose.orientation.w
			+ past_value.pose.orientation.w) / 2;
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
	bin_detection_object.fiducials_init_(nh);
	ROS_INFO("Trash Bin detection Server initialized.....");
	ros::spin();
	return 0;
}

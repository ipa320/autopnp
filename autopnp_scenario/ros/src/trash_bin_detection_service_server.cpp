#include <autopnp_scenario/trash_bin_detection.h>

//#define __DEBUG_DISPLAYS__

//constructor Initialization
TrashBinDetectionNode::TrashBinDetectionNode(ros::NodeHandle& nh)
: //grasp_trash_bin_server_(nh, "grasp_trash_bin", boost::bind(&TrashBinDetectionNode::graspTrashBin, this, _1), false),	// this initializes the action server; important: always set the last parameter to false
  nh_(nh), listener_(nh, ros::Duration(40.0))
 // sdh_follow_joint_client_("/sdh_controller/follow_joint_trajectory", true)
{
	std::cout << "\n--------------------------------------\nTrash Bin Detection Parameters:\n--------------------------------------\n";
	nh_.param("trash_bin_detection_server/publish_marker_array", publish_marker_array_, false);
	std::cout << "publish_marker_array = " << publish_marker_array_ << std::endl;
	nh_.param("trash_bin_detection_server/trash_bin_radius", trash_bin_radius_, 0.15);
	std::cout << "trash_bin_radius = " << trash_bin_radius_ << std::endl;

	prev_marker_array_size_ = 0;
}

//Initialize Trash bin detection to receive necessary raw data
void TrashBinDetectionNode::init(ros::NodeHandle& nh)
{
	detect_trash_bin_again_server_ = nh.advertiseService("detect_trash_bin_again_service", &TrashBinDetectionNode::detect_trash_bin_again_callback_,this);
	activate_trash_bin_detection_server_ = nh.advertiseService("activate_trash_bin_detection_service", &TrashBinDetectionNode::activate_trash_bin_detection_callback_, this);
	deactivate_trash_bin_detection_server_ = nh.advertiseService("deactivate_trash_bin_detection_service", &TrashBinDetectionNode::deactivate_trash_bin_detection_callback_, this);
	//trash_bin_poses-> So you have to take the data from this topic
	trash_bin_location_publisher_ = nh.advertise<autopnp_scenario::TrashBinDetection>("trash_bin_poses", 1, this);

	if (publish_marker_array_ == true)
		fiducials_marker_array_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>( "trash_bin_locations", 0 );

	//grasp_trash_bin_server_.start();
	//sdh_follow_joint_client_.waitForServer();

	ROS_INFO("TrashBinDetectionNode: initialized.");
}

//fiducials topic data call-back
void TrashBinDetectionNode::fiducials_data_callback_(const cob_object_detection_msgs::DetectionArray::ConstPtr& fiducials_msg_data)
{
	for (unsigned int i=0; i<fiducials_msg_data->detections.size(); i++)
	{
		if (fiducials_msg_data->detections.size() == 0)
		{
			ROS_INFO("No markers detected.\n");
			return;
		}
		else
		{
			tag_label_name_ = fiducials_msg_data->detections[i].label;
			fiducials_pose_ = fiducials_msg_data->detections[i].pose;

			if (fiducials_msg_data->detections[i].label.compare("tag_25")==0)
			{
				std::string frame_id = fiducials_msg_data->detections[i].header.frame_id;
				trash_bin_pose_estimator_(fiducials_msg_data->detections[i].pose, frame_id);
			}
		}
	}
}

//This function process the data to calculate the trash bin location
void TrashBinDetectionNode::trash_bin_pose_estimator_(const geometry_msgs::PoseStamped& pose_from_fiducials_frame_id, std::string& frame_id)
{
	tf::Stamped<tf::Pose> original_pose(
	tf::Pose(
			tf::Quaternion(
					pose_from_fiducials_frame_id.pose.orientation.x,
					pose_from_fiducials_frame_id.pose.orientation.y,
					pose_from_fiducials_frame_id.pose.orientation.z,
					pose_from_fiducials_frame_id.pose.orientation.w),
			tf::Vector3(
					pose_from_fiducials_frame_id.pose.position.x,
					pose_from_fiducials_frame_id.pose.position.y,
					pose_from_fiducials_frame_id.pose.position.z+trash_bin_radius_)), //-
			ros::Time(0), frame_id);
	tf::Stamped<tf::Pose> transformed_pose;
	try
	{
		listener_.waitForTransform("/map", original_pose.frame_id_, original_pose.stamp_, ros::Duration(5.0));
		listener_.transformPose("/map", original_pose, transformed_pose);
//		std::cout << "check: tranformed_pose.frameid: " << transformed_pose.frame_id_ << std::endl;
//		transformed_pose.frame_id_ = "/map";
	}
	catch (tf::TransformException& ex)
	{
		ROS_ERROR("%s",ex.what());
		return;
	}
	geometry_msgs::PoseStamped pose_with_respect_to_map;
	pose_with_respect_to_map.pose.position.x = transformed_pose.getOrigin().x();
	pose_with_respect_to_map.pose.position.y = transformed_pose.getOrigin().y();
	pose_with_respect_to_map.pose.position.z = transformed_pose.getOrigin().z();
	pose_with_respect_to_map.pose.orientation.x = transformed_pose.getRotation().x();
	pose_with_respect_to_map.pose.orientation.y = transformed_pose.getRotation().y();
	pose_with_respect_to_map.pose.orientation.z = transformed_pose.getRotation().z();
	pose_with_respect_to_map.pose.orientation.w = transformed_pose.getRotation().w();
	pose_with_respect_to_map.header.frame_id = transformed_pose.frame_id_;
	pose_with_respect_to_map.header.stamp = transformed_pose.stamp_;
	pose_with_respect_to_map.header.seq = pose_from_fiducials_frame_id.header.seq;

	if (trash_bin_location_storage_.trash_bin_locations.size() > 0)
	{
		for (unsigned int i=0; i<trash_bin_location_storage_.trash_bin_locations.size(); ++i)
		{
			if (similarity_checker_(pose_with_respect_to_map, trash_bin_location_storage_.trash_bin_locations[i], 0.3) == true)
			{
				trash_bin_location_storage_.trash_bin_locations[i] = average_calculator_(pose_with_respect_to_map, trash_bin_location_storage_.trash_bin_locations[i], trash_bin_location_average_count_[i]);
				trash_bin_location_average_count_[i]++;
				ROS_INFO("Updated trash bin location of entry %i.", i);
				break;
			}
			else if (i == (trash_bin_location_storage_.trash_bin_locations.size() - 1))
			{
				trash_bin_location_storage_.trash_bin_locations.push_back(pose_with_respect_to_map);
				trash_bin_location_average_count_.push_back(1);
				ROS_INFO("Added a new trash bin location at xyz=(%f,%f,%f)m.", pose_with_respect_to_map.pose.position.x, pose_with_respect_to_map.pose.position.y, pose_with_respect_to_map.pose.position.z);
			}
		}
	}
	else
	{
		trash_bin_location_storage_.trash_bin_locations.push_back(pose_with_respect_to_map);
		trash_bin_location_average_count_.push_back(1);
		ROS_INFO("Added a new trash bin location at xyz=(%f,%f,%f)m.", pose_with_respect_to_map.pose.position.x, pose_with_respect_to_map.pose.position.y, pose_with_respect_to_map.pose.position.z);
	}

	trash_bin_location_publisher_.publish(trash_bin_location_storage_);
}

bool TrashBinDetectionNode::detect_trash_bin_again_callback_(autopnp_scenario::DetectFiducials::Request &req, autopnp_scenario::DetectFiducials::Response &res)
{
	ROS_WARN("call to detect trash bin again callback");
	ROS_INFO("Received request to detect trash bin.....");

	if((tag_label_name_ == req.tag_name))
	{
		res.waste_bin_location = fiducials_pose_ ;
	}

	return true;
}

//This function activate the trash bin detection service
bool TrashBinDetectionNode::activate_trash_bin_detection_callback_(autopnp_scenario::ActivateTrashBinDetection::Request &req, autopnp_scenario::ActivateTrashBinDetection::Response &res)
{
	ROS_INFO("Received request to turn on trash bin detection.....");

	trash_bin_location_storage_.trash_bin_locations.clear();
	trash_bin_location_average_count_.clear();

	ROS_INFO("FiducialsDetectionServer: Waiting to receive data.....");
	fiducials_msg_sub_ = nh_.subscribe<cob_object_detection_msgs::DetectionArray>("/fiducials/detect_fiducials", 1, &TrashBinDetectionNode::fiducials_data_callback_,this);
	ROS_INFO("FiducialsDetectionCheck: data received.");

	ROS_INFO("Trash bin detection is turned on.");

	return true;
}

//This function deactivate the trash bin detection service
bool TrashBinDetectionNode::deactivate_trash_bin_detection_callback_(autopnp_scenario::DeactivateTrashBinDetection::Request &req, autopnp_scenario::DeactivateTrashBinDetection::Response &res)
{
	ROS_INFO("Received request to turn off trash bin detection.....");
	fiducials_msg_sub_.shutdown();
	ROS_INFO("Trash bin detection is turned off.");

	cob_object_detection_msgs::Detection temp_detection_obj;
	for(unsigned int i=0; i<trash_bin_location_storage_.trash_bin_locations.size(); i++)
	{
		//temp_detection_obj.pose.pose = trash_bin_location_storage_.trash_bin_locations[i].pose;
		temp_detection_obj.pose = trash_bin_location_storage_.trash_bin_locations[i];
		temp_detection_obj.header =  trash_bin_location_storage_.trash_bin_locations[i].header;
		res.detected_trash_bin_poses.detections.push_back(temp_detection_obj);
	}
	res.detected_trash_bin_poses.header.stamp = ros::Time::now();
	if (trash_bin_location_storage_.trash_bin_locations.size() > 0)
		res.detected_trash_bin_poses.header.frame_id = trash_bin_location_storage_.trash_bin_locations[0].header.frame_id;

    // Publish marker array
    if (publish_marker_array_ == true)
    {
        // 3 arrows for each coordinate system of each detected fiducial
    	unsigned int pose_array_size = res.detected_trash_bin_poses.detections.size();
        unsigned int marker_array_size = 3*pose_array_size;
        if (marker_array_size >= prev_marker_array_size_)
        {
            marker_array_msg_.markers.resize(marker_array_size);
        }

		// publish a coordinate system from arrow markers for each object
		for (unsigned int i = 0; i < pose_array_size; i++) {
			for (unsigned int j = 0; j < 3; j++) {
				unsigned int idx = 3 * i + j;
				marker_array_msg_.markers[idx].header = res.detected_trash_bin_poses.header;
				marker_array_msg_.markers[idx].header.frame_id = res.detected_trash_bin_poses.detections[i].header.frame_id; // "/" + frame_id;//"tf_name.str()";
				marker_array_msg_.markers[idx].header.stamp = res.detected_trash_bin_poses.detections[i].header.stamp;
				marker_array_msg_.markers[idx].ns = "fiducials";
				marker_array_msg_.markers[idx].id = idx;
				marker_array_msg_.markers[idx].type = visualization_msgs::Marker::ARROW;
				marker_array_msg_.markers[idx].action = visualization_msgs::Marker::ADD;
				marker_array_msg_.markers[idx].color.a = 0.85;
				marker_array_msg_.markers[idx].color.r = 0;
				marker_array_msg_.markers[idx].color.g = 0;
				marker_array_msg_.markers[idx].color.b = 0;

				marker_array_msg_.markers[idx].points.resize(2);
				marker_array_msg_.markers[idx].points[0].x = 0.0;
				marker_array_msg_.markers[idx].points[0].y = 0.0;
				marker_array_msg_.markers[idx].points[0].z = 0.0;
				marker_array_msg_.markers[idx].points[1].x = 0.0;
				marker_array_msg_.markers[idx].points[1].y = 0.0;
				marker_array_msg_.markers[idx].points[1].z = 0.0;

				if (j == 0) {
					marker_array_msg_.markers[idx].points[1].x = 0.2;
					marker_array_msg_.markers[idx].color.r = 255;
				} else if (j == 1) {
					marker_array_msg_.markers[idx].points[1].y = 0.2;
					marker_array_msg_.markers[idx].color.g = 255;
				} else if (j == 2) {
					marker_array_msg_.markers[idx].points[1].z = 0.2;
					marker_array_msg_.markers[idx].color.b = 255;
				}

				marker_array_msg_.markers[idx].pose = res.detected_trash_bin_poses.detections[i].pose.pose;

				ros::Duration one_hour = ros::Duration(3600); // 1 second
				marker_array_msg_.markers[idx].lifetime = one_hour;
				marker_array_msg_.markers[idx].scale.x = 0.01; // shaft diameter
				marker_array_msg_.markers[idx].scale.y = 0.015; // head diameter
				marker_array_msg_.markers[idx].scale.z = 0; // head length 0=default
			}

			if (prev_marker_array_size_ > marker_array_size)
			{
				for (unsigned int i = marker_array_size; i < prev_marker_array_size_; ++i)
				{
					marker_array_msg_.markers[i].action = visualization_msgs::Marker::DELETE;
				}
			}
			prev_marker_array_size_ = marker_array_size;

			fiducials_marker_array_publisher_.publish(marker_array_msg_);
		}
	} // End: publish markers

	// hack for demo in holland
//	if (trash_bin_location_storage_.trash_bin_locations.size() == 0)
//	{
//		temp_detection_obj.pose.header.frame_id = "/map";
//		temp_detection_obj.pose.pose.position.x = 0.0;
//		temp_detection_obj.pose.pose.position.y = 0.0;
//		temp_detection_obj.pose.pose.position.z = 0.0;
//		temp_detection_obj.header.frame_id = "/map";
//		res.detected_trash_bin_poses.detections.push_back(temp_detection_obj);
//	}

	return true;
}

//This function checks the similarity between the two consecutive pose
bool TrashBinDetectionNode::similarity_checker_(geometry_msgs::PoseStamped &present_value, geometry_msgs::PoseStamped &past_value , double difference_value )
{
	double dx = present_value.pose.position.x - past_value.pose.position.x;
	double dy = present_value.pose.position.y - past_value.pose.position.y;
	double dz = present_value.pose.position.z - past_value.pose.position.z;
	if (sqrt(dx*dx + dy*dy + dz*dz) < difference_value)
		return true;
	else
		return false;
}

//This function calculate the average between the two pose
geometry_msgs::PoseStamped TrashBinDetectionNode::average_calculator_(geometry_msgs::PoseStamped &present_value, geometry_msgs::PoseStamped &past_value, const int averaged_numbers)
{
	double factor = 1./(1.+averaged_numbers);
	geometry_msgs::PoseStamped average_value;
	average_value.pose.position.x = (present_value.pose.position.x + past_value.pose.position.x*averaged_numbers) * factor;
	average_value.pose.position.y = (present_value.pose.position.y + past_value.pose.position.y*averaged_numbers) * factor;
	average_value.pose.position.z = (present_value.pose.position.z + past_value.pose.position.z*averaged_numbers) * factor;
	average_value.pose.orientation.x = (present_value.pose.orientation.x + past_value.pose.orientation.x*averaged_numbers) * factor;
	average_value.pose.orientation.y = (present_value.pose.orientation.y + past_value.pose.orientation.y*averaged_numbers) * factor;
	average_value.pose.orientation.z = (present_value.pose.orientation.z + past_value.pose.orientation.z*averaged_numbers) * factor;
	average_value.pose.orientation.w = (present_value.pose.orientation.w + past_value.pose.orientation.w*averaged_numbers) * factor;

	average_value.header = present_value.header;

	return average_value;
}

//void TrashBinDetectionNode::graspTrashBin(const autopnp_scenario::GraspTrashBinGoalConstPtr& goal)
//{
//	ROS_INFO("Grasping trash bin ...");
///*
//	// open hand
////header:
////  seq: 6
////  stamp:
////    secs: 156
////    nsecs: 580000000
////  frame_id: ''
////goal_id:
////  stamp:
////    secs: 156
////    nsecs: 580000000
////  id: /cob_console-6-156.580
////goal:
////  trajectory:
////    header:
////      seq: 0
////      stamp:
////        secs: 157
////        nsecs: 80000000
////      frame_id: ''
////    joint_names: ['sdh_knuckle_joint', 'sdh_thumb_2_joint', 'sdh_thumb_3_joint', 'sdh_finger_12_joint', 'sdh_finger_13_joint', 'sdh_finger_22_joint', 'sdh_finger_23_joint']
////    points:
////      -
////        positions: [0.0, 0.0, 0.0, -1.4, 0.0, -1.4, 0.0]
////        velocities: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
////        accelerations: []
////        time_from_start:
////          secs: 3
////          nsecs: 0
////  path_tolerance: []
////  goal_tolerance: []
////  goal_time_tolerance:
////    secs: 0
////    nsecs: 0
//
////	control_msgs::FollowJointTrajectoryGoal sdh_goal;
////	sdh_goal.trajectory.joint_names.push_back("sdh_knuckle_joint");
////	sdh_goal.trajectory.joint_names.push_back("sdh_thumb_2_joint");
////	sdh_goal.trajectory.joint_names.push_back("sdh_thumb_3_joint");
////	sdh_goal.trajectory.joint_names.push_back("sdh_finger_12_joint");
////	sdh_goal.trajectory.joint_names.push_back("sdh_finger_13_joint");
////	sdh_goal.trajectory.joint_names.push_back("sdh_finger_22_joint");
////	sdh_goal.trajectory.joint_names.push_back("sdh_finger_23_joint");
////	trajectory_msgs::JointTrajectoryPoint point;
////	point.positions.push_back(0.0);
////	point.positions.push_back(0.0);
////	point.positions.push_back(0.0);
////	point.positions.push_back(-1.4);
////	point.positions.push_back(0.0);
////	point.positions.push_back(-1.4);
////	point.positions.push_back(0.0);
////	point.time_from_start.sec = 3;
////	sdh_goal.trajectory.points.push_back(point);
////	sdh_follow_joint_client_.sendGoal(sdh_goal);
////	bool finished_before_timeout = sdh_follow_joint_client_.waitForResult(ros::Duration(15.0));
////	if (finished_before_timeout)
////	{
////		actionlib::SimpleClientGoalState state = sdh_follow_joint_client_.getState();
////		ROS_INFO("TrashBinDetectionNode::graspTrashBin: Action finished: %s",state.toString().c_str());
////	}
////	else
////		ROS_INFO("TrashBinDetectionNode::graspTrashBin: Action did not finish before the time out.");
//
//	// move arm
//	// --------
//
//	// this connecs to a running instance of the move_group node
//	moveit::planning_interface::MoveGroup group("arm");
//	// specify that our target will be a random one
//	geometry_msgs::PoseStamped pose;
////	trash bin (pre-)grasp back-right
////	- Translation: [-0.518, -0.421, 0.566]
////	- Rotation: in Quaternion [0.014, 0.001, 0.014, 1.000]
////	            in RPY [0.027, 0.001, 0.027]
//
//	// trash bin (pre-)grasp right
////	- Translation: [-0.002, -0.580, 0.743]
////	- Rotation: in Quaternion [0.014, -0.006, 0.737, 0.676]
////	            in RPY [0.010, -0.028, 1.656]
//
//	pose.pose.position.x = 0.0;
//	pose.pose.position.y = -0.580;
//	pose.pose.position.z =  0.743;
//	tf::Quaternion q;
//	q.setRPY(0.0, 0.0, 1.57079632679);
//	tf::quaternionTFToMsg(q, pose.pose.orientation);
//
////	pose.pose.position.x = -0.466;
////	pose.pose.position.y = -0.538;
////	pose.pose.position.z =  1.538;
////	tf::Quaternion q;
////	q.setRPY(3.141, 0.051, 1.307);
////	tf::quaternionTFToMsg(q, pose.pose.orientation);
////	-0.466, -0.538, 1.538
////	0.999, -0.027, 0.012, 0.025
//
//	pose.header.frame_id = "base_link";
//	pose.header.stamp = ros::Time::now();
//	group.setPoseTarget(pose, "arm_7_link");
//
////	pose = group.getCurrentPose();
////	pose.pose.position.y += 0.05;
////	group.setGoalTolerance(0.01);
//
//	group.setPoseReferenceFrame("base_link");
//	std::cout << "group.getEndEffectorLink()=" << group.getEndEffectorLink() << "  group.getPoseReferenceFrame()=" << group.getPoseReferenceFrame() << std::endl;
//
//	group.move();
//	// plan the motion and then move the group to the sampled target
////	bool have_plan = false;
////	moveit::planning_interface::MoveGroup::Plan plan;
////	for (int trial=0; have_plan==false && trial<5; ++trial)
////		have_plan = group.plan(plan);
////	if (have_plan==true)
////		group.execute(plan);
////	else
////		ROS_WARN("No valid plan found for arm movement.");
//
//*/
//	// this sends the response back to the caller
//	autopnp_scenario::GraspTrashBinResult res;
//	grasp_trash_bin_server_.setSucceeded(res);
//}

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
	TrashBinDetectionNode trash_bin_detection(nh);
	trash_bin_detection.init(nh);
	ROS_INFO("Trash Bin detection Server initialized.....");

	ros::spin();

	return 0;
}

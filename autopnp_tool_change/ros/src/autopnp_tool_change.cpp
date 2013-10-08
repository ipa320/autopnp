#include <autopnp_tool_change/autopnp_tool_change.h>



ToolChange::ToolChange(ros::NodeHandle nh):
move_to_slot_server(nh, "autopnp_tool_change", boost::bind(&ToolChange::moveToSlot, this, _1), false)
{
	std::cout << "server constructor" << std::endl;

	node_handle_ = nh;
	input_marker_detection_sub_.unsubscribe();
	camera_matrix_received_ = false;

	// subscribers

	// ins action callback
	bool arm_finished = false;

	input_marker_detection_sub_.subscribe(node_handle_, "input_marker_detections", 1);
	input_marker_detection_sub_.registerCallback(boost::bind(&ToolChange::inputCallback, this, _1));



	//while (arm_finished == false)
	//	loop_rate.sleep();

	// ans ende ins action callback, wenn position erreicht
	//input_marker_detection_sub_.unsubscribe()
}



ToolChange::~ToolChange()
{
}

void ToolChange::init() {
	std::cout << "server init()" << std::endl;
	ToolChange::move_to_slot_server.start();
}

void ToolChange::moveToSlot(const autopnp_tool_change::MoveToSlotGoalConstPtr& goal) {
	// this callback function is executed each time a request (= goal message) comes in for this service server
	// here we just read the number from the goal message

	ROS_INFO("Action Server: Received a request : "
			"translation [%u, %u, %u], rotation [%u, %u, %u, %u]",

			(unsigned int) goal->transform.translation.x,
			(unsigned int) goal->transform.translation.y,
			(unsigned int) goal->transform.translation.z,

			(unsigned int) goal->transform.rotation.w,
			(unsigned int) goal->transform.rotation.x,
			(unsigned int) goal->transform.rotation.y,
			(unsigned int) goal->transform.rotation.z);

	// this command sends a feedback message to the caller
	autopnp_tool_change::MoveToSlotFeedback feedback;
	ToolChange::move_to_slot_server.publishFeedback(feedback);

	// this sends the response back to the caller
	autopnp_tool_change::MoveToSlotResult res;
	ToolChange::move_to_slot_server.setSucceeded(res);
}


/// callback for the incoming  data stream
void ToolChange::inputCallback(const cob_object_detection_msgs::DetectionArray::ConstPtr& input_marker_detections_msg)
{
	std::cout << "Recording data ..." << std::endl;

	if (input_marker_detections_msg->detections.size() == 0)
	{
		ROS_INFO("ToolChange::inputCallback: No markers detected.\n");
		return;
	}
	ROS_INFO("Message callback");
	// compute mean coordinate system if multiple markers detected
	tf::Transform fiducial_pose = computeMarkerPose(input_marker_detections_msg);
	tf::Transform pose_recorded = fiducial_pose.inverse();

	ROS_INFO("Averaged %u markers.", (unsigned int)input_marker_detections_msg->detections.size());


	printPose(fiducial_pose);

	for (unsigned int i = 0; i < input_marker_detections_msg->detections.size(); ++i)
	{
		std::string fiducial_label = input_marker_detections_msg->detections[i].label;
		unsigned int fiducial_label_num = boost::lexical_cast<int>(fiducial_label);
		//ROS_INFO("Detected label %s", fiducial_label.c_str());

		if(fiducial_label_num == 4) {
			ROS_INFO("Detected arm");
		}
	}

}
/**
 *
 *Calculates translation and orientation distance between arm marker and board
 */
void ToolChange::calculateDistanceToArm(
		const cob_object_detection_msgs::DetectionArray::ConstPtr& input_marker_detections_msg) {

	/**  tf::Vector3 board_translation;
		tf::Quaternion board_orientation(0.,0.,0.);
		tf::Vector3 arm_translation;
		tf::Quaternion arm_orientation(0.,0.,0.);*/
}

/// Converts a color image message to cv::Mat format.
bool ToolChange::convertColorImageMessageToMat(const sensor_msgs::Image::ConstPtr& image_msg, cv_bridge::CvImageConstPtr& image_ptr, cv::Mat& image)
{
	try
	{
		image_ptr = cv_bridge::toCvShare(image_msg, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("ObjectCategorization: cv_bridge exception: %s", e.what());
		return false;
	}
	image = image_ptr->image;

	return true;
}

tf::Transform ToolChange::computeMarkerPose(
		const cob_object_detection_msgs::DetectionArray::ConstPtr& input_marker_detections_msg)
{
	tf::Vector3 board_translation;
	tf::Quaternion board_orientation(0.,0.,0.);

	for (unsigned int i = 0; i < input_marker_detections_msg->detections.size(); ++i)
	{
		// todo: only average the 4 markers from the board
		std::string fiducial_label = input_marker_detections_msg->detections[i].label;
		unsigned int fiducial_label_num = boost::lexical_cast<int>(fiducial_label);

		if(fiducial_label_num != 4) {

			tf::Point translation;
			tf::pointMsgToTF(input_marker_detections_msg->detections[i].pose.pose.position, translation);
			tf::Quaternion orientation;
			tf::quaternionMsgToTF(input_marker_detections_msg->detections[i].pose.pose.orientation, orientation);

			if (i==0)
			{
				board_translation = translation;
				board_orientation = orientation;
			}
			else
			{
				board_translation += translation;
				board_orientation += orientation;
			}
		}
	}
	board_translation /= (double)input_marker_detections_msg->detections.size();
	board_orientation /= (double)input_marker_detections_msg->detections.size();
	board_orientation.normalize();

	return tf::Transform(board_orientation, board_translation);
}

unsigned long ToolChange::ProjectXYZ(double x, double y, double z, int& u, int& v)
{
	cv::Mat XYZ(4, 1, CV_64FC1);
	cv::Mat UVW(3, 1, CV_64FC1);

	x *= 1000;
	y *= 1000;
	z *= 1000;

	double* d_ptr = XYZ.ptr<double>(0);
	d_ptr[0] = x;
	d_ptr[1] = y;
	d_ptr[2] = z;
	d_ptr[3] = 1.;

	UVW = color_camera_matrix_ * XYZ;

	d_ptr = UVW.ptr<double>(0);
	double du = d_ptr[0];
	double dv = d_ptr[1];
	double dw = d_ptr[2];

	u = cvRound(du/dw);
	v = cvRound(dv/dw);

	return 1;
}

void ToolChange::calibrationCallback(const sensor_msgs::CameraInfo::ConstPtr& calibration_msg)
{
	if (camera_matrix_received_ == false)
	{
		//	pointcloud_height_ = calibration_msg->height;
		//	pointcloud_width_ = calibration_msg->width;
		cv::Mat temp(3,4,CV_64FC1);
		for (int i=0; i<12; i++)
			temp.at<double>(i/4,i%4) = calibration_msg->P.at(i);
		//		std::cout << "projection_matrix: [";
		//		for (int v=0; v<3; v++)
		//			for (int u=0; u<4; u++)
		//				std::cout << temp.at<double>(v,u) << " ";
		//		std::cout << "]" << std::endl;
		color_camera_matrix_ = temp;
		camera_matrix_received_ = true;
	}
}

void ToolChange::printPose(tf::Transform& trans_msg)
{

	ROS_INFO("translation [%f, %f, %f] , orientation [%f, %f, %f, %f]",
			(float) trans_msg.getOrigin().m_floats[0],
			(float) trans_msg.getOrigin().m_floats[1],
			(float) trans_msg.getOrigin().m_floats[2],

			(float) trans_msg.getRotation().getX(),
			(float) trans_msg.getRotation().getY(),
			(float) trans_msg.getRotation().getZ(),
			(float) trans_msg.getRotation().getW());

}

int main (int argc, char** argv)
{
	// Initialize ROS, specify name of node
	ros::init(argc, argv, "autopnp_tool_change_server");


	// Create a handle for this node, initialize node
	ros::NodeHandle nh;

	// Create and initialize an instance of Object
	ToolChange toolChange(nh);
	toolChange.init();

	ros::spin();

	return (0);
}

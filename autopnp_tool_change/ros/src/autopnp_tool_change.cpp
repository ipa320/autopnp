#include <autopnp_tool_change/autopnp_tool_change.h>




//ToolChange::ToolChange(ros::NodeHandle nh)
//: node_handle_(nh)
//{
//	camera_matrix_received_ = false;
//
//	// subscribers
//
//	// ins action callback
//	bool arm_finished = false;
//	input_marker_detection_sub_.subscribe(node_handle_, "input_marker_detections", 1);
//	input_marker_detection_sub_.registerCallback(boost::bind(&ToolChange::inputCallback, this, _1));
//
//	//server
//	move_to_slot_server(nh, "go_to_slot", boost::bind(&ToolChange::moveToSlot, this, _1), false);
//	// warten
//	// slow down the response to show the effect of feedback messages
//	ros::Rate loop_rate(0.5);
//	while (arm_finished==false)
//		loop_rate.sleep();
//
//	// ans ende ins action callback, wenn position erreicht
//	//input_marker_detection_sub_.unsubscribe()
//}



ToolChange::ToolChange(std::string name):
move_to_slot_server(node_handle_, name, boost::bind(&ToolChange::moveToSlot, this, _1), false),
action_name_(name)
{
	move_to_slot_server.start();
	input_marker_detection_sub_.unsubscribe();
	camera_matrix_received_ = false;

	// subscribers

	// ins action callback
	bool arm_finished = false;
	input_marker_detection_sub_.subscribe(node_handle_, "input_marker_detections", 1);
	input_marker_detection_sub_.registerCallback(boost::bind(&ToolChange::inputCallback, this, _1));

	// warten
	// slow down the response to show the effect of feedback messages
	ros::Rate loop_rate(0.5);
	while (arm_finished==false)
		loop_rate.sleep();

	// ans ende ins action callback, wenn position erreicht
	//input_marker_detection_sub_.unsubscribe()
}



ToolChange::~ToolChange()
{
}

//void init() {
//	ToolChange::move_to_slot_server.start();
//}
void ToolChange::moveToSlot(const autopnp_tool_change::MoveToSlotGoalConstPtr& goal) {
	// this callback function is executed each time a request (= goal message) comes in for this service server
	// here we just read the number from the goal message, square it and put the result into the result message

	ROS_INFO("Action Server: Received a request with number ");

	// this command sends a feedback message to the caller, here we transfer that the task is completed 25%
	autopnp_tool_change::MoveToSlotFeedback feedback;

	ToolChange::move_to_slot_server.publishFeedback(feedback);

	// slow down the response to show the effect of feedback messages
	ros::Rate loop_rate(0.5);
	loop_rate.sleep();

	autopnp_tool_change::MoveToSlotResult res;

	// this sends the response back to the caller
	ToolChange::move_to_slot_server.setSucceeded(res);
}




/// callback for the incoming  data stream
void ToolChange::inputCallback(const cob_object_detection_msgs::DetectionArray::ConstPtr& input_marker_detections_msg)
{
	//std::cout << "Recording data..." << std::endl;

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

	//todo: compute translation and orientation distance between arm marker and board markers
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
	tf::Vector3 mean_translation;
	tf::Quaternion mean_orientation(0.,0.,0.);
	for (unsigned int i=0; i<input_marker_detections_msg->detections.size(); ++i)
	{
		// todo: only average the 4 markers from the board

		tf::Point translation;
		tf::pointMsgToTF(input_marker_detections_msg->detections[i].pose.pose.position, translation);
		tf::Quaternion orientation;
		tf::quaternionMsgToTF(input_marker_detections_msg->detections[i].pose.pose.orientation, orientation);

		if (i==0)
		{
			mean_translation = translation;
			mean_orientation = orientation;
		}
		else
		{
			mean_translation += translation;
			mean_orientation += orientation;
		}
	}
	mean_translation /= (double)input_marker_detections_msg->detections.size();
	mean_orientation /= (double)input_marker_detections_msg->detections.size();
	mean_orientation.normalize();
	return tf::Transform(mean_orientation, mean_translation);
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


int main (int argc, char** argv)
{
	// Initialize ROS, specify name of node
	ros::init(argc, argv, "autopnp_tool_change");

	// Create a handle for this node, initialize node
	ros::NodeHandle nh;

	// Create and initialize an instance of Object
	ToolChange toolChange(ros::this_node::getName());


	ros::spin();

	return (0);
}

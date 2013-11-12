#include <autopnp_tool_change/autopnp_tool_change.h>
#include <math.h>
#include <tf/transform_broadcaster.h>
#include <moveit/move_group_interface/move_group.h>

using namespace std;

/*
 * Initializing the server before start and waiting for data :
 * Starts all needed subscriptions and publications.
 * Makes the server sleep while the data in question
 * have not been received jet. The spinOnce operation
 * allows the calculation of the background processes
 * and sleeping functions to run simultaneously .
 */
ToolChange::ToolChange(ros::NodeHandle nh): move_to_wagon_server
		(nh, "tool_change", boost::bind(&ToolChange::moveToWagon, this, _1), false)
{
	cout << "Starting server." << endl;

	node_handle = nh;
	input_marker_detection_sub.unsubscribe();
	joint_states_sub.unsubscribe();
	camera_matrix_received = false;
	slot_position_detected = false;

	// subscribers
	//joint_states_sub_.subscribe(node_handle_, "/joint_states",1);
	//joint_states_sub_.registerCallback(boost::bind(&ToolChange::jointInputCallback, this, _1));
	vis_pub = node_handle.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );


	input_marker_detection_sub.subscribe(node_handle, "input_marker_detections", 1);
	input_marker_detection_sub.registerCallback(boost::bind(&ToolChange::markerInputCallback, this, _1));

	//sleep
	while(slot_position_detected == false)
	{

		ros::spinOnce();
	}
	//subscription quit
	input_marker_detection_sub.unsubscribe();
	joint_states_sub.unsubscribe();
}

/*
 * TO DO: set the garbage container free !!!!!!
 */
ToolChange::~ToolChange()
{

}

/*
 * Starting server after initialization process.
 */
void ToolChange::init()
{
	ToolChange::move_to_wagon_server.start();
}

/**
 * Callback for the incoming  data stream.
 * Retrieves coming data from the joint_states message
 * and saves them as an instance of a
 * JointState msgs {@value jm}.
 */
void ToolChange::jointInputCallback(const sensor_msgs::JointState::ConstPtr& input_joint_msg)
{
	sensor_msgs::JointState jm;

	jointVelocities.clear();
	jointPositions.clear();

	jm.name = input_joint_msg->name;
	jm.header.stamp = input_joint_msg->header.stamp;
	jm.position = input_joint_msg->position;
	jm.velocity = input_joint_msg->velocity;
	jm.effort = input_joint_msg->effort;

	for (auto s: input_joint_msg->velocity)
	{
		jointVelocities.push_back(s);
	}
	for(auto s : input_joint_msg->position)
	{
		jointPositions.push_back(s);
	}

	//ToolChange::printVector(jointPositions);
	// ROS_INFO(" joint input callback %u", (unsigned int) input_joint_msg->position.size());
}


/**
 * Callback for the incoming  data stream.
 * Retrieves coming data from the input_marker_detections message
 * and calculates the distance and orientation between the position
 * of the arm and board setting the {@value arm_board_transform}.
 * Sets the variable {@value slot_position_detected} as true,
 * if the calculations took place. Quits the subscription to
 * the marker detection topic to save the last coordinates.
 */
void ToolChange::markerInputCallback(const cob_object_detection_msgs::DetectionArray::ConstPtr& input_marker_detections_msg)
{

	if (input_marker_detections_msg->detections.size() == 0)
	{
		ROS_INFO("ToolChange::inputCallback: No markers detected.\n");
		return;
	}
	//ROS_INFO("InputCallback for input_marker_detections_msg received.");
	struct ToolChange::components result_components;
	tf::Transform fiducial_pose_board;
	tf::Transform fiducial_pose_arm;


	// compute mean coordinate system if multiple markers detected
	result_components = computeMarkerPose(input_marker_detections_msg);
	fiducial_pose_board = result_components.board.translation;
	fiducial_pose_arm = result_components.arm.translation;


	//if the incoming data are not empty, calculate the distance and orientation
	bool empty_data = (fiducial_pose_board.getOrigin().length() == 0 ||
			fiducial_pose_arm.getOrigin().length() == 0);

	if(! empty_data)
	{
		arm_board_transform = calculateTransformationToArm(fiducial_pose_board, fiducial_pose_arm);

		//ToolChange::printPose(fiducial_pose_board);
		//ToolChange::printPose(fiducial_pose_arm);
		//ToolChange::printPose(arm_board_transform);

		/*
		 ROS_INFO("Board distance %f, arm distance %f, distance %f",
				(float) fiducial_pose_board.getOrigin().length(),
				(float) fiducial_pose_arm.getOrigin().length(),
				(float)arm_board_transform.getOrigin().length());
		 */

		if(arm_board_transform.getOrigin().length() != 0) {

			ROS_INFO("Slot position has been detected. ");
			input_marker_detection_sub.unsubscribe();
			ToolChange::slot_position_detected = true;

		}
	}
}

/*
 * Computes mean coordinate system if multiple markers detected
 * and saves the data as an array of two fiducial objects
 * {@value arm, @value board} with the transform data
 * {@value translation} respectively.
 */
struct ToolChange::components ToolChange::computeMarkerPose(
		const cob_object_detection_msgs::DetectionArray::ConstPtr& input_marker_detections_msg)
{
	ToolChange::components result;
	unsigned int count = 0;

	for (unsigned int i = 0; i < input_marker_detections_msg->detections.size(); ++i)
	{
		//retrieve the number of label and format the string message to an int number
		std::string fiducial_label = input_marker_detections_msg->detections[i].label;
		unsigned int fiducial_label_num = boost::lexical_cast<int>(fiducial_label);

		tf::Point translation;
		tf::Quaternion orientation;

		//convert translation and orientation Points msgs to Points respectively
		tf::pointMsgToTF(input_marker_detections_msg->detections[i].pose.pose.position, translation);
		tf::quaternionMsgToTF(input_marker_detections_msg->detections[i].pose.pose.orientation, orientation);

		// average only the 4 markers from the board (label values 0,1,2,3 set in common/files)
		if(fiducial_label_num < 4) {

			count++;
			if (i==0)
			{
				result.board.translation.setOrigin(translation);
				result.board.translation.setRotation(orientation);
			}
			else
			{
				result.board.translation.getOrigin() += translation;
				result.board.translation.getRotation() += orientation;
			}
		}
		// set the arm marker (label value 4 set in common/files)
		if(fiducial_label_num == 4) {

			result.arm.translation.setOrigin(translation);
			result.arm.translation.setRotation(orientation);
			result.arm.translation.getRotation().normalize();
		}
	}

	if(count != 0)
	{
		result.board.translation.getOrigin() /= (double)count;
		result.board.translation.getRotation() /= (double)count;
		result.board.translation.getRotation().normalize();
	}

	return result;
}

/*
 * Calculates translation and orientation distance between arm marker and wagon board.
 * Vector orientation points from arm to board. Arm quaternion must be inverted to justify its
 * parent position according to a child position of the board.
 *
 * Simulates the endeffector pose depending on the precise chosen position
 * of the fiducial, which has been fixed/clamped to the arm_7_link.
 * Here : in initial arm position (x-down, y-right, z-to observer)
 * corresponds to a rotation around y-axes to its left (positive angle).
 */
tf::Transform ToolChange::calculateTransformationToArm(
		const tf::Transform& board_pose, const tf::Transform& arm_pose)
{
	tf::Quaternion rotate_Y_45_left;
	rotate_Y_45_left.setRPY(0.0, M_PI/2, 0.0);

	tf::Transform arm_pose_simulated;

	//tf::Vector3 translation = (- arm_pose_simulated.getOrigin() + board_pose.getOrigin());
	//tf::Quaternion orientation =
	//	((arm_pose_simulated.getRotation() * rotate_Y_45_left).inverse()) * board_pose.getRotation();
	arm_pose_simulated.setRotation(arm_pose.getRotation() * rotate_Y_45_left);
	arm_pose_simulated.setOrigin(arm_pose.getOrigin());

	tf::Transform result;
	result = (arm_pose_simulated.inverse()) * board_pose;

	//return tf::Transform(orientation, translation);
	return result;
}

/*
 * Converts a color image message to cv::Mat format.
 */
bool ToolChange::convertColorImageMessageToMat(
		const sensor_msgs::Image::ConstPtr& image_msg, cv_bridge::CvImageConstPtr& image_ptr, cv::Mat& image)
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

/*
 * This callback function is executed each time a request (= goal message)
 * comes to this server (eventually this will be an initial
 * position message for the arm or a number of the tool
 * to be taken from the wagon). Starting move commands for the arm from here..
 *
 */
void ToolChange::moveToWagon(const autopnp_tool_change::MoveToWagonGoalConstPtr& goal) {

	/*
	ROS_INFO(" Received a goal request ");
	tf::Transform tfGoal;
	tf::Vector3 goal_translation = tf::Vector3(
			goal->transform.translation.x,
			goal->transform.translation.y,
			goal->transform.translation.z);
	tf::Quaternion goal_orientation = tf::Quaternion(
			goal->transform.rotation.w,
			goal->transform.rotation.x,
			goal->transform.rotation.y,
			goal->transform.rotation.z);
	 */
	//to do: set initial arm position as goal

	ToolChange::moveArm();

	// this command sends a feedback message to the caller
	autopnp_tool_change::MoveToWagonFeedback feedback;
	ToolChange::move_to_wagon_server.publishFeedback(feedback);

	// this sends the response back to the caller
	autopnp_tool_change::MoveToWagonResult res;
	ToolChange::move_to_wagon_server.setSucceeded(res);

}

/*
 * Executes the motion operations for the arm
 * using the moveIt interface utilities.
 * The reference frame, the "base_link" frame,
 * initializes the starting point of the coordinate system.
 * the goal frame describes the end effector ("arm_7_link")
 * which will be moved to a new position according to
 * its given position and orientation.
 */
void ToolChange::moveArm()
{
	ROS_INFO("Starting move motion.");

	moveit::planning_interface::MoveGroup group("arm");
	geometry_msgs::PoseStamped pose;
	pose.header.frame_id = "base_link";
	pose.header.stamp = ros::Time::now();
	group.setPoseReferenceFrame("base_link");

	//get the position of the end effector (= arm_7_joint)
	geometry_msgs::PoseStamped link_7_pose = group.getCurrentPose("arm_7_link");

	//transform end effector msg pose in tf
	tf::Transform link_pose_transform;
	tf::poseMsgToTF(link_7_pose.pose, link_pose_transform);

    //calculate a new end effector position in respect to the reference frame (= base_frame)
	tf::Transform new_link_7_pose = link_pose_transform * arm_board_transform;

    //transform tf to msg
	pose.pose.position.x = new_link_7_pose.getOrigin().getX();
	pose.pose.position.y = new_link_7_pose.getOrigin().getY();
	pose.pose.position.z = new_link_7_pose.getOrigin().getZ();
	tf::quaternionTFToMsg(new_link_7_pose.getRotation(), pose.pose.orientation);

	//draw transformation in rviz
	drawLine(0.85, 0.55, 0.0, 1.0, group.getCurrentPose("arm_7_link"), pose);
	/*
	 * gazebo sumu
	 *
	//pose.pose.position.x = 0.0;
	//pose.pose.position.y = -0.580;
	//pose.pose.position.z = 0.743;

	//geometry_msgs !!
	pose.pose.position.x = 0.0;
	pose.pose.position.y = -0.580;
	pose.pose.position.z = 0.943;
	//tf msgs !!
	q.setRPY(-0.17, -0.42, 0.0);
	tf::quaternionTFToMsg(q, pose.pose.orientation);
	 */

	/*
	 * schunk simu
	 */
	/*
	tf::Quaternion rotate_X_45_left;
	rotate_X_45_left.setRPY(0.0, M_PI, 0.0);

	pose.pose.position.x = 0.60;
	pose.pose.position.y = 0.0;
	pose.pose.position.z = 1.00;
	tf::Quaternion q;
	tf::Quaternion w;
	tf::Vector3 v = tf::Vector3(0.60, 0.0, 1.0);
	tf::Transform t;

	q.setRPY(M_PI/2, 0.0, -M_PI/2);
	t.setOrigin(v);
	t.setRotation(q);

	w.setRPY(M_PI/2, 0.0, 0.0);
	//tf::quaternionTFToMsg(q, pose.pose.orientation);
	//tf::quaternionTFToMsg(q, pose.pose.orientation);

	tf::quaternionTFToMsg(t.getRotation()*w, pose.pose.orientation);
	pose.pose.position.x = t.getOrigin().getX();
	pose.pose.position.y = t.getOrigin().getY();
	pose.pose.position.z = t.getOrigin().getZ();

	geometry_msgs::PoseStamped link_pose = group.getCurrentPose("arm_7_link");
	 */
	/*
    tf::Quaternion q;
	pose.pose.position.x = 0.05;
	pose.pose.position.y = 0.0;
	pose.pose.position.z = 1.40;
	q.setRPY(1.57,0.0,1.57);

	//tf to geometry_msgs
	tf::quaternionTFToMsg(q, pose.pose.orientation);

	 */

	group.setPoseTarget(pose, "arm_7_link");

	// plan the motion and then move the group to the sampled target
	bool have_plan = false;
	moveit::planning_interface::MoveGroup::Plan plan;

	//for (int i = 0; have_plan == false && i < 2; ++i)
	have_plan = group.plan(plan);
	if (have_plan==true)
		group.execute(plan);
	else
		ROS_WARN("No valid plan found for arm movement.");
	group.move();


	/*
	 * random movement
	 */
	/*group.setRandomTarget();
	printVector(group.getCurrentJointValues());
	printVector(group.getCurrentRPY("arm_7_link"));
	geometry_msgs::PoseStamped t = group.getCurrentPose("arm_7_link");
	group.move();
	 */
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

	UVW = color_camera_matrix * XYZ;

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
	if (camera_matrix_received == false)
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
		color_camera_matrix = temp;
		camera_matrix_received = true;
	}
}
/**
 * A helper function to print the results of
 * a transform message.
 */
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
/*
 * A helper function to print out
 * the PoseStamped msg.
 */
void ToolChange::printMsg(const geometry_msgs::PoseStamped pose)
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
/**
 * A helper function to print the
 * members of a vector.
 */
void ToolChange::printVector(const std::vector<double> v)
{

	std::cout<<" received vector with size : "<< v.size()<<std::endl;
	for( std::vector<double>::const_iterator i = v.begin(); i != v.end(); ++i)
	{
		std::cout << *i << ' ';
	}
	std::cout<<std::endl;

}
/*
 * A helper function to draw a simple arrow in rviz.
 */
void ToolChange::drawArrow (const double r, const double g, const double b, const double a,
		const geometry_msgs::PoseStamped& pose)
{
	visualization_msgs::Marker marker;
	marker.pose.position.x = pose.pose.position.x;
	marker.pose.position.y = pose.pose.position.y;
	marker.pose.position.z = pose.pose.position.z;

	marker.pose.orientation.x = pose.pose.orientation.x;
	marker.pose.orientation.y = pose.pose.orientation.y;
	marker.pose.orientation.z = pose.pose.orientation.z;
	marker.pose.orientation.w = pose.pose.orientation.w;

	marker.scale.x = 1;
	marker.scale.y = 0.025;
	marker.scale.z = 0.025;

	marker.color.r = r;
	marker.color.g = g;
	marker.color.b = b;
	marker.color.a = a;

	vis_pub.publish(marker);
}
/**
 * A helper function to draw a simple line in rviz.
 */
void ToolChange::drawLine (const double r, const double g, const double b, const double a,
		const geometry_msgs::PoseStamped& pose_start, const geometry_msgs::PoseStamped& pose_end)
{
	visualization_msgs::Marker marker;
	marker.header.frame_id = "base_link";
	marker.header.stamp = ros::Time::now();
	marker.action = visualization_msgs::Marker::ADD;
	marker.type = visualization_msgs::Marker::LINE_STRIP;
	geometry_msgs::Point p;
	p.x = pose_start.pose.position.x;
	p.y = pose_start.pose.position.y;
	p.z = pose_start.pose.position.z;
	marker.points.push_back(p);
	p.x =pose_end.pose.position.x;
	p.y = pose_end.pose.position.y;
	p.z = pose_end.pose.position.z;

	marker.points.push_back(p);

	marker.scale.x = 0.025;
	marker.scale.y = 0.0;
	marker.scale.z = 0.0;

	marker.color.r = r;
	marker.color.g = g;
	marker.color.b = b;
	marker.color.a = a;

	vis_pub.publish(marker);
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

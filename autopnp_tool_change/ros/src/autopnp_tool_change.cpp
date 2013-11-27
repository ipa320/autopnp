#include <autopnp_tool_change/autopnp_tool_change.h>
#include <math.h>
#include <tf/transform_broadcaster.h>
#include <moveit/move_group_interface/move_group.h>

/*
 * Initializing the server before start and waiting for data :
 * Starts all needed subscriptions and publications.
 * Makes the server sleep while the data in question
 * have not been received jet. The spinOnce operation
 * allows the calculation of the background processes
 * and sleeping functions to run simultaneously .
 */
ToolChange::ToolChange(ros::NodeHandle nh): move_to_wagon_server_
(nh, "tool_change", boost::bind(&ToolChange::moveToWagonSlot, this, _1), false)
{
	std::cout << "Starting server..." << std::endl;

	node_handle_ = nh;
	input_marker_detection_sub_.unsubscribe();
	joint_states_sub_.unsubscribe();
	slot_position_detected_ = false;
	reached_pregrasp_pose_ = false;
	marker_id_ = 0;

	// subscribers
	//joint_states_sub_.subscribe(node_handle_, "/joint_states",1);
	//joint_states_sub_.registerCallback(boost::bind(&ToolChange::jointInputCallback, this, _1));
	vis_pub_ = node_handle_.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
	input_marker_detection_sub_.subscribe(node_handle_, "input_marker_detections", 1);
	input_marker_detection_sub_.registerCallback(boost::bind(&ToolChange::markerInputCallback, this, _1));


	//sleep till the transformation found
	while(slot_position_detected_ == false)
	{
		ros::spinOnce();
	}

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
	ToolChange::move_to_wagon_server_.start();

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

	jointVelocities_.clear();
	jointPositions_.clear();

	jm.name = input_joint_msg->name;
	jm.header.stamp = input_joint_msg->header.stamp;
	jm.position = input_joint_msg->position;
	jm.velocity = input_joint_msg->velocity;
	jm.effort = input_joint_msg->effort;

	for (auto s: input_joint_msg->velocity)
	{
		jointVelocities_.push_back(s);
	}
	for(auto s : input_joint_msg->position)
	{
		jointPositions_.push_back(s);
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

	//ROS_INFO("InputCallback for input_marker_detections_msg received.");
	struct ToolChange::components result_components;
	tf::Transform fiducial_pose_board;
	tf::Transform fiducial_pose_arm;
	tf::Transform arm_board;

	if (input_marker_detections_msg->detections.size() != 0 )
	{
		//set marker components if such detected
		result_components = computeMarkerPose(input_marker_detections_msg);

		//use marker components only if both,
		//arm and board markers detected
		//else the components are empty
		if(detected_both_fiducials_ == true)
		{
			fiducial_pose_board = result_components.board.translation;
			fiducial_pose_arm = result_components.arm.translation;

			arm_board = calculateArmBoardTransformation(fiducial_pose_board, fiducial_pose_arm);

			if(!arm_board.getOrigin().isZero())
			{
				//ROS_INFO("Slot position has been detected. ");
				arm_board_transform_ = arm_board;
				slot_position_detected_ = true;
			}
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
	detected_both_fiducials_ = false;
	bool detected_arm_fiducial = false;
	bool detected_board_fiducial = false;
	tf::Point translation;
	tf::Quaternion orientation;

	for (unsigned int i = 0; i < input_marker_detections_msg->detections.size(); ++i)
	{

		//retrieve the number of label and format the string message to an int number
		std::string fiducial_label = input_marker_detections_msg->detections[i].label;
		unsigned int fiducial_label_num = boost::lexical_cast<int>(fiducial_label);
		//ROS_INFO("number %u , ", (unsigned int) fiducial_label_num) ;

		//convert translation and orientation Points msgs to Points respectively
		tf::pointMsgToTF(input_marker_detections_msg->detections[i].pose.pose.position, translation);
		tf::quaternionMsgToTF(input_marker_detections_msg->detections[i].pose.pose.orientation, orientation);

		// average only the 4 markers from the board (label values 0,1,2,3 set in common/files)
		if(fiducial_label_num < 4)
		{
			detected_board_fiducial = true;
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
		if(fiducial_label_num == 4)
		{
			detected_arm_fiducial = true;
			result.arm.translation.setOrigin(translation);
			result.arm.translation.setRotation(orientation);
			result.arm.translation.getRotation().normalize();
		}
	}

	if(count != 0)
	{
		result.board.translation.getOrigin() /=(double)count;
		result.board.translation.getRotation() /= (double)count;
		result.board.translation.getRotation().normalize();
	}

	detected_both_fiducials_ = detected_arm_fiducial && detected_board_fiducial;

	return result;
}

/*
 * Calculates translation and orientation distance between arm marker and wagon board.
 * Vector orientation points from arm to board. Arm quaternion must be inverted to justify its
 * parent position according to a child position of the board.
 *
 * Both fiducial positions are not real. Thats why they must undergo
 * simulated transformations. After those it will be possible
 * to calculate the real transformation between arm and board positions.
 *
 *
 * Simulates the endeffector pose depending on the precise chosen position
 * of the fiducial, which has been fixed/clamped to the arm_7_link.
 * Here :
 * - arm fiducial position (x-up, y-left, z-to observer) is shifted to its
 * corresponding arm position : rotation around x-axes in 45 (left) and after it
 * around y-axes in 45 degrees(left).
 * - board fiducial position (x-down, y-right, z-to observer) is shifted
 * to its original position : rotation around y-axes in 45 degrees(left).
 *
 */
tf::Transform ToolChange::calculateArmBoardTransformation(
		const tf::Transform& board_pose, const tf::Transform& arm_pose)
{
	//define tf poses
	tf::Transform board_pose_simulated;
	tf::Transform arm_pose_simulated;
	tf::Transform result;

	//define rotations
	tf::Quaternion rotate_Y_45_right;
	tf::Quaternion rotate_Y_45_left;
	tf::Quaternion rotate_X_pi_4_left;

	//set rotations
	rotate_X_pi_4_left.setRPY( M_PI/4, 0.0, 0.0);
	rotate_Y_45_left.setRPY(0.0, -M_PI/2, 0.0);
	rotate_Y_45_left.setRPY(0.0, -M_PI/2, 0.0);

	//set tf poses according to there real
	//orientation and position to the arm and board
	board_pose_simulated.setRotation(board_pose.getRotation() * rotate_Y_45_left);
	board_pose_simulated.setOrigin(board_pose.getOrigin());

	arm_pose_simulated.setRotation(((arm_pose.getRotation()) * rotate_X_pi_4_left) * rotate_Y_45_left);
	arm_pose_simulated.setOrigin(arm_pose.getOrigin());

	//calculate the transformation between arm and board
	result.mult(arm_pose_simulated.inverse() ,(board_pose_simulated));

	/// JUST DRAWING IN RVIZ FOR TEST PURPOSES
	tf::Transform a_tf;
	tf::Transform b_tf;
	geometry_msgs::PoseStamped a = origin;
	geometry_msgs::PoseStamped pose;
	geometry_msgs::PoseStamped base;
	geometry_msgs::PoseStamped result_stamped;

	tf::poseMsgToTF(a.pose, a_tf);
	b_tf = a_tf * result;

	tf::poseTFToMsg(b_tf, pose.pose);
	tf::poseTFToMsg(result, result_stamped.pose);

	drawLine(0.85, 0.55, 0.0, 1.0, a, pose);
	drawLine(0.85, 0.55, 0.0, 1.0, base, result_stamped);
	///END DRAWING

	return result;

}

/*
 * This callback function is executed each time a request (= goal message)
 * comes to this server (eventually this will be an initial
 * position message for the arm or a number of the tool
 * to be taken from the wagon). Starting move commands for the arm from here..
 *
 */
void ToolChange::moveToWagonSlot(const autopnp_tool_change::MoveToWagonGoalConstPtr& goal) {

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
	//TO DO: set initial arm position as goal

	ToolChange::moveArm();

	// this command sends a feedback message to the caller
	autopnp_tool_change::MoveToWagonFeedback feedback;
	ToolChange::move_to_wagon_server_.publishFeedback(feedback);

	// this sends the response back to the caller
	autopnp_tool_change::MoveToWagonResult res;
	ToolChange::move_to_wagon_server_.setSucceeded(res);

}
/*
 * Executes the motion operations for the arm
 * using the moveIt interface utilities.
 * The reference frame, the "base_link" frame,
 * initializes the starting point of the coordinate system.
 * the goal frame describes the end effector ("arm_7_link")
 * which will be moved to a new position according to
 * its given pre-grasp (with hand down) position and orientation.
 */
void ToolChange::moveToInitialPose()
{
	ROS_INFO("Moving to initial position.");

	//start from base
	geometry_msgs::PoseStamped pose;
	//endeffector msg pose
	geometry_msgs::PoseStamped link_pose;
	tf::Quaternion q;

	moveit::planning_interface::MoveGroup group("arm");

	pose.header.frame_id = "base_link";
	pose.header.stamp = ros::Time::now();
	group.setPoseReferenceFrame("base_link");

	link_pose = group.getCurrentPose("arm_7_link");
	origin = link_pose;

	q.setRPY(0.0, 0.0, M_PI/4);
	//q.setRPY(0.0, 0.0, 0.0);
	//q.setRPY(0.0,M_PI, 0.0);

	pose.pose.position.x = link_pose.pose.position.x - 0.4;
	pose.pose.position.y = link_pose.pose.position.y;
	pose.pose.position.z = link_pose.pose.position.z - 0.6;

	tf::quaternionTFToMsg(q, pose.pose.orientation);

	drawLine(0.55, 0.55, 0.0, 1.0, group.getCurrentPose("arm_7_link"), pose);
	group.setPoseTarget(pose, "arm_7_link");

	// plan the motion and then move the group to the target
	bool have_plan = false;
	moveit::planning_interface::MoveGroup::Plan plan;

	have_plan = group.plan(plan);

	//EXECUTE !!!!!! BE CAREFUL
	if (have_plan==true) {
		group.execute(plan);
		group.move();
		reached_pregrasp_pose_ = true;
	}
	else
	{
		ROS_WARN("No valid plan found for arm movement.");
		reached_pregrasp_pose_ = false;
	}


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
	//endeffector tf pose
	tf::Transform link_7_pose_tf;
	//arm-to-board transformation
	tf::Transform arm_board_tf;
	//base start pose
	geometry_msgs::PoseStamped pose;
	//endeffector msg pose
	geometry_msgs::PoseStamped link_7_pose;

	moveToInitialPose();

	//wait till pre_grasp position has been reached
	if(reached_pregrasp_pose_ != false)
	{
		//set the planning interface group
		moveit::planning_interface::MoveGroup group("arm");
		pose.header.frame_id = "base_link";
		pose.header.stamp = ros::Time::now();
		group.setPoseReferenceFrame("base_link");

		if(! arm_board_transform_.getOrigin().isZero())
		{
			//get the position of the end effector (= arm_7_joint)
			link_7_pose = group.getCurrentPose("arm_7_link");
			origin = link_7_pose;
            //translate msg to tf
			tf::poseMsgToTF(link_7_pose.pose, link_7_pose_tf);
            //calculate transformation tf pose from arm to board
			arm_board_tf = link_7_pose_tf * arm_board_transform_;
			//transform tf pose to msg
			tf::poseTFToMsg(arm_board_tf, pose.pose);

			drawLine(0.85, 0.55, 0.0, 1.0, group.getCurrentPose("arm_7_link"), pose);
			/*
			 * gazebo sumu
			 *
	tf::Quaternion q;

	//geometry_msgs !!
	pose.pose.position.x = 0.0;
	pose.pose.position.y = -0.580;
	pose.pose.position.z = 0.943;
	//tf msgs !!
	q.setRPY(-0.17, -0.42, 0.0);
			 */

			group.setPoseTarget(pose, "arm_7_link");

			// plan the motion and then move the group to the target
			bool have_plan = false;
			moveit::planning_interface::MoveGroup::Plan plan;

			//for (int i = 0; have_plan == false && i < 2; ++i)
			have_plan = group.plan(plan);

			//BE CAREFUL, IT IS GOING TO EXECUTE !!!!!!
			if (have_plan==true) {
				group.execute(plan);
				group.move();
			}
			else
			{
				ROS_WARN("No valid plan found for arm movement.");
			}
		}
		else
		{
			ROS_WARN("Invalid arm-to-board transformation.");
		}
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
	marker.header.frame_id = "base_link";
	marker.header.stamp = ros::Time::now();
	marker.id = marker_id_;
	marker.action = visualization_msgs::Marker::ADD;
	marker.type = visualization_msgs::Marker::ARROW;

	marker.pose.position.x = pose.pose.position.x;
	marker.pose.position.y = pose.pose.position.y;
	marker.pose.position.z = pose.pose.position.z;

	marker.pose.orientation.x = pose.pose.orientation.x;
	marker.pose.orientation.y = pose.pose.orientation.y;
	marker.pose.orientation.z = pose.pose.orientation.z;
	marker.pose.orientation.w = pose.pose.orientation.w;

	marker.scale.x = 0.25;
	marker.scale.y = 0.025;
	marker.scale.z = 0.025;

	marker.color.r = r;
	marker.color.g = g;
	marker.color.b = b;
	marker.color.a = a;

	vis_pub_.publish(marker);

	marker_id_++;
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
	marker.id = marker_id_;
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

	marker.scale.x = 0.005;
	marker.scale.y = 0.0;
	marker.scale.z = 0.0;

	marker.color.r = r;
	marker.color.g = g;
	marker.color.b = b;
	marker.color.a = a;

	vis_pub_.publish(marker);
	marker_id_++;
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

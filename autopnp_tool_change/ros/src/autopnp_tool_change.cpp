#include <autopnp_tool_change/autopnp_tool_change.h>
#include <math.h>
#include <tf/transform_broadcaster.h>
#include <moveit/move_group_interface/move_group.h>

/** Transformation Tree
____________________________________________________________
______FA_____--------------->________FB_____________________
______________/\____________/\______________________________
______|________\____________/________|______________________
______|_________\__________/_________|______________________
______|______________CAM_____________|______________________
______|_________/__________\_________|______________________
_____\|/_______/____/|\_____\_______\|/_____________________
______________\/_____|_______\/______________________________
______EE_____------------ --->_______GOAL___________________
_____________________|_______________________________________
__________/\_________|______________________________________
___________\_________|_______________________________________
____________\________|_______________________________________
_____________\_______________________________________________
____________________BASE_____________________________________
_____________________________________________________________
 **/

/*
 * Initializing the server before start and waiting for data :
 * Starts all needed subscriptions and publications.
 * Makes the server sleep while the data in question
 * have not been received jet. The spinOnce operation
 * allows the calculation of the background processes
 * and sleeping functions to run simultaneously .
 */
ToolChange::ToolChange(ros::NodeHandle nh): change_tool_server_
(nh, "tool_change", boost::bind(&ToolChange::changeTool, this, _1), false)
{
	std::cout << "Starting server..." << std::endl;

	node_handle_ = nh;
	input_marker_detection_sub_.unsubscribe();
	joint_states_sub_.unsubscribe();
	slot_position_detected_ = false;
	move_action_ = false;
	marker_id_ = 0;

	//set rotations
	rotate_3Z_pi_4_left.setRPY( 0.0, 0.0, 3*M_PI/4);
	rotate_3Z_pi_4_right.setRPY( 0.0, 0.0, -3*M_PI/4);
	rotate_Y_90_right.setRPY(0.0, -M_PI/2, 0.0);
	rotate_Y_90_left.setRPY(0.0, M_PI/2, 0.0);
	rotate_X_90_right.setRPY(-M_PI/2, 0.0,  0.0);
	rotate_X_90_left.setRPY(M_PI/2,0.0,  0.0);
	rotate_Z_90_left.setRPY(0.0, 0.0, M_PI/2);
	rotate_Z_90_right.setRPY(0.0, 0.0, -M_PI/2);

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
	ToolChange::change_tool_server_.start();
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

	struct ToolChange::components result_components;
	tf::Transform arm_board;
	tf::Transform cam_base;

	if (input_marker_detections_msg->detections.size() != 0 )
	{
		//set marker components if such detected
		result_components = computeMarkerPose(input_marker_detections_msg);

		//use marker components only if both,
		//arm and board markers detected
		//else the components are empty
		if(detected_all_fiducials_ == true)
		{
			arm_board = calculateArmBoardTransformation(result_components.board.translation, result_components.arm.translation);

			if(!arm_board.getOrigin().isZero())
			{
				transform_CA_BA_ = result_components.cam.translation;
				transform_CA_FA_ = result_components.arm.translation;
				transform_CA_FB_ = result_components.board.translation;

				/*
				 * ROS_INFO("*************");
				ROS_INFO("CA_BA :");
				printPose(transform_CA_BA_);
				ROS_INFO("CA_FA :");
				printPose(transform_CA_FA_);
				ROS_INFO("CA_FB :");
				printPose(transform_CA_FB_);
				 */
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
	detected_all_fiducials_ = false;
	bool detected_arm_fiducial = false;
	bool detected_board_fiducial = false;
	bool detected_cam_fiducial = false;
	tf::Point translation;
	tf::Quaternion orientation;

	for (unsigned int i = 0; i < input_marker_detections_msg->detections.size(); ++i)
	{

		//retrieve the number of label and format the string message to an int number
		std::string fiducial_label = input_marker_detections_msg->detections[i].label;
		//unsigned int fiducial_label_num = boost::lexical_cast<int>(fiducial_label);
		//ROS_INFO("number %u , ", (unsigned int) fiducial_label_num) ;

		//convert translation and orientation Points msgs to Points respectively
		tf::pointMsgToTF(input_marker_detections_msg->detections[i].pose.pose.position, translation);
		tf::quaternionMsgToTF(input_marker_detections_msg->detections[i].pose.pose.orientation, orientation);

		// average only the 3 markers from the board
		if (fiducial_label.compare(VAC_CLEANER)==0 || fiducial_label.compare(ARM_STATION)==0 || fiducial_label.compare(EXTRA_FIDUCIAL)==0)
			//if (fiducial_label.compare("tag_0")==0 || fiducial_label.compare("tag_1")==0 || fiducial_label.compare("tag_2")==0 || fiducial_label.compare("tag_3")==0)
		{
			detected_board_fiducial = true;
			count++;
			if (count==1)
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

		//if(fiducial_label_num == arm_marker_number)
		if (fiducial_label.compare(ARM)==0)
		{
			detected_arm_fiducial = true;
			result.arm.translation.setOrigin(translation);
			result.arm.translation.setRotation(orientation);
			result.arm.translation.getRotation().normalize();
		}
		if (fiducial_label.compare("tag_3")==0)
		{
			detected_cam_fiducial = true;
			result.cam.translation.setOrigin(translation);
			result.cam.translation.setRotation(orientation);
			result.cam.translation.getRotation().normalize();
		}
	}

	if(count != 0)
	{
		result.board.translation.getOrigin() /=(double)count;
		result.board.translation.getRotation() /= (double)count;
		result.board.translation.getRotation().normalize();
	}

	detected_all_fiducials_ = detected_arm_fiducial && detected_board_fiducial && detected_cam_fiducial;

	return result;
}

/*
 * Calculates translation and orientation distance between arm marker and wagon board.
 * Vector orientation points from arm to board.
 *
 *
 * Here :
 * - arm fiducial pregrasp position (x-down, y-right, z-to observer)
 * - board fiducial position (x-down, y-right, z-to observer)
 *
 */
tf::Transform ToolChange::calculateArmBoardTransformation(
		const tf::Transform& board_pose, const tf::Transform& arm_pose)
{
	tf::Transform result;
	tf::Transform transform_CA_FA, transform_CA_FB;

	transform_CA_FA.setOrigin(arm_pose.getOrigin() );
	transform_CA_FB.setOrigin(board_pose.getOrigin() );
	transform_CA_FA.setRotation( arm_pose.getRotation() );
	transform_CA_FB.setRotation( board_pose.getRotation());

	//calculate the transformation between arm and board
	result.mult(transform_CA_FA.inverse() ,transform_CA_FB);

	/// JUST DRAWING IN RVIZ FOR TEST PURPOSES
	tf::Transform a_tf;
	tf::Transform b_tf;
	geometry_msgs::PoseStamped a = current_ee_pose_;
	geometry_msgs::PoseStamped pose;
	geometry_msgs::PoseStamped base;
	geometry_msgs::PoseStamped result_stamped;
	//geometry_msgs::PoseStamped result_stamped_0;

	tf::poseMsgToTF(a.pose, a_tf);
	b_tf = a_tf * result;

	tf::poseTFToMsg(b_tf, pose.pose);
	tf::poseTFToMsg(result, result_stamped.pose);
	//drawLine(0.85, 0.55, 0.30, 1.0, base, result_stamped);
	//drawLine(0.85, 0.55, 0.0, 1.0,current_ee_pose_, pose);
	///END DRAWING

	return result;

}

/*
 * This callback function is executed each time a request (= goal message)
 * comes to this server (eventually this will be an initial
 * position message for the arm or a number of the tool
 * to be taken from the wagon). Starting move commands for the arm from here.
 *
 */
void ToolChange::changeTool(const autopnp_tool_change::MoveToWagonGoalConstPtr& goal) {

	geometry_msgs::PoseStamped fake_goal;
	tf::Quaternion q;

	q.setValue(-0.153, 0.9875, 0.017, -0.026);
	//q.setRPY(0.0, 0.0, M_PI);
	tf::quaternionTFToMsg(q, fake_goal.pose.orientation);

	fake_goal.pose.position.x =  0.06714;
	fake_goal.pose.position.y =  0.3877;
	fake_goal.pose.position.z =  0.9613;

	// this command sends a feedback message to the caller
	autopnp_tool_change::MoveToWagonFeedback feedback;
	ToolChange::change_tool_server_.publishFeedback(feedback);

	// this sends the response back to the caller
	autopnp_tool_change::MoveToWagonResult res;

	//res = processGoal(fake_goal);
	processGoal(fake_goal);
	change_tool_server_.setSucceeded(res);

}

/*
 * Goal processing. Returns true if the process succeeded, false if not.
 */
//bool ToolChange::processGoal(const autopnp_tool_change::MoveToWagonGoalConstPtr& goal);
bool ToolChange::processGoal(const geometry_msgs::PoseStamped& start_pose)
{
	int fake_command = 1;
	return coupleOrDecouple(fake_command, start_pose);
}

/*
 * Executes series of tasks to process coupling and decoupling.
 * Works with the initial start position of the arm {@value start_pose}
 * and the command {@value command} to execute. Here: couple or decouple.
 */
bool ToolChange::coupleOrDecouple(const int command, const geometry_msgs::PoseStamped& pose)
{
	//--------------------------------------------------------------------------------------
	// move arm to a start position
	//--------------------------------------------------------------------------------------
	tf::Vector3 direction;
	double offset = 0.0;

	/*	ROS_INFO("Moving arm to a pre-grasp position.");
	if(!executeMoveCommand(pose, offset))
	{
		ROS_WARN("Error occurred executing move to start position.");
		return false;
	}
	waitForMoveit();
	 */  //--------------------------------------------------------------------------------------
	// move arm to the wagon (pre-start position) using fiducials
	//--------------------------------------------------------------------------------------
	ROS_INFO("Moving arm to wagon fiducial position.");
	if(!turnToWagonFiducial(offset))
	{
		ROS_WARN("Error occurred executing turn to wagon fiducial position.");
		return false;
	}
	//TO DO: compute current fiducial position.
	if(!moveToWagonFiducial(offset))
	{
		ROS_WARN("Error occurred executing move to wagon fiducial position.");
		return false;
	}
	/*ROS_INFO("Moving arm to wagon fiducial position.");
	if(!turnToWagonFiducial(offset))
	{
		ROS_WARN("Error occurred executing turn to wagon fiducial position.");
		return false;
	}
	//TO DO: compute current fiducial position.
	if(!moveToWagonFiducial(offset))
	{
		ROS_WARN("Error occurred executing move to wagon fiducial position.");
		return false;
	}
	ROS_INFO("Moving arm to wagon fiducial position.");
	if(!turnToWagonFiducial(offset))
	{
		ROS_WARN("Error occurred executing turn to wagon fiducial position.");
		return false;
	}
	//TO DO: compute current fiducial position.
	if(!moveToWagonFiducial(offset))
	{
		ROS_WARN("Error occurred executing move to wagon fiducial position.");
		return false;
	}
	//   COUPLE / DECOUPLE

	//--------------------------------------------------------------------------------------
		// move arm straight up
		//--------------------------------------------------------------------------------------

	if(!executeStraightMoveCommand(UP, MAX_STEP_MIL))
		{
			ROS_WARN("Error occurred executing move arm straight up.");
			return false;
		}
	//--------------------------------------------------------------------------------------
	// move arm straight forward
	//--------------------------------------------------------------------------------------
	direction = FORWARD;

	if(!executeStraightMoveCommand(FORWARD, MAX_STEP_CM))
	{
		ROS_WARN("Error occurred executing move to wagon fiducial.");
		return false;
	}
	/*
	//--------------------------------------------------------------------------------------
	// move arm down to coupler
	//--------------------------------------------------------------------------------------

	if(!executeStraightMoveCommand(direction, MAX_STEP_CM))
	{
		ROS_WARN("Error occurred executing move arm down to coupler .");
		return false;
	}
	 */
	/*	//--------------------------------------------------------------------------------------
	// couple / decouple (different services !!!)
	//--------------------------------------------------------------------------------------
	if(command == COUPLE)
	{
		ROS_INFO("Start to couple.");
	}
	if(command == DECOUPLE)
	{
		ROS_INFO("Start to decouple.");
	}
	//--------------------------------------------------------------------------------------
	// move arm straight up
	//--------------------------------------------------------------------------------------


	if(!executeStraightMoveCommand(UP, MAX_STEP_MIL))
	{
		ROS_WARN("Error occurred executing move arm straight up.");
		return false;
	}
	//waitForMoveit();
	//--------------------------------------------------------------------------------------
	// move arm straight back
	//--------------------------------------------------------------------------------------
	direction = BACK;

	if(!executeStraightMoveCommand(direction, MAX_STEP_CM))
	{
		ROS_WARN("Error occurred executing move arm straight back.");
		return false;
	}
	 */
	return true;
}


bool ToolChange::moveToWagonFiducial(const double offset)
{
	move_action_ = false;
	geometry_msgs::PoseStamped ee_pose;
	geometry_msgs::PoseStamped goal_pose;
	tf::Transform ee_pose_tf;
	tf::Transform goal_pose_tf;
	tf::Quaternion q;
	q.setRPY(0.0, 0.0, -M_PI);
	tf::Transform transform_CA_FA;
	transform_CA_FA.setOrigin(transform_CA_FA_.getOrigin());
	transform_CA_FA.setRotation(transform_CA_FA_.getRotation());
	tf::Transform transform_CA_FB;
	transform_CA_FB.setOrigin(transform_CA_FB_.getOrigin());
	transform_CA_FB.setRotation(transform_CA_FB_.getRotation());
	tf::Transform transform_CA_EE;
	tf::Transform transform_EE_FA;
	tf::Transform transform_BA_FA;
	tf::Transform transform_BA_FB;
	tf::Transform transform_FA_FB;
	tf::Transform transform_EE_FB;
	tf::Transform transform_CA_BA;
	tf::Transform transform_EE_GO;

	transform_CA_BA.setOrigin(transform_CA_BA_.getOrigin());
	transform_CA_BA.setRotation(transform_CA_BA_.getRotation() * q);

	/*ROS_INFO("*************");
	ROS_INFO("CA_BA :");
	printPose(transform_CA_BA);
	ROS_INFO("CA_BA inverse:");
	tf::Transform t;
	t = transform_CA_BA.inverse();
	printPose(t);
	ROS_INFO("CA_FA :");
	printPose(transform_CA_FA);
	ROS_INFO("CA_FB :");
	printPose(transform_CA_FB);
	 */

	moveit::planning_interface::MoveGroup group(PLANNING_GROUP_NAME);
	goal_pose.header.frame_id = BASE_LINK;
	goal_pose.header.stamp = ros::Time::now();
	group.setPoseReferenceFrame(BASE_LINK);

	//get the position of the end effector (= arm_7_joint)
	ee_pose.pose = group.getCurrentPose(EE_NAME).pose;
	current_ee_pose_.pose = group.getCurrentPose(EE_NAME).pose;
	//msg -> tf
	tf::poseMsgToTF(ee_pose.pose, ee_pose_tf);
	geometry_msgs::PoseStamped cam_msg;
	geometry_msgs::PoseStamped test_msg;
	geometry_msgs::PoseStamped test1_msg;
	geometry_msgs::PoseStamped test2_msg;
	geometry_msgs::PoseStamped test3_msg;
	geometry_msgs::PoseStamped test4_msg;
	geometry_msgs::PoseStamped test5_msg;
	geometry_msgs::PoseStamped test6_msg;
	geometry_msgs::PoseStamped base;

	//cam-base
	tf::poseTFToMsg(transform_CA_BA.inverse(), cam_msg.pose);
	drawLine(0.55, 0.0, 0.0, 1.0, base, cam_msg);

	//base-cam-fiducial A
	transform_BA_FA.mult(transform_CA_BA.inverse(), transform_CA_FA);
	tf::poseTFToMsg(transform_BA_FA, test_msg.pose);
	drawLine(0.9, 0.0, 0.0, 1.0, cam_msg, test_msg);

	//base-cam-fiducial B
	transform_BA_FB.mult(transform_CA_BA.inverse(), transform_CA_FB);
	tf::poseTFToMsg(transform_BA_FB, test4_msg.pose);
	drawLine(0.55, 0.0, 0.0, 1.0, cam_msg, test4_msg);

	//drawLine(0.5, 0.5, 0.0, 1.0, base, ee_pose);
	//drawLine(0.0, 0.9, 0.0, 1.0, base, test_msg);
	//drawLine(0.0, 0.9, 0.0, 1.0, base, test4_msg);

	transform_FA_FB.mult(transform_BA_FA.inverse(), transform_BA_FB);
	drawLine(0.55, 0.0, 0.0, 1.0, test_msg, test4_msg);

	transform_EE_FA.mult(ee_pose_tf.inverse(), transform_BA_FA);
	tf::poseTFToMsg(transform_EE_FA, test5_msg.pose);
	//drawLine(0.0, 0.0, 0.3, 1.0, ee_pose, test5_msg);

	transform_EE_FB.mult( ee_pose_tf * transform_EE_FA, transform_FA_FB );
	tf::poseTFToMsg( transform_EE_FB, test6_msg.pose);
	//drawLine(0.0, 0.0, 0.3, 1.0, ee_pose, test6_msg);
	//drawLine(0.0, 0.0, 0.3, 1.0, test6_msg, test5_msg);

	transform_EE_GO.mult(ee_pose_tf.inverse(), transform_EE_FB);
	transform_EE_GO.setOrigin(transform_EE_GO.getOrigin() + TOOL_FIDUCIAL_OFFSET_0);
	//ROS_INFO("EE_GO :");
	//printPose(transform_EE_GO);
	goal_pose_tf.mult( ee_pose_tf , transform_EE_GO);

	goal_pose_tf.setRotation(goal_pose_tf.getRotation() * rotate_Y_90_right);
	goal_pose_tf.setRotation(goal_pose_tf.getRotation() * rotate_X_90_left);
	goal_pose_tf.setOrigin(goal_pose_tf.getOrigin());

	//ROS_INFO("goal :");
	//printPose(goal_pose_tf);

	//tf -> msg
	tf::poseTFToMsg(goal_pose_tf, goal_pose.pose);
	drawLine(0.55, 0.55, 0.0, 1.0, ee_pose, goal_pose);
	//drawSystem(goal_pose);
	drawArrowX(0.55, 0.0, 0.0, 1.0, goal_pose);

	group.setPoseTarget(goal_pose, EE_NAME);
	double distance = 0.0;
	//ROS_WARN_STREAM("distance " << distance << "!");
	if(distance < 2.0)
	{
		ROS_WARN_STREAM("STARTE MOVE TO FIDUCIAL");
		// plan the motion
		bool have_plan = false;
		moveit::planning_interface::MoveGroup::Plan plan;
		have_plan = group.plan(plan);

		//EXECUTE THE PLAN !!!!!! BE CAREFUL
		if (have_plan==true) {
			group.execute(plan);
			group.move();
		}
		else
		{
			ROS_WARN("No valid plan found for the arm movement.");
			move_action_ = false;
			return false;
		}
	}

	current_ee_pose_.pose = group.getCurrentPose(EE_NAME).pose;
	move_action_ = true;

	return true;
}
bool ToolChange::turnToWagonFiducial(const double offset)
{
	move_action_ = false;
	geometry_msgs::PoseStamped ee_pose;
	geometry_msgs::PoseStamped goal_pose;
	tf::Transform ee_pose_tf;
	tf::Transform goal_pose_tf;
	tf::Quaternion q;
	q.setRPY(0.0, 0.0, -M_PI);
	tf::Transform transform_CA_FA;
	transform_CA_FA.setOrigin(transform_CA_FA_.getOrigin());
	transform_CA_FA.setRotation(transform_CA_FA_.getRotation());
	tf::Transform transform_CA_FB;
	transform_CA_FB.setOrigin(transform_CA_FB_.getOrigin());
	transform_CA_FB.setRotation(transform_CA_FB_.getRotation());
	tf::Transform transform_CA_EE;
	tf::Transform transform_EE_FA;
	tf::Transform transform_BA_FA;
	tf::Transform transform_BA_FB;
	tf::Transform transform_FA_FB;
	tf::Transform transform_EE_FB;
	tf::Transform transform_CA_BA;
	tf::Transform transform_EE_GO;

	transform_CA_BA.setOrigin(transform_CA_BA_.getOrigin());
	transform_CA_BA.setRotation(transform_CA_BA_.getRotation() * q);

	/*
	ROS_INFO("*************");
	ROS_INFO("CA_BA :");
	printPose(transform_CA_BA);
	ROS_INFO("CA_BA inverse:");
	tf::Transform t;
	t = transform_CA_BA.inverse();
	printPose(t);
	ROS_INFO("CA_FA :");
	printPose(transform_CA_FA);
	ROS_INFO("CA_FB :");
	printPose(transform_CA_FB);
	 */

	moveit::planning_interface::MoveGroup group(PLANNING_GROUP_NAME);
	goal_pose.header.frame_id = BASE_LINK;
	goal_pose.header.stamp = ros::Time::now();
	group.setPoseReferenceFrame(BASE_LINK);

	//get the position of the end effector (= arm_7_joint)
	ee_pose.pose = group.getCurrentPose(EE_NAME).pose;
	current_ee_pose_.pose = group.getCurrentPose(EE_NAME).pose;
	//msg -> tf
	tf::poseMsgToTF(ee_pose.pose, ee_pose_tf);
	geometry_msgs::PoseStamped cam_msg;
	geometry_msgs::PoseStamped test_msg;
	geometry_msgs::PoseStamped test1_msg;
	geometry_msgs::PoseStamped test2_msg;
	geometry_msgs::PoseStamped test3_msg;
	geometry_msgs::PoseStamped test4_msg;
	geometry_msgs::PoseStamped test5_msg;
	geometry_msgs::PoseStamped test6_msg;
	geometry_msgs::PoseStamped base;

	//cam-base
	tf::poseTFToMsg(transform_CA_BA.inverse(), cam_msg.pose);
	//drawLine(0.55, 0.0, 0.0, 1.0, base, cam_msg);

	//base-cam-fiducial A
	transform_BA_FA.mult(transform_CA_BA.inverse(), transform_CA_FA);
	tf::poseTFToMsg(transform_BA_FA, test_msg.pose);
	//drawLine(0.9, 0.0, 0.0, 1.0, cam_msg, test_msg);

	//base-cam-fiducial B
	transform_BA_FB.mult(transform_CA_BA.inverse(), transform_CA_FB);
	tf::poseTFToMsg(transform_BA_FB, test4_msg.pose);
	//drawLine(0.55, 0.0, 0.0, 1.0, cam_msg, test4_msg);

	//drawLine(0.5, 0.5, 0.0, 1.0, base, ee_pose);
	//drawLine(0.0, 0.9, 0.0, 1.0, base, test_msg);
	//drawLine(0.0, 0.9, 0.0, 1.0, base, test4_msg);

	transform_FA_FB.mult(transform_BA_FA.inverse(), transform_BA_FB);
	//drawLine(0.55, 0.0, 0.0, 1.0, test_msg, test4_msg);

	transform_EE_FA.mult(ee_pose_tf.inverse(), transform_BA_FA);
	tf::poseTFToMsg(transform_EE_FA, test5_msg.pose);
	//drawLine(0.0, 0.0, 0.3, 1.0, ee_pose, test5_msg);

	transform_EE_FB.mult( ee_pose_tf * transform_EE_FA, transform_FA_FB );
	tf::poseTFToMsg( transform_EE_FB, test6_msg.pose);
	//drawLine(0.0, 0.0, 0.3, 1.0, ee_pose, test6_msg);
	//drawLine(0.0, 0.0, 0.3, 1.0, test6_msg, test5_msg);

	transform_EE_GO.mult(ee_pose_tf.inverse(), transform_EE_FB);
	//ROS_INFO("EE_GO :");
	//printPose(transform_EE_GO);
	goal_pose_tf.mult( ee_pose_tf , transform_EE_GO);

	goal_pose_tf.setRotation(goal_pose_tf.getRotation() * rotate_Y_90_right);
	goal_pose_tf.setRotation(goal_pose_tf.getRotation() * rotate_X_90_left);
	goal_pose_tf.setOrigin(ee_pose_tf.getOrigin());

	//ROS_INFO("goal :");
	//printPose(goal_pose_tf);

	//tf -> msg
	tf::poseTFToMsg(goal_pose_tf, goal_pose.pose);
	drawLine(0.55, 0.55, 0.0, 1.0, ee_pose, goal_pose);
	//drawSystem(goal_pose);
	drawArrowX(0.55, 0.0, 0.0, 1.0, goal_pose);

	group.setPoseTarget(goal_pose, EE_NAME);
	double distance = 0.0;
	//ROS_WARN_STREAM("distance " << distance << "!");
	if(distance < 2.0)
	{
		ROS_WARN_STREAM("STARTE TURN");
		// plan the motion
		bool have_plan = false;
		moveit::planning_interface::MoveGroup::Plan plan;
		have_plan = group.plan(plan);

		//EXECUTE THE PLAN !!!!!! BE CAREFUL
		if (have_plan==true) {
			group.execute(plan);
			group.move();
		}
		else
		{
			ROS_WARN("No valid plan found for the arm movement.");
			move_action_ = false;
			return false;
		}
	}

	current_ee_pose_.pose = group.getCurrentPose(EE_NAME).pose;
	move_action_ = true;

	return true;
}
/*
 * Move arm to the wagon using fiducials.
 */
/*
 * Execute a planning action with moveIt interface utilities.
 * The reference frame, the "base_link" frame,
 * initializes the starting point of the coordinate system.
 * the goal frame describes the end effector ("arm_7_link")
 * which will be moved to a new position.
 * Returns true, if the planned action has been executed.
 */
bool ToolChange::executeMoveCommand(const geometry_msgs::PoseStamped& goal_pose, const double offset)
{
	ROS_INFO("************** move ***********");
	move_action_ = false;

	geometry_msgs::PoseStamped pose;
	geometry_msgs::PoseStamped ee_pose;

	pose.pose = goal_pose.pose;

	moveit::planning_interface::MoveGroup group(PLANNING_GROUP_NAME);
	pose.header.frame_id = BASE_LINK;
	pose.header.stamp = ros::Time::now();
	group.setPoseReferenceFrame(BASE_LINK);

	ee_pose.pose = group.getCurrentPose(EE_NAME).pose;
	drawLine(0.55, 0.55, 0.0, 1.0, ee_pose, pose);
	drawSystem(pose);
	drawArrowX(0.55, 0.0, 0.0, 1.0, pose);
	group.setPoseTarget(pose, EE_NAME);

	// plan the motion
	bool have_plan = false;
	moveit::planning_interface::MoveGroup::Plan plan;
	have_plan = group.plan(plan);

	ROS_WARN("STARTE MOVE");
	//EXECUTE THE PLAN !!!!!! BE CAREFUL
	if (have_plan==true) {
		group.execute(plan);
		group.move();
	}
	else
	{
		ROS_WARN("No valid plan found for the arm movement.");
		move_action_ = false;
		return false;
	}

	current_ee_pose_.pose = group.getCurrentPose(EE_NAME).pose;
	move_action_ = true;

	return true;
}

/*
 * Executes straight movement with the help of
 * cartesian path. {@variable ee_max_step} is the maximum step
 * between the interpolated points for the resulting path and
 * {@variable jump_threshold} is the fraction of the path
 * or the average distance.
 * jump_threshold, also called "jump", is disabled if it is set to 0.0.
 */
bool ToolChange::executeStraightMoveCommand(const tf::Vector3& goal_direction, const double ee_max_step)
{
	/** \brief Compute a Cartesian path that follows specified waypoints with a step size of at most \e eef_step meters
	      between end effector configurations of consecutive points in the result \e trajectory. The reference frame for the
	      waypoints is that specified by setPoseReferenceFrame(). No more than \e jump_threshold
	      is allowed as change in distance in the configuration space of the robot (this is to prevent 'jumps' in IK solutions).
	      Collisions are avoided if \e avoid_collisions is set to true. If collisions cannot be avoided, the function fails.
	      Return a value that is between 0.0 and 1.0 indicating the fraction of the path achieved as described by the waypoints.
	      Return -1.0 in case of error. */

	move_action_ = false;
	double jump_threshold = 0.0;

	execute_known_traj_client_ = node_handle_.serviceClient<moveit_msgs::ExecuteKnownTrajectory>("/execute_kinematic_path");

	geometry_msgs::PoseStamped pose;
	geometry_msgs::PoseStamped ee_pose;
	tf::Transform pose_tf;
	tf::Transform ee_pose_tf;
	tf::Transform transf;
	tf::Transform t;

	moveit::planning_interface::MoveGroup group(PLANNING_GROUP_NAME);
	pose.header.frame_id = BASE_LINK;
	pose.header.stamp = ros::Time::now();
	group.setPoseReferenceFrame(BASE_LINK);

	ee_pose.pose = group.getCurrentPose(EE_NAME).pose;
	tf::poseMsgToTF(ee_pose.pose, ee_pose_tf);

	transf.setOrigin(goal_direction);
	t.mult(ee_pose_tf, transf);
	pose_tf.setOrigin(t.getOrigin());
	pose_tf.setRotation(ee_pose_tf.getRotation());

	//tf -> msg
	tf::poseTFToMsg(pose_tf, pose.pose);
	ROS_WARN("STARTE STRAIGT MOVE");
	executeMoveCommand(pose, 0.0);

	group.setPoseTarget(pose);

	// set waypoints for which to compute path
	std::vector<geometry_msgs::Pose> waypoints;
	waypoints.push_back(group.getCurrentPose().pose);
	waypoints.push_back(pose.pose);

	//DRAW IN RVIZ
	drawLine(0.55, 0.85, 0.0, 1.0, ee_pose, pose);
	//END DRAW

	moveit_msgs::ExecuteKnownTrajectory srv;
	// compute cartesian path
	double frac = group.computeCartesianPath(waypoints, ee_max_step, jump_threshold, srv.request.trajectory, false);

	ROS_WARN_STREAM(" Fraction is " << frac<< "%.");

	if(frac < 0){
		// no path could be computed
		ROS_ERROR("Unable to compute Cartesian path!");
		move_action_ = true;
		return false;
	} else if (frac < 1){
		// path started to be computed, but did not finish
		ROS_WARN_STREAM("Cartesian path computation finished " << frac * 100 << "% only!");
		move_action_ = true;
		return false;
	}

	// send trajectory to arm controller
	srv.request.wait_for_execution = true;
	execute_known_traj_client_.call(srv);

	current_ee_pose_.pose = group.getCurrentPose(EE_NAME).pose;
	move_action_ = true;

	return true;
}
/*
 * A helper function to slow the execution
 * of the server commands.
 */
void ToolChange::waitForMoveit()
{
	while(!move_action_)
	{
		ros::spinOnce();
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
void ToolChange::drawArrowX (const double r, const double g, const double b, const double a,
		const geometry_msgs::PoseStamped& pose)
{
	visualization_msgs::Marker marker;
	marker.header.frame_id = "base_link";
	marker.header.stamp = ros::Time::now();
	marker.id = marker_id_;
	marker.action = visualization_msgs::Marker::ADD;
	marker.type = visualization_msgs::Marker::ARROW;

	marker.pose.position.x = pose.pose.position.x ;
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

	marker.scale.x = 0.009;
	marker.scale.y = 0.0;
	marker.scale.z = 0.0;

	marker.color.r = r;
	marker.color.g = g;
	marker.color.b = b;
	marker.color.a = a;

	vis_pub_.publish(marker);
	marker_id_++;
}

void ToolChange::drawSystem(const geometry_msgs::PoseStamped& pose)
{

	geometry_msgs::PoseStamped goal_pose;
	goal_pose.pose = pose.pose;
	goal_pose.pose.position.x = pose.pose.position.x + 0.30;
	drawLine(0.55, 0.0, 0.0, 1.0, pose, goal_pose);
	goal_pose.pose = pose.pose;
	goal_pose.pose.position.y = pose.pose.position.y + 0.30;
	drawLine(0.3, 0.3, 0.0, 1.0, pose, goal_pose);
	goal_pose.pose = pose.pose;
	goal_pose.pose.position.z = pose.pose.position.z + 0.30;
	drawLine(0.0, 0.0, 1.0, 1.0, pose, goal_pose);
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

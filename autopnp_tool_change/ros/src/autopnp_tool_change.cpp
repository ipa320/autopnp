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
 * - starts all needed subscriptions and publications;
 * - starts action server in false mode;
 *
 * Makes the server sleep while the data in question
 * have not been received jet. The spinOnce operation
 * allows the calculation of the background processes
 * and sleeping functions to run simultaneously.
 */
ToolChange::ToolChange(ros::NodeHandle nh)
{
	std::cout << "Starting server..." << std::endl;

	node_handle_ = nh;
	input_marker_detection_sub_.unsubscribe();
	slot_position_detected_ = false;
	move_action_ = false;
	marker_id_ = 0;

	//SUBSCRIBERS
	vis_pub_ = node_handle_.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
	input_marker_detection_sub_.subscribe(node_handle_, "input_marker_detections", 1);
	input_marker_detection_sub_.registerCallback(boost::bind(&ToolChange::markerInputCallback, this, _1));

	//waiting till all fiducials detected.
	//Do not start servers before !!
	while(slot_position_detected_ == false)
	{
		ros::spinOnce();
	}

	//SERVERS
	resetServers();
}
/*
 * Reset the servers to false. Let them start and wait for a goal message.
 */
void ToolChange::resetServers()
{
   //RESET

	//moves the arm to a start position in front of the wagon
	as_go_to_start_position_.reset(new actionlib::SimpleActionServer<autopnp_tool_change::GoToStartPositionAction>(
			node_handle_, GO_TO_START_POSITION_ACTION_NAME, boost::bind(&ToolChange::goToStartPosition, this, _1), false));
	as_go_to_start_position_->start();

	//moves the arm to a chosen slot or tool on the wagon
	as_move_to_chosen_tool_.reset(new actionlib::SimpleActionServer<autopnp_tool_change::GoToStartPositionAction>(
			node_handle_, MOVE_TO_CHOSEN_TOOL_ACTION_NAME, boost::bind(&ToolChange::moveToChosenTool, this, _1), false));
	as_move_to_chosen_tool_->start();

}

/*
 * Set the garbage container free  and shutdown running processes !!!!!!
 */
ToolChange::~ToolChange()
{

}

/*
 * Running server after initialization process.
 * Make it spin around waiting for a goal messages.
 */
void ToolChange::run()
{
	ROS_INFO("tool_change spinning");
	ros::spin();
}

/**
 * Callback for the incoming data stream.
 * Retrieves coming data from the input_marker_detections message
 * and calculates the distance and orientation between the position
 * of the arm and board setting the {@value arm_board}.
 * Sets the variable {@value slot_position_detected} as true,
 * if the calculations took place and saves the last transformations as globals.
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

		//use marker components only if both (arm and board) markers detected
		//else the components are empty
		if(detected_all_fiducials_ == true)
		{
			//calculate transformation FA_FB
			arm_board = calculateArmBoardTransformation(result_components.board_.translation, result_components.arm_.translation);

			//allow no empty messages. There is always some distance between FA and FB !!!!
			if(!arm_board.getOrigin().isZero())
			{
				transform_CA_FA_ = result_components.arm_.translation;
				transform_CA_FB_ = result_components.board_.translation;

				//everything is fine, the server can start
				slot_position_detected_ = true;
			}

		}
		else
		{
			//markers are not visible or error occurred.
			slot_position_detected_ = false;
			//make the
			resetServers();
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
	tf::Point translation;
	tf::Quaternion orientation;

	for (unsigned int i = 0; i < input_marker_detections_msg->detections.size(); ++i)
	{
		//retrieve the number of label
		std::string fiducial_label = input_marker_detections_msg->detections[i].label;

		//convert translation and orientation Points msgs to tf Pose respectively
		tf::pointMsgToTF(input_marker_detections_msg->detections[i].pose.pose.position, translation);
		tf::quaternionMsgToTF(input_marker_detections_msg->detections[i].pose.pose.orientation, orientation);

		// average only the 3 markers from the board. Set the average on initial position of the {@value VAC_CLEANER)
		if (fiducial_label.compare(VAC_CLEANER)==0 || fiducial_label.compare(ARM_STATION)==0 || fiducial_label.compare(EXTRA_FIDUCIAL)==0)
		{
			detected_board_fiducial = true;
			count++;
			if (count==1)
			{
				result.board_.translation.setOrigin(translation);
				result.board_.translation.setRotation(orientation);
			}
			else
			{
				result.board_.translation.getOrigin() += translation;
				result.board_.translation.getRotation() += orientation;
			}
		}

		//if(fiducial_label_num == arm_marker_number)
		if (fiducial_label.compare(ARM)==0)
		{
			detected_arm_fiducial = true;
			result.arm_.translation.setOrigin(translation);
			result.arm_.translation.setRotation(orientation);
			result.arm_.translation.getRotation().normalize();
		}

		/*if (fiducial_label.compare("tag_3")==0)
{
detected_cam_fiducial = true;
result.cam.translation.setOrigin(translation);
result.cam.translation.setRotation(orientation);
result.cam.translation.getRotation().normalize();
}
		 */
	}
	if(count != 0)
	{
		result.board_.translation.getOrigin() /=(double)count;
		result.board_.translation.getRotation() /= (double)count;
		result.board_.translation.getRotation().normalize();
	}
	detected_all_fiducials_ = detected_arm_fiducial && detected_board_fiducial;

	return result;
}

/*
 * Calculates translation and orientation distance between arm marker and wagon board.
 * Vector orientation points from arm to board.
 *
 * ================================================
 *          FA  --------------->  FB
 *           ^                     ^
 *           |                     |
 *           |_____________________|
 *                    CAM
 * ================================================
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
 * comes to go_to_start_position server.
 * ==============================================
 *     ARM        ||  VAC_CLEANER    ||   X    ||
 * ==============================================
 *                     start
 * ==============================================
 *                     GOAL_FIDUCIAL
 *                         |
 *                         |
 *                        move
 *                         |
 *                        turn
 *                         |
 *                    (ARM_FIDUCIAL)
 *
 *================================================
 *             (result: succeeded/not succeeded)
 *================================================
 */
void ToolChange::goToStartPosition(const autopnp_tool_change::GoToStartPositionGoalConstPtr& goal)
{
	ROS_INFO("GoToStartPosition received new goal:  %s", goal->goal.c_str());
	bool success = false;

	//move to a start position
	success = processGoToStartPosition();

	autopnp_tool_change::GoToStartPositionResult result;
	std::string feedback;

	//set response
	if(success)
	{
		ROS_INFO("GoToStartPosition was successful !");
		//result.result = true;
		feedback ="ARM ON THE START POSITION !!";
		as_go_to_start_position_->setSucceeded(result, feedback);
	}
	else
	{
		ROS_INFO("GoToStartPosition failed !");
		//result.result = true;
		feedback ="FAILD TO GET TO THE START POSITION !!";
		as_go_to_start_position_->setAborted(result, feedback);
	}
}

/*
 * Processes movement to the start position in front of
 * the wagon:
 *
 * - first: end effector gets the goal orientation and executes the turn action;
 * - second: end effector moves to the position using the transformation
 *  between arm and wagon reference fiducials.
 *
 *===============================================
 *     ARM        ||  VAC_CLEANER    ||   X    ||
 * ==============================================
 *                     start
 * ==============================================
 *                     GOAL_FIDUCIAL
 *                         |
 *                         |
 *                        move
 *                         |
 *                        turn
 *                         |
 *                    (ARM_FIDUCIAL)
 *
 *================================================
 *                 (return true/false)
 *================================================
 */
bool ToolChange::processGoToStartPosition()
{
	//rotate
	ROS_INFO("GoToStartPosition process TURN");
	if(!moveToWagonFiducial(TURN))
	{
		ROS_WARN("Error occurred executing turn to wagon fiducial position.");
		return false;
	}

	//move to position
	ROS_INFO("GoToStartPosition process MOVE");
	if(!moveToWagonFiducial(MOVE))
	{
		ROS_WARN("Error occurred executing move to wagon fiducial position.");
		return false;
	}

	return true;
}



/*
 * This callback function is executed each time a request (= goal message)
 * comes to move_to_chosen_tool server.
 *
 * ==============================================
 *     ARM        ||  VAC_CLEANER    ||   X    ||
 * ==============================================
 *                     start
 * ==============================================
 *            offset
 *      <---------------->
 *
 *================================================
 *           (result: succeeded/not succeeded)
 *================================================
 */
void ToolChange::moveToChosenTool(const autopnp_tool_change::GoToStartPositionGoalConstPtr& goal)
{
	ROS_INFO("MoveToChosenTool received new goal:  %s", goal->goal.c_str());
	bool success = false;

	std::string tool_name = goal->goal;

	if(tool_name.compare(VAC_CLEANER) == 0)
	{
		//move to start position in front of the vacuum cleaner
		success = processMoveToChosenTool(VAC_CLEANER_OFFSET);
	}
	else if(tool_name.compare(ARM_STATION) == 0)
	{
		// move to start position in front of the arm slot
		success = processMoveToChosenTool(ARM_STATION_OFFSET);
	}
	autopnp_tool_change::GoToStartPositionResult result;
	std::string feedback;

	//set the response
	if(success)
	{
		ROS_INFO("GoToStartPosition was successful!");
		//result.result = true;
		feedback ="ARM ON THE CHOSEN TOOL POSITION !!";
		as_move_to_chosen_tool_->setSucceeded(result, feedback);
	}
	else
	{
		ROS_INFO("moveToChosenTool failed!");
		//result.result = true;
		feedback ="FAILD TO GET TO THE CHOSEN TOOL POSITION !!";
		as_move_to_chosen_tool_->setAborted(result, feedback);
	}
}

/*
 * Processes movement to a position of a chosen tool using an offset vector.
 * It can be the position of a vacuum cleaner or of an arm
 * on the wagon.
 *
 * ==============================================
 *     ARM        ||  VAC_CLEANER    ||   X    ||
 * ==============================================
 *                     start
 * ==============================================
 *            offset
 *      <---------------->
 *================================================
 *              (return true/false)
 *================================================
 */
bool ToolChange::processMoveToChosenTool(const tf::Vector3& offset)
{
	tf::Vector3 movement;
	movement = offset;

	if(!executeStraightMoveCommand(movement, MAX_STEP_CM))
	{
		ROS_WARN("Error occurred executing move to wagon fiducial.");
		return false;
	}
	return true;
}

/*
 * Sets the global values to identity.
 */
void ToolChange::clearFiducials()
{
	transform_CA_FA_.setIdentity();
	transform_CA_FB_.setIdentity();
}

/*
 * Executes a planning action with moveIt interface utilities.
 * The reference frame, the "base_link" frame,
 * initializes the starting point of the coordinate system.
 * the goal frame describes the end effector ("arm_7_link")
 * which will be moved to a new position.
 *
 * The goal position is known after some transformations
 * between frames : camera, base, end effector, arm fiducial and wagon fiducial.
 *
 * The Transformation of EE_FA (end effector and fiducial arm) is a
 * fixed transformation, depending on where and how far
 * the fiducial is placed/sticked on the arm.
 *
 * The Transformation EE_GO is fixed and improved through
 * distance offset (between wagon and start position, cm)
 * and rotation (between end effector and board fiducial)
 *
 */
bool ToolChange::moveToWagonFiducial(const std::string& action)
{
	move_action_ = false;
	geometry_msgs::PoseStamped ee_pose;
	geometry_msgs::PoseStamped goal_pose;
	tf::Transform ee_pose_tf;
	tf::Transform goal_pose_tf;

	tf::Transform transform_CA_FA;
	transform_CA_FA.setOrigin(transform_CA_FA_.getOrigin());
	transform_CA_FA.setRotation(transform_CA_FA_.getRotation());
	tf::Transform transform_CA_FB;
	transform_CA_FB.setOrigin(transform_CA_FB_.getOrigin() );
	transform_CA_FB.setRotation(transform_CA_FB_.getRotation());
	tf::Transform transform_EE_FA;
	transform_EE_FA.setOrigin(ARM_FIDUCIAL_OFFSET);
	transform_EE_FA.setRotation(ARM_FIDUCIAL_ORIENTATION_OFFSET);

	tf::Transform transform_BA_FA;
	tf::Transform transform_BA_FB;
	tf::Transform transform_CA_BA;

	tf::Transform transform_FA_FB;
	tf::Transform transform_EE_FB;
	tf::Transform transform_EE_GO;
	tf::Transform transform_EE_GO_schort;


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
	geometry_msgs::PoseStamped test7_msg;
	geometry_msgs::PoseStamped test8_msg;
	geometry_msgs::PoseStamped base;


	/*
//+++++++++++++++++++++++
// ARM FIDUCIAL TRANSFORMATION
tf::Quaternion q;
q.setRPY(0.0, 0.0, -M_PI);
tf::Transform cam;
tf::Transform base_cam;
tf::Transform arm_fiducial;
cam.setOrigin(transform_CA_BA_.getOrigin());
cam.setRotation(transform_CA_BA_.getRotation() * q);
//base-cam-fiducial A
base_cam.mult(cam.inverse(), transform_CA_FA);
arm_fiducial.mult(ee_pose_tf.inverse(), base_cam);
ROS_INFO(" TRANSFORMATION :");
printPose(cam);
printPose(arm_fiducial);
//+++++++++++++++++++++++++++++++++++
	 */

	//base FA
	tf::poseTFToMsg(transform_CA_FA, test_msg.pose);
	//base FB
	tf::poseTFToMsg(transform_CA_FB, test1_msg.pose);

	//BA FA
	transform_BA_FA.mult(ee_pose_tf, transform_EE_FA);
	tf::poseTFToMsg(transform_BA_FA, test2_msg.pose);
	//drawLine(0.20, 0.0, 0.0, 1.0, base, test2_msg);
	//CA BA
	transform_CA_BA.mult(transform_CA_FA, transform_BA_FA.inverse());
	tf::poseTFToMsg(transform_CA_BA.inverse(), cam_msg.pose);
	//drawLine(0.55, 0.0, 0.0, 1.0, base, cam_msg);
	//CA FA FB
	transform_FA_FB.mult(transform_BA_FA.inverse(), transform_CA_FB);
	//BA FB
	transform_BA_FB.mult(transform_CA_BA.inverse(), transform_CA_FB);
	tf::poseTFToMsg( transform_BA_FB, test6_msg.pose);

	//drawLine(0.50, 0.0, 0.0, 1.0, base, test6_msg);
	//drawLine(0.0, 0.0, 0.3, 1.0, test6_msg, test2_msg);

	//drawLine(0.5, 0.5, 0.0, 1.0, cam_msg, test2_msg);
	//drawLine(0.5, 0.5, 0.0, 1.0, cam_msg, test6_msg);


	transform_EE_GO.mult(transform_BA_FB, transform_EE_FA);
	tf::poseTFToMsg( transform_EE_GO, test5_msg.pose);
	//drawLine(0.0, 0.30, 0.0, 1.0, base, test5_msg);


	// just move without rotation
	if(action.compare(MOVE)== 0)
	{
		transform_EE_FB.mult(ee_pose_tf.inverse(), transform_EE_GO);
		tf::poseTFToMsg( transform_EE_FB, test8_msg.pose);
		//drawLine(0.0, 0.30, 0.0, 1.0, base, test8_msg);


		transform_EE_GO_schort.mult(ee_pose_tf.inverse(), transform_EE_FB);
		transform_EE_GO_schort.setOrigin(transform_EE_FB.getOrigin() + TOOL_FIDUCIAL_OFFSET_0);
		goal_pose_tf.mult( ee_pose_tf , transform_EE_GO_schort);

		goal_pose_tf.setRotation(ee_pose_tf.getRotation());
		goal_pose_tf.setOrigin(goal_pose_tf.getOrigin());

	}
	//just rotate without movement
	else if(action.compare(TURN)==0)
	{
		tf::Quaternion s;
		s.setRPY(4.0, 0.0, 0.0);
		//goal_pose_tf.setOrigin(transform_EE_GO.getOrigin());
		goal_pose_tf.setOrigin(ee_pose_tf.getOrigin());
		goal_pose_tf.setRotation(transform_EE_GO.getRotation() * s);

	}

	//tf -> msg
	tf::poseTFToMsg(goal_pose_tf, goal_pose.pose);
	drawLine(0.55, 0.55, 0.0, 1.0, ee_pose, goal_pose);
	//drawSystem(goal_pose);
	drawArrowX(0.55, 0.0, 0.0, 1.0, goal_pose);


	double length = transform_EE_GO_schort.getOrigin().length();
	ROS_WARN_STREAM(" distance in move :"<< length << ".");

	group.setPoseTarget(goal_pose, EE_NAME);

	if(length > MAX_TOLERANCE)
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
	}else{
		ROS_WARN_STREAM(" no movement occured with length "<< length <<".");
	}

	current_ee_pose_.pose = group.getCurrentPose(EE_NAME).pose;
	move_action_ = true;
	clearFiducials();

	return true;
}




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
	/** \brief Compute a Cartesian path that follows specified waypoints with a step size of at most  eef_step meters
between end effector configurations of consecutive points in the result  trajectory. The reference frame for the
waypoints is that specified by setPoseReferenceFrame(). No more than jump_threshold
is allowed as change in distance in the configuration space of the robot (this is to prevent 'jumps' in IK solutions).
Collisions are avoided if avoid_collisions is set to true. If collisions cannot be avoided, the function fails.
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
	toolChange.run();
	//ros::spin();

	return (0);
}

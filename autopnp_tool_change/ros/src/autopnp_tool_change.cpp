#include <autopnp_tool_change/autopnp_tool_change.h>
#include <math.h>
#include <moveit/move_group_interface/move_group.h>

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

/*
 /base_link /fiducial/tag_board
At time 1403881288.362
- Translation: [-0.660, -0.215, 1.010]
- Rotation: in Quaternion [0.413, 0.574, 0.563, 0.428]
            in RPY [1.572, 0.027, 1.868]
At time 1403881289.373
- Translation: [-0.660, -0.215, 1.010]
- Rotation: in Quaternion [0.413, 0.574, 0.563, 0.428]
            in RPY [1.570, 0.027, 1.868]
At time 1403881290.377
- Translation: [-0.660, -0.215, 1.010]
- Rotation: in Quaternion [0.413, 0.574, 0.563, 0.428]
            in RPY [1.571, 0.026, 1.868]
 */

ToolChange::ToolChange(ros::NodeHandle nh)
: transform_listener_(nh), br_()
{
	std::cout << "Starting server..." << std::endl;

	node_handle_ = nh;
	input_marker_detection_sub_.unsubscribe();
	slot_position_detected_ = false;
	move_action_state_ = false;
	marker_id_ = 0;

	//SUBSCRIBERS
	vis_pub_ = node_handle_.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
	input_marker_detection_sub_.subscribe(node_handle_, "input_marker_detections", 1);
	input_marker_detection_sub_.registerCallback(boost::bind(&ToolChange::markerInputCallback, this, _1));

	//wait till all fiducials have been detected.
	//Do not start servers before !!

	while(slot_position_detected_ == false)
	{
		ros::spinOnce();
	}

	//SERVERS
	resetServers();

	ROS_INFO("Done init");
}
/*
 * Reset the servers to false. Let them start and wait for a goal message.
 */
void ToolChange::resetServers()
{
	ROS_INFO("Reseting servers.");
	//RESET
	//moves the arm to a start position in front of the wagon
	as_go_to_start_position_.reset(new actionlib::SimpleActionServer<autopnp_tool_change::GoToStartPositionAction>(
			node_handle_, GO_TO_START_POSITION_ACTION_NAME, boost::bind(&ToolChange::goToStartPosition, this, _1), false));
	as_go_to_start_position_->start();

	//moves the arm to a chosen slot or tool on the wagon
	as_go_to_slot_and_turn_.reset(new actionlib::SimpleActionServer<autopnp_tool_change::GoToStartPositionAction>(
			node_handle_, GO_TO_SLOT_AND_TURN_ACTION_NAME, boost::bind(&ToolChange::goToSlotAndTurn, this, _1), false));
	as_go_to_slot_and_turn_->start();

	//moves the arm to a chosen slot or tool on the wagon
	as_go_back_to_start_.reset(new actionlib::SimpleActionServer<autopnp_tool_change::GoToStartPositionAction>(
			node_handle_, GO_BACK_TO_START_ACTION_NAME, boost::bind(&ToolChange::goBackToStart, this, _1), false));
	as_go_back_to_start_->start();

}

/*
 * TO DO: Set the garbage container free
 * and shutdown running processes !!!!!!
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

	if (input_marker_detections_msg->detections.size() != 0 )
	{
		//set marker components if such detected
		computeMarkerPose(input_marker_detections_msg);

		//use marker components only if both (arm and board) markers detected
		//else the components are empty
		if(detected_all_fiducials_ == true)
		{
			slot_position_detected_ = true;
		}
		else
		{
			//ROS_WARN("Not all fiducials are detected.");
			//markers are not visible or error occurred.
			slot_position_detected_ = false;
		}
	}
}

/*
 * Computes mean coordinate system if multiple markers detected
 * and saves the data as an array of two fiducial objects
 * {@value arm, @value board} with the transform data
 * {@value translation} respectively.
 */
void ToolChange::computeMarkerPose(
		const cob_object_detection_msgs::DetectionArray::ConstPtr& input_marker_detections_msg)
{
	ToolChange::components result;
	unsigned int count = 0;
	detected_all_fiducials_ = false;
	bool detected_arm_fiducial = false;
	bool detected_board_fiducial = false;
	tf::Point translation;
	tf::Quaternion orientation = tf::createIdentityQuaternion();
	//static tf::TransformBroadcaster br;
	std::string cam_name = "/head_cam3d_link";

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

		if (fiducial_label.compare(ARM)==0)
		{
			std::string fidu_name_arm = "/fiducial/tag_2";
			detected_arm_fiducial = true;
			result.arm_.translation.setOrigin(translation);
			result.arm_.translation.setRotation(orientation);
			result.arm_.translation.getRotation().normalize();

			std::string fiducial_arm_real = "/arm_7_link_real";
			tf::StampedTransform stamped_transform_FA_EE;
			stamped_transform_FA_EE.setIdentity();
			stamped_transform_FA_EE.setOrigin(FA_EE_OFFSET);
			stamped_transform_FA_EE.setRotation(FA_EE_ORIENTATION_OFFSET);

			ros::Time time = ros::Time::now();

			try
			{
				//broadcast pose to tf
				static tf::TransformBroadcaster br;
				br_.sendTransform(tf::StampedTransform(result.arm_.translation, time,
						cam_name, fidu_name_arm ));

				br_.sendTransform(tf::StampedTransform(stamped_transform_FA_EE, time,
						fidu_name_arm, fiducial_arm_real));
			}
			catch (tf::TransformException ex)
			{
				ROS_ERROR("Broadcaster unavailable %s", ex.what());
			}
		}
	}
	if(count != 0)
	{
		ros::Time time = ros::Time::now();
		std::string fidu_name_board = "/fiducial/tag_board";
		result.board_.translation.getOrigin() /=(double)count;
		result.board_.translation.getRotation() /= (double)count;
		result.board_.translation.getRotation().normalize();

		std::string tool_change_start_pose = "/fiducial/start_point";
		tf::Quaternion start_point_rotation = tf::createIdentityQuaternion();
		start_point_rotation.setRPY(M_PI/2, -M_PI/2, 0.0);
		tf::Transform fidu_board_translated(start_point_rotation,
				START_POINT_OFFSET);

		std::string fiducial_reference = "/fiducial/reference";
		tf::Quaternion reference_point_rotation = tf::createIdentityQuaternion();
		reference_point_rotation.setRPY( 0.0, 0.0, -M_PI/2);
		//reference_point_rotation.setRPY( M_PI/2, -M_PI/2, 0.0);
		tf::Transform fidu_reference_translated(reference_point_rotation,
				tf::Vector3(0.0,0.0,0.0));

		std::string fiducial_slot = "/fiducial/slot_point";
		tf::Transform fidu_reference_in( start_point_rotation,
				SLOT_POINT_OFFSET);

		try
		{
			//broadcast pose to tf
			br_.sendTransform(tf::StampedTransform(result.board_.translation, time,
					cam_name, fidu_name_board ));

			br_.sendTransform(tf::StampedTransform(fidu_board_translated, time,
					fidu_name_board, tool_change_start_pose ));

			br_.sendTransform(tf::StampedTransform(fidu_reference_translated, time,
					fidu_name_board, fiducial_reference));

			br_.sendTransform(tf::StampedTransform(fidu_reference_in, time,
					fidu_name_board, fiducial_slot));

		}
		catch (tf::TransformException ex)
		{
			ROS_ERROR("Broadcaster unavailable %s", ex.what());
		}
	}
	detected_all_fiducials_ = detected_arm_fiducial && detected_board_fiducial;

}

/*
 * This callback function is executed each time a client request
 * comes to go_to_start_position server. It executes the movement to
 * the defined start position.
 *
 * ==============================================
 *     ARM        ||  VAC_CLEANER    ||   X    ||
 * ==============================================
 *    start(goal=arm)        start(goal=vac)
 * ==============================================
 *          |               |
 *          |               |
 *          |               |
 *            (ARM_FIDUCIAL)
 *
 *================================================
 *         (result: succeeded/not succeeded)
 *================================================
 */
void ToolChange::goToStartPosition(const autopnp_tool_change::GoToStartPositionGoalConstPtr& goal)
{

	ROS_INFO("GoToStartPosition received new goal:  %s", goal->goal.c_str());

	std::string received_goal = goal->goal;
/*
	while(detected_all_fiducials_ == false)
	{
		//ROS_WARN("No fiducials detected. Spinning and waiting.");
		ros::spinOnce();
	}
	*/
	//move to a start position
	bool success = processGoToStartPosition(received_goal);

	autopnp_tool_change::GoToStartPositionResult result;
	//std::string feedback;

	//set response
	if(success)
	{
		result.result = success;
		ROS_INFO("GoToStartPosition was successful %i !", (int) result.result);
		as_go_to_start_position_->setSucceeded(result);
	}
	else
	{
		result.result = success;
		ROS_ERROR("GoToStartPosition failed !");
		as_go_to_start_position_->setAborted(result);
	}
}

/*
 * Processes movement to the start position in front of
 * the wagon and correct errors:
 *
 * - first (move free): end effector moves to the position using the transformation
 *  between /base_link and /fiducial/start_point;
 * - second (turn): end effector orientation adjustment. Find two angles for x and y axes
 * from the reference axes of the board, so that z axes of the end effector shows straight down.
 * -third (move straight): move end effector position adjustment. Find an offset vector to the supposed position
 * of the end effector and correct this position. Execute straigt movements in x, y and z direction.
 */

bool ToolChange::processGoToStartPosition(const std::string& received_goal)
{
	//move to position
	ROS_INFO("GoToStartPosition process starts now");

	if(!moveToStartPosition(MOVE))
	{
		ROS_ERROR("Error occurred executing processGoToStartPosition MOVE.");
		return false;
	}

	for(int i = 0; i < 2; i++)
	{
		if(!moveToStartPosition(TURN))
		{
			ROS_ERROR("Error occurred executing processGoToStartPosition TURN.");
			return false;
		}
	}

	ros::Time now = ros::Time::now();
	//static tf::TransformBroadcaster br;
	tf::StampedTransform goal;
	goal.setIdentity();
	tf::StampedTransform offset_st;
	offset_st.setIdentity();
	geometry_msgs::PoseStamped goal_pose;
    std::string err;
	bool slot_position_detected = false;


	try{
		transform_listener_.getLatestCommonTime("/arm_7_link","/arm_7_link_real", now, &err );
		transform_listener_.lookupTransform( "/arm_7_link","/arm_7_link_real", now , offset_st);

		ROS_INFO("Transform exists");
		slot_position_detected = true;
	}
	catch (tf::TransformException ex)
	{
		slot_position_detected = false;
		ROS_ERROR(" Transform unavailable %s", ex.what());
		return false;
	}

	printPose(offset_st);


	double x = -offset_st.getOrigin().getX();
	double y = -offset_st.getOrigin().getY();
	double z = -offset_st.getOrigin().getZ();

	if(x != 0.0 && y != 0.0 && z != 0.0 && slot_position_detected == true)
	{
		tf::Vector3 movement = tf::Vector3(x, 0.0, 0.0);
		if(!executeStraightMoveCommand(movement, MAX_STEP_CM))
		{
			ROS_ERROR("Error occurred executing processGoToStartPosition.");
			return false;
		}

		movement = tf::Vector3(0.0,y, 0.0);
		if(!executeStraightMoveCommand(movement, MAX_STEP_CM))
		{
			ROS_ERROR("Error occurred executing processGoToStartPosition.");
			return false;
		}
		movement = tf::Vector3(0.0, 0.0, z);
		if(!executeStraightMoveCommand(movement, MAX_STEP_CM))
		{
			ROS_ERROR("Error occurred executing processGoToStartPosition.");
			return false;
		}
	}
	else
	{
		ROS_ERROR("process goToStartPosition failed. Transform unavailable.");
		return false;
	}


	return true;
}



/*
 * This callback function is executed each time a client request
 * comes to go_to_slot_and_turn server. It executes straight movements
 * from the start position to the slot position of the tool before
 * couple/uncouple action.
 *
 */
void ToolChange::goToSlotAndTurn(const autopnp_tool_change::GoToStartPositionGoalConstPtr& goal)
{
	ROS_INFO(":goToSlotAndTurn received new goal:  %s", goal->goal.c_str());
	bool success = false;
	std::string tool_name = goal->goal;
	/*
	while(detected_all_fiducials_ == false)
	{
		ROS_WARN("No fiducials detected. Spinning and waiting.");
		ros::spinOnce();
	}
	 */
	if(tool_name.compare(DEFAULT) == 0)
	{
		//move to slot straight once
		success = processGoToSlotAndTurn(tf::Vector3(-0.078, 0.0, 0.0));
	}
	else if(tool_name.compare(UP_AND_DOWN) == 0)
	{
		// move to start position in front of the arm slot
		//success = processGoToSlotAndTurn(up, back, down);
	}
	autopnp_tool_change::GoToStartPositionResult result;
	//std::string feedback;

	//set the response
	if(success)
	{
		result.result = success;
		ROS_INFO("GoToSlotAndTurn was successful!");
		result.result = true;
		//feedback ="ARM ON SLOT POSITION !!";
		as_go_to_slot_and_turn_->setSucceeded(result);
	}
	else
	{
		result.result = success;
		ROS_ERROR("GoToSlotAndTurn  failed!");
		result.result = true;
		//feedback ="FAILD TO GET TO SLOT POSITION !!";
		as_go_to_slot_and_turn_->setAborted(result);
	}
}

/*
 * Processes successively straight movements
 * from the start position to the slot position of the tool before
 * couple/uncouple action. The 3 parameter define the directions of maximal 3 movement.
 * The movement takes place, if the planned distance is not zero.
 */
bool ToolChange::processGoToSlotAndTurn(const tf::Vector3& movement1, const tf::Vector3& movement2, const tf::Vector3& movement3)
{
	tf::Vector3 move1,move2,move3;
	move1 = movement1;
	move2 = movement2;
	move3 = movement3;


		if(!executeStraightMoveCommand(move1, MAX_STEP_CM))
		{
			ROS_ERROR("Error occurred executing processGoToSlotAndTuren straight movement");
			return false;
		}

		if(!executeStraightMoveCommand(move2, MAX_STEP_CM))
		{
			ROS_ERROR("Error occurred executing processGoToSlotAndTuren straight movement");
			return false;
		}

		if(!executeStraightMoveCommand(move3, MAX_STEP_CM))
		{
			ROS_ERROR("Error occurred executing processGoToSlotAndTuren straight movement");
			return false;
		}

	return true;
}

/*
 * Processes a straight movement
 * from the start position to the slot position of the tool before
 * couple/uncouple action.
 * */
bool ToolChange::processGoToSlotAndTurn(const tf::Vector3& movement1)
{
	tf::Vector3 move1,move2,move3;
	move1 = movement1;


		if(!executeStraightMoveCommand(move1, MAX_STEP_CM))
		{
			ROS_ERROR("Error occurred executing processGoToSlotAndTuren straight movement");
			return false;
		}

	tf::Quaternion rotate_x_offset = tf::createIdentityQuaternion();
	rotate_x_offset.setRPY(0.0, 0.0, TOOL_CHANGER_OFFSET_TO_X_AXES);

	if(!executeTurn(rotate_x_offset))
	{
		ROS_ERROR("Error occurred executing processGoToSlotAndTuren turn");
		return false;
	}

	return true;
}
/*
 * Executes a planning action with moveIt interface utilities.
 * The reference frame, the "base_link" frame,
 * initializes the starting point of the coordinate system.
 * the goal frame describes the end effector ("arm_7_link")
 * which will be moved to a new position.
 *
 */

bool ToolChange::moveToStartPosition(const std::string& action)
{
	ROS_INFO("Execute %s ", action.c_str());
	move_action_state_ = false;
	bool slot_position_detected = false;
	geometry_msgs::PoseStamped ee_pose, goal_pose;
	tf::Transform ee_pose_tf = tf::Transform::getIdentity();
	tf::Transform goal_pose_tf = tf::Transform::getIdentity();
	tf::StampedTransform st_START_POINT;
	st_START_POINT.setIdentity();
	tf::StampedTransform tag2_to_ref;
	tag2_to_ref.setIdentity();
	tf::StampedTransform st_BA_FB;
	st_BA_FB.setIdentity();
	tf::Quaternion quat = tf::createIdentityQuaternion();
	double rall, pitch, yaw;

	moveit::planning_interface::MoveGroup group(PLANNING_GROUP_NAME);
	goal_pose.header.frame_id = BASE_LINK;
	goal_pose.header.stamp = ros::Time::now();
	group.setPoseReferenceFrame(BASE_LINK);
	ros::Time time = goal_pose.header.stamp;

	//get the position of the end effector (= arm_7_joint)
	ee_pose.pose = group.getCurrentPose(EE_NAME).pose;
	tf::poseMsgToTF(ee_pose.pose, ee_pose_tf);

	ros::Time now = ros::Time::now();

	try{
		transform_listener_.waitForTransform("/fiducial/tag_2", "/fiducial/reference", now, ros::Duration(3.0));
		transform_listener_.lookupTransform( "/fiducial/tag_2","/fiducial/reference",
				time, tag2_to_ref);

		transform_listener_.waitForTransform("/base_link", "/fiducial/start_point", now, ros::Duration(3.0));
		transform_listener_.lookupTransform( "/base_link", "/fiducial/start_point",
				time, st_BA_FB);

		transform_listener_.waitForTransform("/arm_7_link", "/fiducial/start_point", now, ros::Duration(3.0));
		transform_listener_.lookupTransform( "/arm_7_link", "/fiducial/start_point",
				time, st_START_POINT);
		ROS_INFO("Transform exists");
		slot_position_detected = true;
	}
	catch (tf::TransformException ex)
	{
		slot_position_detected = false;
		ROS_ERROR("Transform unavailable %s", ex.what());
		return false;
	}

	tf::Matrix3x3 m(tag2_to_ref.getRotation());
	m.getRPY(rall, pitch, yaw);
	ROS_INFO("rpy %f, %f, %f ",(float)rall,(float)pitch,(float)yaw);
	quat.setRPY( pitch, yaw, 0.0);


	//must be more than zero
	if(!st_BA_FB.getOrigin().isZero())

		// just move without rotation
		if(action.compare(MOVE)== 0)

		{
			goal_pose_tf.setOrigin(st_BA_FB.getOrigin());
			goal_pose_tf.setRotation(st_BA_FB.getRotation());
		}

	// just rotate
	if(action.compare(TURN)== 0 )
	{
		goal_pose_tf.setOrigin(st_BA_FB.getOrigin());
		//goal_pose_tf.setOrigin(ee_pose_tf.getOrigin());
		goal_pose_tf.setRotation(st_BA_FB.getRotation() * quat);
	}

	//tf -> msg
	tf::poseTFToMsg(goal_pose_tf, goal_pose.pose);

	double length = st_START_POINT.getOrigin().length();
	ROS_WARN_STREAM(" distance to move " << length << ".");

	group.setPoseTarget(goal_pose, EE_NAME);

	// plan the motion
	bool have_plan = false;
	moveit::planning_interface::MoveGroup::Plan plan;
	have_plan = group.plan(plan);

	//EXECUTE THE PLAN !!!!!! BE CAREFUL
	if (have_plan==true && slot_position_detected == true)
	{
		group.execute(plan);
		group.move();
	}
	else
	{
		ROS_WARN(" No valid plan found for the arm movement.");
		move_action_state_ = false;
		return false;
	}

	move_action_state_ = true;

	return true;
}

/*
 * Execute a planning action with moveIt interface utilities.
 * The reference frame, the "base_link" frame,
 * initializes the starting point of the coordinate system.
 * the goal frame describes the end effector ("arm_7_link")
 * which will be moved to a new position. The {@ goal_pose}
 * is relative to the end effector frame.
 * Returns true, if the planned action has been executed.
 */
bool ToolChange::executeTurn(const tf::Quaternion& quat)
{
	ROS_INFO("Start execute move command with the goal.");

	move_action_state_ = false;

	bool slot_position_detected = false;
	geometry_msgs::PoseStamped pose;
	geometry_msgs::PoseStamped ee_pose;
	tf::Transform ee_pose_tf = tf::Transform::getIdentity();
	tf::Transform current_tf = tf::Transform::getIdentity();

	moveit::planning_interface::MoveGroup group(PLANNING_GROUP_NAME);
	pose.header.frame_id = BASE_LINK;
	pose.header.stamp = ros::Time::now();
	group.setPoseReferenceFrame(BASE_LINK);

	ee_pose.pose = group.getCurrentPose(EE_NAME).pose;
	tf::poseMsgToTF(ee_pose.pose, ee_pose_tf);
	ros::Time now = ros::Time::now();
	printPose(ee_pose_tf);

	tf::StampedTransform st_BA_SLOT;
	st_BA_SLOT.setIdentity();

	try
	{
		transform_listener_.waitForTransform("/arm_7_link_real","/fiducial/slot_point", now, ros::Duration(3.0));
		transform_listener_.lookupTransform( "/arm_7_link_real","/fiducial/slot_point",
				pose.header.stamp, st_BA_SLOT);
		ROS_INFO("Transform exists");
		slot_position_detected = true;
	}
	catch (tf::TransformException ex)
	{
		slot_position_detected = false;
		ROS_ERROR("Transform unavailable %s", ex.what());
		return false;
	}

	current_tf.setOrigin(ee_pose_tf.getOrigin());
	current_tf.setRotation(ee_pose_tf.getRotation() * quat);
	//current_tf.setRotation(st_BA_SLOT.getRotation());

	tf::poseTFToMsg(current_tf, pose.pose);
	printPose(current_tf);

	//br_.sendTransform(tf::StampedTransform(current_tf, ros::Time::now(),
	//		"/base_link", "fiducial/drehung"));

	group.setPoseTarget(pose, EE_NAME);

	// plan the motion
	bool have_plan = false;
	moveit::planning_interface::MoveGroup::Plan plan;
	have_plan = group.plan(plan);

	//EXECUTE THE PLAN !!!!!! BE CAREFUL
	if (have_plan==true && slot_position_detected == true) {
		group.execute(plan);
		group.move();
	}
	else
	{
		ROS_WARN("No valid plan found for the arm movement.");
		move_action_state_ = false;
		return false;
	}
	move_action_state_ = true;

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
	ROS_INFO("Execute straight move command !");
	move_action_state_ = false;
	double jump_threshold = 0.0;


	execute_known_traj_client_ = node_handle_.serviceClient<moveit_msgs::ExecuteKnownTrajectory>("/execute_kinematic_path");

	geometry_msgs::PoseStamped pose;
	geometry_msgs::PoseStamped ee_pose;
	tf::Transform pose_tf= tf::Transform::getIdentity();
	tf::Transform ee_pose_tf= tf::Transform::getIdentity();
	tf::Transform transf= tf::Transform::getIdentity();
	tf::Transform t= tf::Transform::getIdentity();

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

	group.setPoseTarget(pose);

	// set waypoints for which to compute path
	std::vector<geometry_msgs::Pose> waypoints;
	waypoints.push_back(group.getCurrentPose().pose);
	waypoints.push_back(pose.pose);

	moveit_msgs::ExecuteKnownTrajectory srv;
	// compute cartesian path
	double frac = group.computeCartesianPath(waypoints, ee_max_step, jump_threshold, srv.request.trajectory, false);

	ROS_WARN_STREAM(" Fraction is " << frac<< "%.");

	if(frac < 0){
		// no path could be computed
		ROS_ERROR("Unable to compute Cartesian path!");
		move_action_state_ = true;
		return false;

	} else if (frac < 1){
		// path started to be computed, but did not finish
		ROS_WARN_STREAM("Cartesian path computation finished " << frac * 100 << "% only!");
		move_action_state_ = true;
		return false;
	}

	// send trajectory to arm controller
		srv.request.wait_for_execution = true;
		execute_known_traj_client_.call(srv);

	move_action_state_ = true;

	return true;
}
/*
 * A helper function to slow the execution
 * of the server commands.
 */
void ToolChange::waitForMoveit()
{
	while(!move_action_state_)
	{
		ros::spinOnce();
	}
}

void ToolChange::goBackToStart(const autopnp_tool_change::GoToStartPositionGoalConstPtr& goal)
{

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

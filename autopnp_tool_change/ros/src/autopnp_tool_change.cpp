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


ToolChange::ToolChange(ros::NodeHandle nh)
: transform_listener_(nh), br_()
{
	std::cout << "Starting server..." << std::endl;

	node_handle_ = nh;
	input_marker_detection_sub_.unsubscribe();
	slot_position_detected_ = false;
	move_action_state_ = false;

	//SUBSCRIBERS
	//vis_pub_ = node_handle_.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
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
	as_go_to_start_position_.reset(new actionlib::SimpleActionServer<autopnp_tool_change::ToolChangeAction>(
			node_handle_, GO_TO_START_POSITION_ACTION_NAME, boost::bind(&ToolChange::goToStartPosition, this, _1), false));
	as_go_to_start_position_->start();

	//moves the arm to a chosen slot or tool on the wagon
	as_go_to_slot_.reset(new actionlib::SimpleActionServer<autopnp_tool_change::ToolChangeAction>(
			node_handle_, GO_TO_SLOT_ACTION_NAME, boost::bind(&ToolChange::goToSlot, this, _1), false));
	as_go_to_slot_->start();

	//moves the arm to a chosen slot or tool on the wagon
	as_go_back_to_start_.reset(new actionlib::SimpleActionServer<autopnp_tool_change::ToolChangeAction>(
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
	tf::Transform tag_0;



	for (unsigned int i = 0; i < input_marker_detections_msg->detections.size(); ++i)
	{
		//retrieve the number of label
		std::string fiducial_label = input_marker_detections_msg->detections[i].label;

		//convert translation and orientation Points msgs to tf Pose respectively
		tf::pointMsgToTF(input_marker_detections_msg->detections[i].pose.pose.position, translation);
		tf::quaternionMsgToTF(input_marker_detections_msg->detections[i].pose.pose.orientation, orientation);


		// average only the 3 markers from the board. Set the average on initial position of the {@value VAC_CLEANER)
	//	if (fiducial_label.compare(VAC_CLEANER)==0 || fiducial_label.compare(ARM_STATION)==0 || fiducial_label.compare(EXTRA_FIDUCIAL)==0)
			if (fiducial_label.compare(EXTRA_FIDUCIAL)==0)

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

		if(fiducial_label.compare(TAG_0)== 0)
		{
			ros::Time time = ros::Time::now();
			tf::Quaternion rotation2 = tf::createIdentityQuaternion();
			rotation2.setRPY(0.0, M_PI, M_PI/2);
			tag_0.setOrigin(translation);
			tag_0.setRotation(orientation*rotation2);
			br_.sendTransform(tf::StampedTransform(tag_0, time,
					CAM, TAG_0 ));

		}

		if (fiducial_label.compare(ARM)==0)
		{
			detected_arm_fiducial = true;
			result.arm_.translation.setOrigin(translation);
			result.arm_.translation.setRotation(orientation);
			result.arm_.translation.getRotation().normalize();

			tf::StampedTransform stamped_transform_FA_EE;
			stamped_transform_FA_EE.setIdentity();
			stamped_transform_FA_EE.setOrigin(FA_EE_OFFSET);

			tf::Quaternion quat = tf::createIdentityQuaternion();
			//quat.setRPY(M_PI/2, -0.95, M_PI/2);
			quat.setRPY(M_PI/2, -0.95, M_PI/2);
			stamped_transform_FA_EE.setRotation(quat);
			stamped_transform_FA_EE.setOrigin(FA_EE_OFFSET);

			ros::Time time = ros::Time::now();

			try
			{
				//broadcast pose to tf
				static tf::TransformBroadcaster br;
				br_.sendTransform(tf::StampedTransform(result.arm_.translation, time,
						CAM, TAG_ARM ));

				br_.sendTransform(tf::StampedTransform(stamped_transform_FA_EE, time,
						TAG_ARM, ARM_7_LINK_REAL));

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
		result.board_.translation.getOrigin() /=(double)count;
		result.board_.translation.getRotation() /= (double)count;
		result.board_.translation.getRotation().normalize();

		tf::Quaternion start_point_rotation = tf::createIdentityQuaternion();
		//start_point_rotation.setRPY(M_PI/2, -M_PI/2, 0.0);
	// gut 	start_point_rotation = tf::Quaternion(0.481, -0.524, 0.476, 0.518);
		//start_point_rotation = tf::Quaternion(-0.480, 0.528, -0.498, -0.493);
		start_point_rotation = tf::Quaternion(0.474, -0.532, 0.493, 0.500);

		tf::Transform fidu_board_translated_arm((start_point_rotation),
				START_POINT_OFFSET_ARM);

		tf::Transform fidu_board_translated_couple_arm((start_point_rotation),
				START_POINT_OFFSET_COUPLE_ARM);

		tf::Transform fidu_board_translated_vac(start_point_rotation ,
				START_POINT_OFFSET_VAC);

		tf::Transform fidu_board_translated_couple_vac(start_point_rotation,
				START_POINT_OFFSET_COUPLE_VAC);


		tf::Quaternion quat = tf::createIdentityQuaternion();
		quat.setRPY(0.0, 0.0, TOOL_CHANGER_OFFSET_ANGLE);

		tf::Transform slot_down_arm( (start_point_rotation * quat),
				SLOT_POINT_DOWN_ARM);

		tf::Transform slot_arm( (start_point_rotation *quat ),
				SLOT_POINT_OFFSET_ARM);

		tf::Transform slot_vac( start_point_rotation * quat,
				SLOT_POINT_OFFSET_VAC);

		tf::Transform slot_couple_arm( (start_point_rotation * quat),
				SLOT_POINT_OFFSET_COUPLE_ARM);

		tf::Transform slot_couple_vac( start_point_rotation * quat,
				SLOT_POINT_OFFSET_COUPLE_VAC);

		tf::Transform slot_down_vac( (start_point_rotation * quat),
				SLOT_POINT_DOWN_VAC);

		try
		{
			//broadcast pose to tf
			br_.sendTransform(tf::StampedTransform(result.board_.translation, time,
					CAM, TAG_BOARD ));

			br_.sendTransform(tf::StampedTransform(fidu_board_translated_arm, time,
					TAG_BOARD, START_POSE_ARM ));

			br_.sendTransform(tf::StampedTransform(fidu_board_translated_couple_arm, time,
					TAG_BOARD, START_POSE_COUPLE_ARM ));

			br_.sendTransform(tf::StampedTransform(fidu_board_translated_vac, time,
					TAG_BOARD, START_POSE_VAC ));

			br_.sendTransform(tf::StampedTransform(fidu_board_translated_couple_vac, time,
					TAG_BOARD, START_POSE_COUPLE_VAC ));

			br_.sendTransform(tf::StampedTransform(slot_arm, time,
					TAG_BOARD, SLOT_POSE_ARM));

			br_.sendTransform(tf::StampedTransform(slot_down_arm, time,
					TAG_BOARD, SLOT_POSE_DOWN_ARM));


			br_.sendTransform(tf::StampedTransform(slot_couple_arm, time,
					TAG_BOARD, SLOT_POSE_COUPLE_ARM));

			br_.sendTransform(tf::StampedTransform(slot_vac, time,
					TAG_BOARD, SLOT_POSE_VAC));

			br_.sendTransform(tf::StampedTransform(slot_couple_vac, time,
					TAG_BOARD, SLOT_POSE_COUPLE_VAC));

			br_.sendTransform(tf::StampedTransform(slot_down_vac, time,
					TAG_BOARD, SLOT_POSE_DOWN_VAC));
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
 *     ARM        ||         VAC          ||
 * ==============================================
 *    start(goal=arm)        start(goal=vac)
 * ==============================================
 *          |               |
 *          |               |
 *          |               |
 *            (ARM_FIDUCIAL)
 *
 *================================================
 *
 */
void ToolChange::goToStartPosition(const autopnp_tool_change::ToolChangeGoalConstPtr& goal)
{

	ROS_INFO("GoToStartPosition received new goal  %s with state %s", goal->goal.c_str(), goal->state.c_str());

	std::string received_goal = goal->goal;
	std::string received_state = goal->state;

	//move to a start position
	bool success = processGoToStartPosition(received_goal, received_state);

	autopnp_tool_change::ToolChangeResult result;

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
 * Executes Movements to the
 * start pose in front of the wagon.
 *
 *  Algorithm:
 *
 * - executes once a PTP Movement to the start pose. Errors expected !!!
 * - optimize the angle
 * - optimize the current translation
 * - improve the current position using fiducials
 */

bool ToolChange::processGoToStartPosition(const std::string& received_goal,const std::string& received_state)
{
	ROS_INFO("PROCCESS goal  %s with state %s", received_goal.c_str(), received_state.c_str());


	if(!processMoveOrTurn(MOVE, received_goal, received_state))
	{
		ROS_ERROR("Error occurred executing processGoToStartPosition MOVE.");
		return false;
	}

	else if(!processMoveOrTurn(TURN, received_goal, received_state))
	{
		ROS_ERROR("Error occurred executing processGoToStartPosition TURN.");
		return false;
	}


	if(!optimizeTranslation(ARM_7_LINK_REAL, ARM_7_LINK))
	{
		ROS_ERROR("Error occurred optimizing Translation.");
		return false;
	}

	if(received_goal.compare(ARM_NAME) == 0 && received_state.compare(UNCOUPLE) == 0)
	{
		ROS_WARN("2 TRY");
		if(!optimizeTranslation(ARM_7_LINK_REAL, START_POSE_ARM))
		{
			ROS_ERROR("Error occurred optimizing Translation.");
			return false;
		}
		ROS_WARN("3 TRY");
		if(!optimizeTranslation(ARM_7_LINK_REAL, START_POSE_ARM))
		{
			ROS_ERROR("Error occurred optimizing Translation.");
			return false;
		}
	}

	else if(received_goal.compare(ARM_NAME) == 0 && received_state.compare(COUPLE) == 0)
	{
		ROS_WARN("2 TRY");
		if(!optimizeTranslation(ARM_7_LINK_REAL, START_POSE_COUPLE_ARM))
		{
			ROS_ERROR("Error occurred optimizing Translation.");
			return false;
		}
		ROS_WARN("3 TRY");
		if(!optimizeTranslation(ARM_7_LINK_REAL, START_POSE_COUPLE_ARM))
		{
			ROS_ERROR("Error occurred optimizing Translation.");
			return false;
		}
	}

	else if(received_goal.compare(VAC_NAME) == 0 && received_state.compare(UNCOUPLE) == 0)
	{
		ROS_WARN("2 TRY");
		if(!optimizeTranslation(ARM_7_LINK_REAL, START_POSE_VAC))
		{
			ROS_ERROR("Error occurred optimizing Translation.");
			return false;
		}
		ROS_WARN("3 TRY");
		if(!optimizeTranslation(ARM_7_LINK_REAL, START_POSE_VAC))
		{
			ROS_ERROR("Error occurred optimizing Translation.");
			return false;
		}
	}

	else if(received_goal.compare(VAC_NAME) == 0 && received_state.compare(COUPLE) == 0)
	{
		ROS_WARN("2 TRY");
		if(!optimizeTranslation(ARM_7_LINK_REAL, START_POSE_COUPLE_VAC))
		{
			ROS_ERROR("Error occurred optimizing Translation.");
			return false;
		}
		ROS_WARN("3 TRY");
		if(!optimizeTranslation(ARM_7_LINK_REAL, START_POSE_COUPLE_VAC))
		{
			ROS_ERROR("Error occurred optimizing Translation.");
			return false;
		}
	}
	else
	{
		ROS_ERROR("Error occurred while reading parameters.");
		return false;
	}

	return true;
}

/*
 * Improves the current pose of the end effector. Moves to
 * the expected pose retrieved from the fiducial on the arm.
 * Gets the transformations from tf.
 */
/*
 * This callback function is executed each time a client request
 * comes to the server.
 *
 */
bool ToolChange::processMoveOrTurn(const std::string& action, const std::string& tool,
		const std::string& received_state)
{
	ROS_INFO("Execute MoveOrTurn:  %s  with tool %s and state %s", action.c_str(), tool.c_str(), received_state.c_str());
	move_action_state_ = false;
	geometry_msgs::PoseStamped ee_pose, goal_pose;
	tf::Transform ee_pose_tf = tf::Transform::getIdentity();
	tf::Transform goal_pose_tf = tf::Transform::getIdentity();
	tf::StampedTransform reference_offset_couple_vac;
	reference_offset_couple_vac.setIdentity();
	tf::StampedTransform reference_offset_vac;
	reference_offset_vac.setIdentity();
	tf::StampedTransform st_START_POINT_VAC;
	st_START_POINT_VAC.setIdentity();
	tf::StampedTransform reference_offset_arm;
	reference_offset_arm.setIdentity();
	tf::StampedTransform st_BA_FB_ARM;
	st_BA_FB_ARM.setIdentity();
	tf::StampedTransform st_BA_FB_VAC;
	st_BA_FB_VAC.setIdentity();
	tf::StampedTransform reference_offset_couple_arm;
	reference_offset_couple_arm.setIdentity();
	tf::StampedTransform st_BA_FB_COUPLE_ARM;
	st_BA_FB_COUPLE_ARM.setIdentity();
	tf::StampedTransform st_BA_FB_COUPLE_VAC;
	st_BA_FB_COUPLE_VAC.setIdentity();

	//tf::Quaternion quat = tf::createIdentityQuaternion();
	//double rall, pitch, yaw;

	moveit::planning_interface::MoveGroup group(PLANNING_GROUP_NAME);
	goal_pose.header.frame_id = BASE_LINK;
	goal_pose.header.stamp = ros::Time::now();
	group.setPoseReferenceFrame(BASE_LINK);
	ros::Time time = goal_pose.header.stamp;

	//get the position of the end effector (= arm_7_joint)
	ee_pose.pose = group.getCurrentPose(EE_NAME).pose;
	tf::poseMsgToTF(ee_pose.pose, ee_pose_tf);

	ros::Time now = ros::Time::now();
	//goal_pose.pose = ee_pose.pose;
	//get all the relevant poses
	try{

		transform_listener_.waitForTransform(ARM_7_LINK_REAL, START_POSE_ARM, now, ros::Duration(3.0));
		transform_listener_.lookupTransform(ARM_7_LINK_REAL, START_POSE_ARM,
				time, reference_offset_arm);

		transform_listener_.waitForTransform(BASE, START_POSE_ARM, now, ros::Duration(3.0));
		transform_listener_.lookupTransform(BASE, START_POSE_ARM,
				time, st_BA_FB_ARM);

		transform_listener_.waitForTransform(ARM_7_LINK_REAL, START_POSE_VAC, now, ros::Duration(3.0));
		transform_listener_.lookupTransform(ARM_7_LINK_REAL, START_POSE_VAC,
				time, reference_offset_vac);

		transform_listener_.waitForTransform(BASE, START_POSE_VAC, now, ros::Duration(3.0));
		transform_listener_.lookupTransform(BASE, START_POSE_VAC,
				time, st_BA_FB_VAC);

		transform_listener_.waitForTransform(ARM_7_LINK_REAL, START_POSE_COUPLE_ARM, now, ros::Duration(3.0));
		transform_listener_.lookupTransform(ARM_7_LINK_REAL, START_POSE_COUPLE_ARM,
				time, reference_offset_couple_arm);

		transform_listener_.waitForTransform(BASE, START_POSE_COUPLE_ARM, now, ros::Duration(3.0));
		transform_listener_.lookupTransform(BASE, START_POSE_COUPLE_ARM,
				time, st_BA_FB_COUPLE_ARM);

		transform_listener_.waitForTransform(ARM_7_LINK_REAL, START_POSE_COUPLE_VAC, now, ros::Duration(3.0));
		transform_listener_.lookupTransform(ARM_7_LINK_REAL, START_POSE_COUPLE_VAC,
				time, reference_offset_couple_vac);

		transform_listener_.waitForTransform(BASE, START_POSE_COUPLE_VAC, now, ros::Duration(3.0));
		transform_listener_.lookupTransform(BASE, START_POSE_COUPLE_VAC,
				time, st_BA_FB_COUPLE_VAC);

		ROS_INFO("Transform exists");
	}
	catch (tf::TransformException ex)
	{
		ROS_ERROR("Transform unavailable %s", ex.what());
		return false;
	}


	//Set the goal for the arm
	if(tool.compare(ARM_NAME) == 0 && received_state.compare(UNCOUPLE)== 0)
	{
		ROS_INFO("Execute  %s for %s and state %s", action.c_str(), ARM_NAME.c_str(), received_state.c_str());
		goal_pose_tf = executeGoToStartSession(action, reference_offset_arm, st_BA_FB_ARM);

	}

	else if(tool.compare(ARM_NAME) == 0 && received_state.compare(COUPLE) == 0)
	{
		ROS_INFO("Execute  %s for %s and state %s", action.c_str(), ARM_NAME.c_str(), received_state.c_str());
		goal_pose_tf = executeGoToStartSession(action, reference_offset_couple_arm, st_BA_FB_COUPLE_ARM);

	}

	//Set the goal for the vac
	else if (tool.compare(VAC_NAME) == 0 && received_state.compare(UNCOUPLE) == 0)
	{
		ROS_INFO("Execute  %s for %s", action.c_str(), VAC_NAME.c_str());
		goal_pose_tf = executeGoToStartSession(action, reference_offset_vac, st_BA_FB_VAC);
	}

	else if (tool.compare(VAC_NAME) == 0 && received_state.compare(COUPLE) == 0)
	{
		ROS_INFO("Execute  %s for %s", action.c_str(), VAC_NAME.c_str());
		goal_pose_tf = executeGoToStartSession(action, reference_offset_couple_vac, st_BA_FB_COUPLE_VAC);
	}

	else
	{
		ROS_ERROR("Error occurred while reading goal parameters.");
		return false;
	}

	//tf -> msg
	tf::poseTFToMsg(goal_pose_tf, goal_pose.pose);
	group.setPoseTarget(goal_pose, EE_NAME);

	// plan the motion
	bool have_plan = false;
	moveit::planning_interface::MoveGroup::Plan plan;
	have_plan = group.plan(plan);

	//EXECUTE THE PLAN !!!!!! BE CAREFUL
	if (have_plan==true)
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

tf::Transform ToolChange::executeGoToStartSession(const std::string& action,const tf::StampedTransform& reference,
		const tf::StampedTransform& goal_transformation)
{
	tf::Quaternion quat = tf::createIdentityQuaternion();
	double rall, pitch, yaw;
	tf::Transform goal_pose_tf = tf::Transform::getIdentity();

	// just move without rotation
	if(action.compare(MOVE)== 0)
	{
		goal_pose_tf.setOrigin(goal_transformation.getOrigin());
		goal_pose_tf.setRotation(goal_transformation.getRotation());
	}

	// just rotate
	if(action.compare(TURN)== 0 )
	{

		tf::Matrix3x3 m2(reference.getRotation());
		m2.getRPY(rall, pitch, yaw);
		ROS_INFO(" Optimize angles %f, %f, %f ",(float)rall, (float)pitch, (float)yaw);
		quat.setRPY( rall, pitch, yaw);

		goal_pose_tf.setOrigin(goal_transformation.getOrigin());
		goal_pose_tf.setRotation(goal_transformation.getRotation() * quat);
	}

	return goal_pose_tf;
}

/*
 * Callback funktion.
 */
void ToolChange::goToSlot(const autopnp_tool_change::ToolChangeGoalConstPtr& goal)
{
	ROS_INFO(":GoToSlot received new goal state  %s for a tool %s", goal->state.c_str(), goal->goal.c_str());

	bool success = false;
	std::string state = goal->state;
	std::string tool = goal->goal;
	//= goal->tool;

	if(state.compare(COUPLE) == 0)
	{
		//move straight default
		success = processGoToSlot(tool, state);
	}
	else if(state.compare(UNCOUPLE) == 0)
	{
		// move straight with up and down
		success = processGoToSlot(tool, state);
	}
	autopnp_tool_change::ToolChangeResult result;

	if(success)
	{
		result.result = success;
		ROS_INFO("GoToSlot was successful!");
		as_go_to_slot_->setSucceeded(result);
	}
	else
	{
		result.result = success;
		ROS_ERROR("GoToSlot  failed!");
		as_go_to_slot_->setAborted(result);
	}
}


/*
 * Processes a straight movement
 * from the start position to the slot position of the tool before
 * couple/uncouple action.
 * */
bool ToolChange::processGoToSlot(const std::string& tool, const std::string& state)
{
	ros::Time now = ros::Time::now();
	tf::StampedTransform st_SLOT_POINT_ARM;
	st_SLOT_POINT_ARM.setIdentity();
	tf::StampedTransform st_SLOT_POINT_VAC;
	st_SLOT_POINT_VAC.setIdentity();
	tf::StampedTransform st_SLOT_POINT_COUPLE_ARM;
	st_SLOT_POINT_COUPLE_ARM.setIdentity();
	tf::StampedTransform st_SLOT_POINT_COUPLE_VAC;
	st_SLOT_POINT_COUPLE_VAC.setIdentity();
	std::string slot_pose_down = "default";

	try{

		transform_listener_.waitForTransform(ARM_7_LINK_REAL, SLOT_POSE_ARM, now, ros::Duration(3.0));
		transform_listener_.lookupTransform(ARM_7_LINK_REAL, SLOT_POSE_ARM,
				now, st_SLOT_POINT_ARM);

		transform_listener_.waitForTransform(ARM_7_LINK_REAL, SLOT_POSE_VAC, now, ros::Duration(3.0));
		transform_listener_.lookupTransform(ARM_7_LINK_REAL, SLOT_POSE_VAC,
				now, st_SLOT_POINT_VAC);

		transform_listener_.waitForTransform(ARM_7_LINK_REAL, SLOT_POSE_COUPLE_ARM, now, ros::Duration(3.0));
		transform_listener_.lookupTransform(ARM_7_LINK_REAL, SLOT_POSE_COUPLE_ARM,
				now, st_SLOT_POINT_COUPLE_ARM);

		transform_listener_.waitForTransform(ARM_7_LINK_REAL, SLOT_POSE_COUPLE_VAC, now, ros::Duration(3.0));
		transform_listener_.lookupTransform(ARM_7_LINK_REAL, SLOT_POSE_COUPLE_VAC,
				now, st_SLOT_POINT_COUPLE_VAC);

		ROS_INFO("Transform exists");
	}
	catch (tf::TransformException ex)
	{
		ROS_ERROR("Transform unavailable %s", ex.what());
		return false;
	}

	if(tool.compare(ARM_NAME) == 0 && state.compare(UNCOUPLE) == 0)
	{

		if(!executeGoToSlotSession(ARM_NAME, st_SLOT_POINT_ARM, ARM_7_LINK_REAL, SLOT_POSE_ARM))
		{
			ROS_ERROR("Error occurred executing session");
			return false;
		}
		slot_pose_down = SLOT_POSE_DOWN_ARM;
	}

	else if(tool.compare(VAC_NAME) == 0 && state.compare(UNCOUPLE) == 0)
	{
		if(!executeGoToSlotSession(VAC_NAME, st_SLOT_POINT_VAC, ARM_7_LINK_REAL, SLOT_POSE_VAC))
		{
			ROS_ERROR("Error occurred executing session");
			return false;
		}
		slot_pose_down = SLOT_POSE_DOWN_VAC;
	}


	else if(tool.compare(ARM_NAME) == 0 && state.compare(COUPLE) == 0)
	{

		if(!executeGoToSlotSession(ARM_NAME, st_SLOT_POINT_COUPLE_ARM, ARM_7_LINK_REAL, SLOT_POSE_COUPLE_ARM))
		{
			ROS_ERROR("Error occurred executing session");
			return false;
		}

		slot_pose_down = SLOT_POSE_DOWN_ARM;
	}

	else if(tool.compare(VAC_NAME) == 0 && state.compare(COUPLE) == 0)
	{
		if(!executeGoToSlotSession(VAC_NAME, st_SLOT_POINT_COUPLE_VAC, ARM_7_LINK_REAL, SLOT_POSE_COUPLE_VAC))
		{
			ROS_ERROR("Error occurred executing session");
			return false;
		}
		slot_pose_down = SLOT_POSE_DOWN_VAC;
	}

	else
	{
		ROS_ERROR("Error occured while reading parameters for goToSlot.");
		return false;
	}

	if(slot_pose_down.compare("default") == 0)
	{
		ROS_WARN("Slot pose not set.");
		return false;
	}

	if(!executeTranslationZ(ARM_7_LINK_REAL, slot_pose_down))
	{
		ROS_ERROR("Couldn't optimize z position.");
		return false;
	}

	return true;
}

bool ToolChange::executeGoToSlotSession(const std::string& tool_name, const tf::StampedTransform& transformation,
		const std::string& source_frame, const std::string& target_frame)
{
	tf::Vector3 translationX = tf::Vector3(0.0, 0.0, 0.0);
	tf::Quaternion angle = tf::createIdentityQuaternion();
	double rall, pitch, yaw;

	//set data
	tf::Matrix3x3 m(transformation.getRotation());
	m.getRPY(rall, pitch, yaw);
	ROS_INFO("RPY by %s slot pose %f, %f, %f ",tool_name.c_str(), (float)rall,(float)pitch,(float)yaw);
	//quat.setRPY(rall, pitch, yaw);
	translationX.setX(transformation.getOrigin().getX());
	angle.setRPY(0.0, 0.0, yaw );

	//move forward
	if(!executeStraightMoveCommand(translationX, MAX_STEP_MIL))
	{
		ROS_ERROR("Error occurred executing processGoToSlotAndTuren straight movement");
		return false;
	}
	ROS_INFO("Executed translation along x axis to %f for the %s", translationX.getX(), tool_name.c_str());

	//turn
	if(!executeTurn(angle))
	{
		ROS_ERROR("Error occurred executing turn");
		return false;
	}

	ROS_INFO("Turned around z axes, angle %f", yaw);

	for(int i = 0; i < 3; i++)
	{
		//optimize translation
		if(!optimizeTranslation(source_frame, target_frame))
		{
			ROS_ERROR("Couldn't optimize the position.");
			return false;
		}

	}

	return true;
}


bool ToolChange::optimizeTranslation(const std::string& source_frame, const std::string& target_frame)
{
	ros::Time now = ros::Time::now();
	tf::StampedTransform offset_st;
	std::string err;
	offset_st.setIdentity();

	try
	{
		transform_listener_.waitForTransform(source_frame, target_frame, now, ros::Duration(3.0));
		//transform_listener_.getLatestCommonTime(source_frame, target_frame, now, &err );
		transform_listener_.lookupTransform(source_frame, target_frame, now , offset_st);

		ROS_INFO("Transform exists");
	}
	catch (tf::TransformException ex)
	{
		ROS_ERROR(" Transform unavailable %s", ex.what());
		return false;
	}

	ROS_WARN("BE CAREFUL ! OPTOMIZING 3 AXES %f, %f, %f ",
			offset_st.getOrigin().getX(), offset_st.getOrigin().getY(), offset_st.getOrigin().getZ());

	/*
		tf::Vector3	movement = tf::Vector3(0.0, 0.0, z);
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

		movement = tf::Vector3(x, 0.0, 0.0);
		if(!executeStraightMoveCommand(movement, MAX_STEP_CM))
		{
			ROS_ERROR("Error occurred executing processGoToStartPosition.");
			return false;
		}
	 */
	double x = offset_st.getOrigin().getX();
	double y = offset_st.getOrigin().getY();
	double z = offset_st.getOrigin().getZ();

	double path_length = offset_st.getOrigin().length();

	ROS_INFO("path length %f", path_length);
	/*
		 if(path_length > 0.01)
		 {
          step = MAX_STEP_MIL;
		 }
		 else if(path_length > 0.1)
		 {
			 step = MAX_STEP_CM;
		 }
	 */
	tf::Vector3 movement = tf::Vector3(x, y, z);

	if(!executeStraightMoveCommand(movement, MAX_STEP_MMIL))
	{
		ROS_ERROR("Error occurred executing straight command.");
		return false;
	}


	return true;
}

bool ToolChange::executeTranslationZ(const std::string& source_frame, const std::string& target_frame)
{
	ros::Time now = ros::Time::now();
	tf::StampedTransform offset_st;
	std::string err;
	offset_st.setIdentity();

	try
	{
		transform_listener_.getLatestCommonTime(source_frame, target_frame, now, &err );
		transform_listener_.lookupTransform(source_frame, target_frame, now , offset_st);

		ROS_INFO("Transform exists");
	}
	catch (tf::TransformException ex)
	{
		ROS_ERROR(" Transform unavailable %s", ex.what());
		return false;
	}

	ROS_WARN("BE CAREFUL ! OPTOMIZING Z ACHES %f ", offset_st.getOrigin().getZ());

	double z = offset_st.getOrigin().getZ();

	ROS_INFO("path length in z aches is %f", z);

	tf::Vector3 movement = tf::Vector3(0, 0, z);

	if(!executeStraightMoveCommand(movement, MAX_STEP_MMIL))
	{
		ROS_ERROR("Error occurred executing straight command.");
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

/*
 * Execute a planning action with moveIt interface utilities.
 * The reference frame, the "base_link" frame,
 * initializes the starting point of the coordinate system.
 * the goal frame describes the end effector ("arm_7_link")
 * which will be moved to a new position. The {@ goal_pose}
 * is relative to the end effector frame.
 * Returns true, if the planned action has been executed.
 */
bool ToolChange::executeTurn(const tf::Quaternion& quad)
{
	ROS_INFO("Start execute move command with the goal.");

	move_action_state_ = false;

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
	//printPose(ee_pose_tf);


	current_tf.setOrigin(ee_pose_tf.getOrigin());
	current_tf.setRotation(ee_pose_tf.getRotation() * quad);

	tf::poseTFToMsg(current_tf, pose.pose);
	//printPose(current_tf);

	group.setPoseTarget(pose, EE_NAME);

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
	double result = frac * 100;
	ROS_INFO(" Fraction is %i " , (int) result);

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
void ToolChange::goBackToStart(const autopnp_tool_change::ToolChangeGoalConstPtr& goal)
{
	ROS_INFO("GoBackStartPosition received for tool  %s und with state %s ",
			goal->goal.c_str(), goal->state.c_str());

	std::string tool = goal->goal;
	std::string state = goal->state;
	bool success = false;

	if(state.compare(UP_AND_MOVE) == 0)
	{
		success = processGoBackNormal(tool);
	}
	else if(state.compare(LIFT_AND_BACK) == 0)
	{
		success = processGoBackLift(tool);
	}
	autopnp_tool_change::ToolChangeResult result;

	//set response
	if(success)
	{
		result.result = success;
		ROS_INFO("GoBackToStart was successful %i !", (int) result.result);
		as_go_back_to_start_->setSucceeded(result);
	}
	else
	{
		result.result = success;
		ROS_ERROR("GoBackToStart failed !");
		as_go_back_to_start_->setAborted(result);
	}
}

bool ToolChange::processGoBackLift(const std::string& tool)
{
	if(tool.compare(ARM_NAME) == 0)
	{
		if(!executeTranslationZ(ARM_7_LINK_REAL, SLOT_POSE_ARM))
		{
			return false;
		}

	}
	else if(tool.compare(VAC_NAME) == 0)
	{
		if(!executeTranslationZ(ARM_7_LINK_REAL, SLOT_POSE_VAC))
		{
			return false;
		}
	}
	else
	{
		ROS_ERROR("Error occurred while reading goal parameters for goBackToStart.");
		return false;
	}
	/*
	tf::Vector3 translateUp = tf::Vector3(0.0, 0.0, -0.005);
	if(!executeStraightMoveCommand(translateUp, MAX_STEP_MMIL))
	{
		ROS_ERROR("Error occurred executing processGoToSlotAndTuren straight movement");
		return false;
	}
	 */
	tf::Quaternion angle = tf::createIdentityQuaternion();
	angle.setRPY(0.0, 0.0, -TOOL_CHANGER_OFFSET_ANGLE);

	if(!executeTurn(angle))
	{
		ROS_ERROR("Error occurred executing processGoToSlotAndTuren turn");
		return false;
	}

	tf::Vector3 translate = tf::Vector3(0.11, 0.0, 0.0);
	if(!executeStraightMoveCommand(translate, MAX_STEP_MMIL))
	{
		ROS_ERROR("Error occurred executing processGoToSlotAndTuren straight movement");
		return false;
	}

	return true;
}

bool ToolChange::processGoBackNormal(const std::string& tool)
{

	tf::Vector3 translateUp = tf::Vector3(0.0, 0.0, -0.03);
	if(!executeStraightMoveCommand(translateUp, MAX_STEP_MIL))
	{
		ROS_ERROR("Error occurred executing processGoToSlotAndTuren straight movement");
		return false;
	}

	tf::Vector3 translateBack = tf::Vector3(0.11, 0.0, 0.0);
	if(!executeStraightMoveCommand(translateBack, MAX_STEP_CM))
	{
		ROS_ERROR("Error occurred executing processGoToSlotAndTuren straight movement");
		return false;
	}

	return true;
}


void ToolChange::waitForMoveit()
{
	while(!move_action_state_)
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

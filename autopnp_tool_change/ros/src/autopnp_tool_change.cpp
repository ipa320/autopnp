#include <autopnp_tool_change/autopnp_tool_change.h>
#include <math.h>
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

/*
 /base_link /fiducial/tag_board
At time 1403623048.817
- Translation: [-0.771, -0.143, 1.002]
- Rotation: in Quaternion [0.460, 0.540, 0.526, 0.469]
            in RPY [1.578, 0.023, 1.707]
At time 1403623049.721
- Translation: [-0.771, -0.143, 1.002]
- Rotation: in Quaternion [0.460, 0.540, 0.526, 0.469]
            in RPY [1.578, 0.023, 1.708]
At time 1403623050.725
- Translation: [-0.771, -0.143, 1.002]
- Rotation: in Quaternion [0.460, 0.540, 0.526, 0.469]
            in RPY [1.578, 0.023, 1.707]


 */
ToolChange::ToolChange(ros::NodeHandle nh)
: transform_listener_(nh)
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

	//waiting till all fiducials detected.
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
}


/*
 * Set the garbage container free  and shutdown running processes !!!!!!
 */
ToolChange::~ToolChange()
{

}
/*
//base_link /fiducial/tag_board
A- Translation: [-0.772, -0.126, 1.002]
- Rotation: in Quaternion [0.458, 0.541, 0.528, 0.469]
            in RPY [1.575, 0.024, 1.713]
At time 1403626759.448
- Translation: [-0.772, -0.126, 1.002]
- Rotation: in Quaternion [0.457, 0.541, 0.528, 0.468]
            in RPY [1.575, 0.024, 1.715]
At time 1403626760.448
- Translation: [-0.772, -0.126, 1.002]
- Rotation: in Quaternion [0.457, 0.541, 0.528, 0.469]
            in RPY [1.574, 0.024, 1.714]
^CAt time 1403626760.757
- Translation: [-0.772, -0.126, 1.002]
- Rotation: in Quaternion [0.457, 0.541, 0.528, 0.469]
            in RPY [1.574, 0.024, 1.714]

 */
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
			ROS_WARN("Not all fiducials are detected.");
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
	static tf::TransformBroadcaster br;

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
			std::string fidu_name_arm = "/fiducial/"+ input_marker_detections_msg->detections[i].label;
			detected_arm_fiducial = true;
			result.arm_.translation.setOrigin(translation);
			result.arm_.translation.setRotation(orientation);
			result.arm_.translation.getRotation().normalize();

			//broadcast pose to tf
			static tf::TransformBroadcaster br;
			br.sendTransform(tf::StampedTransform(result.arm_.translation, input_marker_detections_msg->header.stamp,
					input_marker_detections_msg->header.frame_id, fidu_name_arm ));
			latest_time_ = input_marker_detections_msg->header.stamp;
		}
	}
	if(count != 0)
	{
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

		std::string fiducial_reference_in = "/fiducial/reference_in";
		tf::Transform fidu_reference_in(tf::Quaternion(0.501, -0.451, 0.448, 0.587),
				tf::Vector3(-0.081, -0.078, 0.112));

		//broadcast pose to tf

		br.sendTransform(tf::StampedTransform(result.board_.translation, input_marker_detections_msg->header.stamp,
				input_marker_detections_msg->header.frame_id, fidu_name_board ));

		ros::spinOnce();

		br.sendTransform(tf::StampedTransform(fidu_board_translated, input_marker_detections_msg->header.stamp,
				fidu_name_board, tool_change_start_pose ));
		latest_time_ = input_marker_detections_msg->header.stamp;


		br.sendTransform(tf::StampedTransform(fidu_reference_translated, latest_time_ ,
				"/fiducial/tag_board", fiducial_reference));

		br.sendTransform(tf::StampedTransform(fidu_reference_in, latest_time_ ,
				"/fiducial/tag_board", fiducial_reference_in ));
	}
	detected_all_fiducials_ = detected_arm_fiducial && detected_board_fiducial;

	if(detected_all_fiducials_)
	{
		tf::StampedTransform stamped_transform_FA_EE;
		stamped_transform_FA_EE.setOrigin(FA_EE_OFFSET);
		stamped_transform_FA_EE.setRotation(FA_EE_ORIENTATION_OFFSET);

		tf::StampedTransform stamped_transform_CAM_EE_FB;
		tf::StampedTransform stamped_transform_BA_FA;
		tf::StampedTransform stamped_transform_FA_START_POINT;
		try{

			br.sendTransform(tf::StampedTransform(stamped_transform_FA_EE, latest_time_,
					"/fiducial/tag_2", "/arm_7_link_real"));


			transform_listener_.lookupTransform( "/fiducial/tag_2", "/fiducial/start_point",
					stamped_transform_FA_EE.stamp_, stamped_transform_FA_START_POINT);

		}
		catch (tf::TransformException ex)
		{
			ROS_WARN("Transform unavailable %s", ex.what());
		}
	}

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
	transform_CA_FA.setRotation(arm_pose.getRotation() );
	transform_CA_FB.setRotation(board_pose.getRotation());

	//calculate the transformation between arm and board
	result.mult(transform_CA_FA.inverse() ,transform_CA_FB);

	return result;

}

/*
 * This callback function is executed each time a request (= goal message)
 * comes to go_to_start_position server.
 * ==============================================
 *     ARM        ||  VAC_CLEANER    ||   X    ||
 * ==============================================
 *    start(arm)        start(vac)
 * ==============================================
 *                     GOAL_FIDUCIAL
 *                         |
 *                         |
 *                    move & correct error
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
	std::string received_goal = goal->goal;
	while(detected_all_fiducials_ == false)
	{
		ROS_WARN("No fiducials detected. Spinning and waiting.");
		ros::spinOnce();
	}
	//move to a start position
	success = processGoToStartPosition(received_goal);

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
bool ToolChange::processGoToStartPosition(const std::string& received_goal)
{
	/*
	//rotate
	ROS_INFO("GoToStartPosition process GO");
	if(!moveToWagonFiducial(TURN))
	{
		ROS_WARN("Error occurred executing turn to wagon fiducial position.");
		return false;
	}
	 */
	//move to position
	ROS_INFO("GoToStartPosition process starts now");
	//	for(int i = 0; i < 1; i++)
	//	{

	if(!moveToWagonFiducial(MOVE))
	{

		ROS_WARN("Error occurred executing move to wagon fiducial position.");

		return false;
	}
	//	}
	for(int i = 0; i < 2; i++)
	{
		if(!moveToWagonFiducial(TURN))
		{

			ROS_WARN("Error occurred executing turn to wagon fiducial position.");

			return false;
		}
	}
	static tf::TransformBroadcaster br;
	tf::StampedTransform goal;
	tf::StampedTransform offset_st;
	geometry_msgs::PoseStamped goal_pose;


	try{
		transform_listener_.lookupTransform( "/arm_7_link","/arm_7_link_real",
				latest_time_, offset_st);
		transform_listener_.lookupTransform( "/base_link","/arm_7_link_real",
				latest_time_, goal);
	}
	catch (tf::TransformException ex)
	{
		ROS_WARN("Transform unavailable %s", ex.what());
	}

	printPose(offset_st);

	double x = offset_st.getOrigin().getX();
	double y = offset_st.getOrigin().getY();
	double z = offset_st.getOrigin().getZ();

	//- Translation: [-0.018, -0.027, 0.026]

	tf::Vector3 movement = tf::Vector3(x, 0.0, 0.0);
	if(!executeStraightMoveCommand(movement, MAX_STEP_CM))
	{
		ROS_WARN("Error occurred executing move to wagon fiducial.");
		return false;
	}

	movement = tf::Vector3(0.0,y, 0.0);
	if(!executeStraightMoveCommand(movement, MAX_STEP_CM))
	{
		ROS_WARN("Error occurred executing move to wagon fiducial.");
		return false;
	}
	movement = tf::Vector3(0.0, 0.0, z);
	if(!executeStraightMoveCommand(movement, MAX_STEP_CM))
	{
		ROS_WARN("Error occurred executing move to wagon fiducial.");
		return false;
	}
	/*
	 * /base_link /fiducial/tag_board
	 *- Translation: [-0.772, -0.127, 1.002]
- Rotation: in Quaternion [0.461, 0.539, 0.526, 0.469]
            in RPY [1.577, 0.021, 1.705]
At time 1403627376.113
- Translation: [-0.772, -0.127, 1.002]
- Rotation: in Quaternion [0.462, 0.539, 0.525, 0.469]
            in RPY [1.579, 0.021, 1.704]
At time 1403627377.121
- Translation: [-0.772, -0.127, 1.002]
- Rotation: in Quaternion [0.461, 0.539, 0.526, 0.469]
            in RPY [1.578, 0.021, 1.706]
^CAt time 1403627378.025
- Translation: [-0.772, -0.127, 1.002]
- Rotation: in Quaternion [0.462, 0.539, 0.526, 0.469]
            in RPY [1.578, 0.021, 1.705]

	 */
	/*

	movement = tf::Vector3(-0.085, 0.0, 0.0);
		if(!executeStraightMoveCommand(movement, MAX_STEP_CM))
		{
			ROS_WARN("Error occurred executing move to wagon fiducial.");
			return false;
		}
	 */
	/*
    tf::poseTFToMsg(goal, goal_pose.pose);


	if(!executeMoveCommand(goal_pose))
		{
			ROS_WARN("Error occurred executing move.");
			return false;
		}
	 */
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
	if(tool_name.compare(VAC_NAME) == 0)
	{
		//move to start position in front of the vacuum cleaner
		//success = processGoToSlotAndTurn(up, forward, down);
	}
	else if(tool_name.compare(ARM_NAME) == 0)
	{
		// move to start position in front of the arm slot
		//success = processGoToSlotAndTurn(default);
	}
	autopnp_tool_change::GoToStartPositionResult result;
	std::string feedback;

	//set the response
	if(success)
	{
		ROS_INFO("GoToSlotAndTurn was successful!");
		//result.result = true;
		feedback ="ARM ON SLOT POSITION !!";
		as_go_to_slot_and_turn_->setSucceeded(result, feedback);
	}
	else
	{
		ROS_ERROR("GoToSlotAndTurn  failed!");
		//result.result = true;
		feedback ="FAILD TO GET TO SLOT POSITION !!";
		as_go_to_slot_and_turn_->setAborted(result, feedback);
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
bool ToolChange::processGoToSlotAndTurn(const tf::Vector3& movement1, const tf::Vector3& movement2, const tf::Vector3& movement3)
{
	tf::Vector3 move1,move2,move3;
	move1 = movement1;
	move2 = movement2;
	move3 = movement3;

	if(!move1.isZero())
	{
		if(!executeStraightMoveCommand(move1, MAX_STEP_CM))
		{
			return false;
		}
	}

	if(!move2.isZero())
	{
		if(!executeStraightMoveCommand(move2, MAX_STEP_CM))
		{
			return false;
		}
	}

	if(!move3.isZero())
	{
		if(!executeStraightMoveCommand(move3, MAX_STEP_CM))
		{
			return false;
		}
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
 * The Transformation of FA_EE (end effector and fiducial arm) is a
 * fixed transformation, depending on where and how far
 * the fiducial is placed/sticked on the arm.
 *
 * The Transformation BA_FB is fixed and improved through
 * distance offset (between wagon and start position, cm)
 * and rotation (between end effector and board fiducial)
 *
 */

bool ToolChange::moveToWagonFiducial(const std::string& action)
{
	ROS_INFO("Execute %s ", action.c_str());
	move_action_state_ = false;
	geometry_msgs::PoseStamped ee_pose;
	geometry_msgs::PoseStamped goal_pose;
	tf::StampedTransform st_BA_FB;
	tf::Transform ee_pose_tf;
	tf::Transform goal_pose_tf;
	tf::StampedTransform st_START_POINT;
	tf::StampedTransform tag2_to_ref;

	moveit::planning_interface::MoveGroup group(PLANNING_GROUP_NAME);
	goal_pose.header.frame_id = BASE_LINK;
	goal_pose.header.stamp = ros::Time::now();
	group.setPoseReferenceFrame(BASE_LINK);


	//get the position of the end effector (= arm_7_joint)
	ee_pose.pose = group.getCurrentPose(EE_NAME).pose;
	current_ee_pose_.pose = group.getCurrentPose(EE_NAME).pose;
	tf::poseMsgToTF(ee_pose.pose, ee_pose_tf);

	tf::StampedTransform transform;
	transform.setIdentity();
	static tf::TransformBroadcaster br;

	//stamped_transform_FA_EE.setOrigin(FA_EE_OFFSET);
	//stamped_transform_FA_EE.setRotation(FA_EE_ORIENTATION_OFFSET);

	//br.sendTransform(tf::StampedTransform(stamped_transform_FA_EE, latest_time_,
	//"/fiducial/tag_2", "/base_camera_ee_link"));

	try{
		transform_listener_.lookupTransform( "/fiducial/tag_2","/fiducial/reference",
				latest_time_, tag2_to_ref);

		transform_listener_.lookupTransform( "/base_link", "/fiducial/start_point",
				latest_time_, st_BA_FB);
		//distance to go
		transform_listener_.lookupTransform( "/arm_7_link", "/fiducial/start_point",
				st_BA_FB.stamp_,st_START_POINT);

	}
	catch (tf::TransformException ex)
	{
		ROS_WARN("Transform unavailable %s", ex.what());
	}

	tf::Quaternion quat = tf::createIdentityQuaternion();

	// if(tag2_to_ref_diviation.getOrigin().isZero())

	//quat.setRPY(-0.126, -0.033, 0.0);
	double r, p, y;
	tf::Matrix3x3 m(tag2_to_ref.getRotation());
	m.getRPY(r, p, y);
	ROS_INFO("rpy %f, %f, %f ",(float)r,(float)p,(float)y);
	quat.setRPY( p, y, 0.0);

	//offset from wechsler to x-achse
	tf::Quaternion help_z;
	help_z.setRPY(0.0,0.0,-0.055);
	//must be more than zero
	if(!st_BA_FB.getOrigin().isZero())

		//- Translation: [-0.018, -0.027, 0.026]

		// just move without rotation
		if(action.compare(MOVE)== 0)

		{

			goal_pose_tf.setOrigin(st_BA_FB.getOrigin());
			goal_pose_tf.setRotation(st_BA_FB.getRotation()*help_z);
			//goal_pose_tf.setRotation(st_BA_FB.getRotation());

		}

	// just move without rotation
	if(action.compare(TURN)== 0)

	{
		goal_pose_tf.setOrigin(st_BA_FB.getOrigin());
		goal_pose_tf.setRotation((st_BA_FB.getRotation()* help_z)* quat);
		//goal_pose_tf.setRotation((st_BA_FB.getRotation())* quat);
	}
	/*
	 * BILDER ABWEICHEN !!!!
	 * - Translation: [-0.171, -0.036, -0.228]
- Rotation: in Quaternion [-0.161, 0.159, 0.680, 0.697]
            in RPY [-0.009, 0.457, 1.544]
	 *
	 * - Translation: [-0.171, -0.036, -0.228]
- Rotation: in Quaternion [-0.161, 0.159, 0.680, 0.697]
            in RPY [-0.009, 0.457, 1.544]

	 * rosrun tf tf_echo /fiducial/tag_2 /fiducial/tag_board
At time 1403597846.576
- Translation: [-0.157, 0.102, -0.163]
- Rotation: in Quaternion [-0.122, 0.129, 0.688, 0.704]
            in RPY [0.006, 0.358, 1.549]
	 *
	 */

	//tf -> msg
	tf::poseTFToMsg(goal_pose_tf, goal_pose.pose);

	double length = st_START_POINT.getOrigin().length();
	//double abstand = tag2_to_ref.getOrigin().length();
	ROS_WARN_STREAM(" distance to move " << length << ".");

	group.setPoseTarget(goal_pose, EE_NAME);

	//if(action.compare(MOVE)== 0 && length > MAX_TOLERANCE)
	//{
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
		move_action_state_ = false;
		return false;
	}
	//}else{
	//	ROS_WARN_STREAM("no movement occured.");
	//}

	current_ee_pose_.pose = group.getCurrentPose(EE_NAME).pose;
	move_action_state_ = true;




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
bool ToolChange::executeMoveCommand(const geometry_msgs::PoseStamped& goal_pose)
{
	ROS_INFO("Start execute move command with the goal.");

	move_action_state_ = false;

	geometry_msgs::PoseStamped pose;
	geometry_msgs::PoseStamped ee_pose;

	pose.pose = goal_pose.pose;

	moveit::planning_interface::MoveGroup group(PLANNING_GROUP_NAME);
	pose.header.frame_id = BASE_LINK;
	pose.header.stamp = ros::Time::now();
	group.setPoseReferenceFrame(BASE_LINK);

	ee_pose.pose = group.getCurrentPose(EE_NAME).pose;
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
		move_action_state_ = false;
		return false;
	}

	current_ee_pose_.pose = group.getCurrentPose(EE_NAME).pose;
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

	move_action_state_ = false;
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
	executeMoveCommand(pose);

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

	current_ee_pose_.pose = group.getCurrentPose(EE_NAME).pose;
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

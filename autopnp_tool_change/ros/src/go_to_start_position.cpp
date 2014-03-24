/*
 * go_to_start_position.cpp
 *
 *  Created on: Mar 18, 2014
 *      Author: rmb-om
 */
// ROS includes
#include <ros/ros.h>
//#include <ros/package.h>
#include <tf/tf.h>
#include <autopnp_tool_change/GoToStartPositionAction.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>

// ROS message includes
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/JointState.h>
#include <cob_object_detection_msgs/DetectionArray.h>
#include <cob_object_detection_msgs/Detection.h>

#include <message_filters/subscriber.h>
#include <moveit/move_group_interface/move_group.h>
// boost
#include <boost/bind.hpp>

// opencv
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/JointState.h>
#include <pr2_controllers_msgs/JointTrajectoryControllerState.h>
#include <moveit_msgs/ExecuteKnownTrajectory.h>
#include <std_msgs/ColorRGBA.h>

#include <vector>


///Static constant variables
static const std::string ARM = "tag_2";
static const std::string VAC_CLEANER = "tag_79";
static const std::string ARM_STATION = "tag_38";
static const std::string EXTRA_FIDUCIAL = "tag_73";



static const double MAX_STEP_MIL = 0.001;

static const std::string PLANNING_GROUP_NAME = "arm";
static const std::string BASE_LINK = "base_link";
static const std::string EE_NAME = "arm_7_link";

static const tf::Vector3 LEFT = tf::Vector3(0.0, 0.0, 0.0);

static const tf::Vector3 ARM_FIDUCIAL_OFFSET = tf::Vector3(0.05, 0.025, 0.03);
static const tf::Quaternion ARM_FIDUCIAL_ORIENTATION_OFFSET = tf::Quaternion(0.653883, 0.225928, 0.668417, -0.273153);
static const tf::Vector3 TOOL_FIDUCIAL_OFFSET = tf::Vector3(0.30, 0.0, 0.0);



class GoToStartPosition
{
private:
	/// ROS node handle
	ros::NodeHandle node_handle_;
	/// array of two transform msgs
	struct fiducials;
	struct fiducials
	{
		tf::Transform translation;
	};
	/// array of two fiducial objects
	struct components;
	struct components
	{
		struct fiducials arm;
		struct fiducials board;
	};

	/// SUBSCRIBERS
	message_filters::Subscriber<cob_object_detection_msgs::DetectionArray> input_marker_detection_sub_;
	///PUBLISHERS
	ros::Publisher vis_pub_;
	/// SERVER
	actionlib::SimpleActionServer<autopnp_tool_change::GoToStartPositionAction>go_to_start_position_server;

	/// CLIENTS
	ros::ServiceClient execute_known_traj_client_ ;


	bool slot_position_detected_;
	bool move_action_;
	bool detected_all_fiducials_;

	///transformation data between the arm and the wagon slot
	tf::Transform transform_CA_FA_;
	tf::Transform transform_CA_FB_;

	geometry_msgs::PoseStamped current_ee_pose_;

public:


	GoToStartPosition(ros::NodeHandle nh) :go_to_start_position_server
	(nh, "go_to_start_position", boost::bind(&GoToStartPosition::goToStartPosition, this, _1), false)
{
		ROS_INFO("Starting go_to_start_position server.");
		node_handle_ = nh;
		input_marker_detection_sub_.unsubscribe();
		slot_position_detected_ = false;
		move_action_ = false;
		marker_id_ = 0;

		//SUBSCRIBERS
		vis_pub_ = node_handle_.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
		input_marker_detection_sub_.subscribe(node_handle_, "input_marker_detections", 1);
		input_marker_detection_sub_.registerCallback(boost::bind(&GoToStartPosition::markerInputCallback, this, _1));

		/*//sleep if the slot position not found
		while(slot_position_detected_ == false)
		{
			ros::spinOnce();
		}
		*/
}

	~GoToStartPosition()
	{
		vis_pub_.shutdown();
		input_marker_detection_sub_.unsubscribe();
	}

	void goToStartPosition(const autopnp_tool_change::GoToStartPositionGoalConstPtr& goal) {
		ROS_INFO("start !!!!!!!!!!!!!!!!!");
		autopnp_tool_change::GoToStartPositionResult res;
		res.result = processGoal(goal->goal);
		go_to_start_position_server.setSucceeded(res);
	}

	void init(){
		ROS_INFO("Init go_to_start_position server");
		go_to_start_position_server.start();
	}


	//CALLBACKS
	/**
	 * Callback for the incoming  data stream.
	 * Retrieves coming data from the input_marker_detections message
	 * and calculates the distance and orientation between the position
	 * of the arm and board setting the {@value arm_board}.
	 * Sets the variable {@value slot_position_detected} as true,
	 * if the calculations took place and saves the last transformations as globals.
	 */
	void markerInputCallback(const cob_object_detection_msgs::DetectionArray::ConstPtr& input_marker_detections_msg)
	{

		struct components result_components;
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
				arm_board = calculateArmBoardTransformation(result_components.board.translation, result_components.arm.translation);

				if(!arm_board.getOrigin().isZero())
				{
					transform_CA_FA_ = result_components.arm.translation;
					transform_CA_FB_ = result_components.board.translation;

					slot_position_detected_ = true;
				}

			} else {
				slot_position_detected_ = false;
			}
		}
	}

	/*
	 * Calculates translation and orientation distance between arm marker and wagon board.
	 * Vector orientation points from arm to board.
	 */
	tf::Transform calculateArmBoardTransformation(
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
	 * Computes mean coordinate system if multiple markers detected
	 * and saves the data as an array of two fiducial objects
	 * {@value arm, @value board} with the transform data
	 * {@value translation} respectively.
	 */
	struct components computeMarkerPose(
			const cob_object_detection_msgs::DetectionArray::ConstPtr& input_marker_detections_msg)
	{
		components result;
		unsigned int count = 0;
		detected_all_fiducials_ = false;
		bool detected_arm_fiducial = false;
		bool detected_board_fiducial = false;
		//bool detected_cam_fiducial = false;
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
		}
		if(count != 0)
		{
			result.board.translation.getOrigin() /=(double)count;
			result.board.translation.getRotation() /= (double)count;
			result.board.translation.getRotation().normalize();
		}
		detected_all_fiducials_ = detected_arm_fiducial && detected_board_fiducial;

		return result;
	}


	bool processGoal(const std::string& goal)
	{
		tf::Vector3 offset;

		if(goal.compare(VAC_CLEANER)==0)
		{
			offset = tf::Vector3(0.0, 0.0, 0.0);
		}
		if(goal.compare(ARM_STATION)==0)
		{
			offset = LEFT;
		}
		//--------------------------------------------------------------------------------------
		// move arm to a start position
		//--------------------------------------------------------------------------------------

		for(int i = 0; i< 5; i++)
		{
			if(!turnToWagonFiducial())
			{
				ROS_WARN("Error occurred executing turn to wagon fiducial position.");
				return false;
			}

			if(!moveToWagonFiducial())
			{
				ROS_WARN("Error occurred executing move to wagon fiducial position.");
				return false;
			}

		}

		//--------------------------------------------------------------------------------------
		// move arm to a start position with offset
		//--------------------------------------------------------------------------------------
		if(!executeStraightMoveCommand(offset, MAX_STEP_MIL))
		{
			ROS_WARN("Error occurred executing move arm straight up.");
			return false;
		}

		return true;
	}



	bool moveToWagonFiducial()
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


		moveit::planning_interface::MoveGroup group(PLANNING_GROUP_NAME);
		goal_pose.header.frame_id = BASE_LINK;
		goal_pose.header.stamp = ros::Time::now();
		group.setPoseReferenceFrame(BASE_LINK);

		//get the position of the end effector (= arm_7_joint)
		ee_pose.pose = group.getCurrentPose(EE_NAME).pose;
		current_ee_pose_.pose = group.getCurrentPose(EE_NAME).pose;
		//msg -> tf
		tf::poseMsgToTF(ee_pose.pose, ee_pose_tf);
		geometry_msgs::PoseStamped cam_msg, test_msg,test1_msg, test2_msg, test3_msg,
		test4_msg, test5_msg, test6_msg, test7_msg, test8_msg, base;

		tf::poseTFToMsg(transform_CA_FA, test_msg.pose);
		tf::poseTFToMsg(transform_CA_FB, test1_msg.pose);

		transform_BA_FA.mult(ee_pose_tf, transform_EE_FA);
		tf::poseTFToMsg(transform_BA_FA, test2_msg.pose);
		//drawLine(0.20, 0.0, 0.0, 1.0, base, test2_msg);

		transform_CA_BA.mult(transform_CA_FA, transform_BA_FA.inverse());
		tf::poseTFToMsg(transform_CA_BA.inverse(), cam_msg.pose);
		//drawLine(0.55, 0.0, 0.0, 1.0, base, cam_msg);

		transform_FA_FB.mult(transform_BA_FA.inverse(), transform_CA_FB);
		transform_BA_FB.mult(transform_CA_BA.inverse(), transform_CA_FB);
		tf::poseTFToMsg( transform_BA_FB, test6_msg.pose);

		//drawLine(0.50, 0.0, 0.0, 1.0, base, test6_msg);
		//drawLine(0.0, 0.0, 0.3, 1.0, test6_msg, test2_msg);
		//drawLine(0.5, 0.5, 0.0, 1.0, cam_msg, test2_msg);
		//drawLine(0.5, 0.5, 0.0, 1.0, cam_msg, test6_msg);

		transform_EE_GO.mult(transform_BA_FB, transform_EE_FA);
		tf::poseTFToMsg( transform_EE_GO, test5_msg.pose);
		//drawLine(0.0, 0.30, 0.0, 1.0, base, test5_msg);

		transform_EE_FB.mult(ee_pose_tf.inverse(), transform_EE_GO);
		tf::poseTFToMsg( transform_EE_FB, test8_msg.pose);
		//drawLine(0.0, 0.30, 0.0, 1.0, base, test8_msg);

		tf::Transform transform_EE_GO_schort;
		transform_EE_GO_schort.mult(ee_pose_tf.inverse(), transform_EE_FB);
		transform_EE_GO_schort.setOrigin(transform_EE_FB.getOrigin() + TOOL_FIDUCIAL_OFFSET);
		goal_pose_tf.mult( ee_pose_tf , transform_EE_GO_schort);

		goal_pose_tf.setRotation(ee_pose_tf.getRotation());
		goal_pose_tf.setOrigin(goal_pose_tf.getOrigin());

		//tf -> msg
		tf::poseTFToMsg(goal_pose_tf, goal_pose.pose);
		drawLine(0.55, 0.55, 0.0, 1.0, ee_pose, goal_pose);
		//drawSystem(goal_pose);
		drawArrowX(0.55, 0.0, 0.0, 1.0, goal_pose);

		double length = transform_EE_GO_schort.getOrigin().length();
		ROS_WARN_STREAM(" distance :"<< length << ".");

		group.setPoseTarget(goal_pose, EE_NAME);

		if(length > 0.001)
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



	bool turnToWagonFiducial()
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
		transform_CA_FB.setOrigin(transform_CA_FB_.getOrigin());
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


		moveit::planning_interface::MoveGroup group(PLANNING_GROUP_NAME);
		goal_pose.header.frame_id = BASE_LINK;
		goal_pose.header.stamp = ros::Time::now();
		group.setPoseReferenceFrame(BASE_LINK);

		//get the position of the end effector (= arm_7_joint)
		ee_pose.pose = group.getCurrentPose(EE_NAME).pose;
		current_ee_pose_.pose = group.getCurrentPose(EE_NAME).pose;

		tf::poseMsgToTF(ee_pose.pose, ee_pose_tf);
		transform_BA_FA.mult(ee_pose_tf, transform_EE_FA);

		transform_CA_BA.mult(transform_CA_FA, transform_BA_FA.inverse());
		transform_FA_FB.mult(transform_BA_FA.inverse(), transform_CA_FB);
		transform_BA_FB.mult(transform_CA_BA.inverse(), transform_CA_FB );
		transform_EE_GO.mult(transform_BA_FB, transform_EE_FA);

		tf::Quaternion angle;
		angle.setRPY(4.0, 0.0, 0.0);
		goal_pose_tf.setOrigin(ee_pose_tf.getOrigin());
		goal_pose_tf.setRotation(transform_EE_GO.getRotation() * angle);

		tf::poseTFToMsg(goal_pose_tf, goal_pose.pose);
		drawLine(0.55, 0.55, 0.0, 1.0, ee_pose, goal_pose);
		drawArrowX(0.55, 0.0, 0.0, 1.0, goal_pose);

		transform_EE_FB.mult(ee_pose_tf.inverse(), goal_pose_tf);
		double length = transform_EE_FB.getOrigin().length();
		ROS_WARN_STREAM(" distance :"<< length << ".");

		group.setPoseTarget(goal_pose, EE_NAME);

		ROS_WARN_STREAM("STARTE TURN TO FIDUCIAL");
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
	bool executeMoveCommand(const geometry_msgs::PoseStamped& goal_pose, const double offset)
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
	bool executeStraightMoveCommand(const tf::Vector3& goal_direction, const double ee_max_step)
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
	void waitForMoveit()
	{
		while(!move_action_)
		{
			ros::spinOnce();
		}


	}

	//HELPER VARIABLES AND FUNKTIONS TO PRINT AND DRAW IN RVIZ
	geometry_msgs::PoseStamped origin;
	int marker_id_;
	/**
	 * A helper function to print the results of
	 * a transform message.
	 */
	void printPose(tf::Transform& trans_msg)
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
	 * A helper function to draw a simple arrow in rviz.
	 */
	void drawArrowX (const double r, const double g, const double b, const double a,
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
	void drawLine (const double r, const double g, const double b, const double a,
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

};



int main (int argc, char** argv)
{
	// Initialize ROS, specify name of node
	ros::init(argc, argv, "go_to_start_position_server");

	// Create a handle for this node, initialize node
	ros::NodeHandle nh;

	// Create and initialize an instance of Object
	GoToStartPosition goToStartPosition(nh);
	goToStartPosition.init();

	ros::spin();

	return (0);
}



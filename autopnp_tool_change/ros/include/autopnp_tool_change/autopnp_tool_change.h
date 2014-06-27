/*
 *autopnp_tool_change.h
 *Author: rmb-om
 */

#ifndef AUTOPNP_TOOL_CHANGE_
#define AUTOPNP_TOOL_CHANGE_

// ROS includes
#include <ros/ros.h>
//#include <ros/package.h>
#include <tf/tf.h>
#include <actionlib/client/terminal_state.h>
#include <fstream>
// ROS message includes
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/JointState.h>
#include <cob_object_detection_msgs/DetectionArray.h>
#include <cob_object_detection_msgs/Detection.h>
#include <tf/tfMessage.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <message_filters/subscriber.h>
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
#include <autopnp_tool_change/GoToStartPositionAction.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>

/**
 * Brief description:
 *
 * This program initializes and starts the server for
 * the tool change operation.
 * It subscribes to the topic /fiducials/detect_fiducials
 * to get the data from input marker detection.
 * The service callback waits till these data are processed and
 * starts the arm motion only if the transformation between arm and board fiducials
 * is found.
 *
 * Arm motion procedure:
 * - move arm to the position set with regard to
 * detected fiducials
 */

///Static constant variables
static const std::string ARM = "tag_2";
static const std::string VAC_CLEANER = "tag_79";
static const std::string ARM_STATION = "tag_38";
static const std::string EXTRA_FIDUCIAL = "tag_73";
static const std::string VAC_NAME = "vac";
static const std::string ARM_NAME = "arm";

static const std::string MOVE = "move";
static const std::string TURN = "turn";

static const std::string GO_TO_START_POSITION_ACTION_NAME = "go_to_start_position_action";
static const std::string GO_TO_SLOT_AND_TURN_ACTION_NAME = "go_to_slot_and_turn_action";
static const std::string GO_BACK_TO_START_ACTION_NAME = "go_back_to_start_action";

static const double MAX_STEP_MIL = 0.001;
static const double MAX_STEP_CM = 0.01;

static const std::string PLANNING_GROUP_NAME = "arm";
static const std::string BASE_LINK = "base_link";
static const std::string EE_NAME = "arm_7_link";


static const tf::Vector3 FA_EE_OFFSET = tf::Vector3(-0.03, -0.05, -0.09);
static const tf::Quaternion FA_EE_ORIENTATION_OFFSET = tf::Quaternion(0.655, 0.198, 0.685, 0.251);
static const double TOOL_CHANGER_OFFSET_TO_X_AXES = -0.055;

static const tf::Vector3 START_POINT_OFFSET = tf::Vector3(-0.11, -0.05, 0.23);

static const tf::Vector3 SLOT_POINT_OFFSET = tf::Vector3(-0.115, -0.145, 0.135);

class ToolChange
{

public:

	ToolChange(ros::NodeHandle nh);
	~ToolChange();
	void run();

protected:


	/// An array of two transform msgs
	struct fiducials;
	struct fiducials
	{
		tf::Transform translation;
	};

	///An array of two fiducial objects
	struct components;
	struct components
	{
		struct fiducials arm_;
		struct fiducials board_;
	};

	/// ROS node handle
	ros::NodeHandle node_handle_;

	/// SUBSCRIBERS
	ros::Subscriber input_color_camera_info_sub_;
	ros::Subscriber tf_sub;
	message_filters::Subscriber<cob_object_detection_msgs::DetectionArray> input_marker_detection_sub_;
	message_filters::Subscriber<sensor_msgs::JointState> joint_states_sub_;

	tf::TransformListener transform_listener_;
    tf::TransformBroadcaster br_;

	///PUBLISHERS
	ros::Publisher vis_pub_;

	/// SERVER
	boost::scoped_ptr<actionlib::SimpleActionServer<autopnp_tool_change::GoToStartPositionAction> > as_go_to_start_position_;
	boost::scoped_ptr<actionlib::SimpleActionServer<autopnp_tool_change::GoToStartPositionAction> > as_go_to_slot_and_turn_;
	boost::scoped_ptr<actionlib::SimpleActionServer<autopnp_tool_change::GoToStartPositionAction> > as_go_back_to_start_;

	/// CLIENTS
	ros::ServiceClient execute_known_traj_client_ ;
	ros::Time latest_time_;


	bool slot_position_detected_;
	bool move_action_state_;
	bool detected_all_fiducials_;



	///CALLBACKS
	// Callback for the incoming point cloud data stream of fiducials.
	void markerInputCallback(const cob_object_detection_msgs::DetectionArray::ConstPtr& input_marker_detections_msg);
	///SERVER CALLBACK if goal received
	void goToStartPosition(const autopnp_tool_change::GoToStartPositionGoalConstPtr& goal);
	void goToSlotAndTurn(const autopnp_tool_change::GoToStartPositionGoalConstPtr& goal);
	void goBackToStart(const autopnp_tool_change::GoToStartPositionGoalConstPtr& goal);

	/// Computes an average pose from multiple detected markers.
	void computeMarkerPose(const cob_object_detection_msgs::DetectionArray::ConstPtr& input_marker_detections_msg);

    ///MOVEMENTS
	bool executeMoveCommand(const geometry_msgs::PoseStamped& pose);
	bool executeStraightMoveCommand(const tf::Vector3& vector, const double max_step);
	bool moveToStartPosition(const std::string& action);

	///PROCESS MOVEMENTS
	bool processGoToSlotAndTurn(const tf::Vector3& movement1, const tf::Vector3& movement2, const tf::Vector3& movement3);
	bool processGoToStartPosition(const std::string& received_goal);
	bool processGoBackToStart(const std::string& received_goal);

    //small functions to split the code
	void resetServers();
	void waitForMoveit();


	//HELPER VARIABLES AND FUNKTIONS TO PRINT AND DRAW IN RVIZ
	geometry_msgs::PoseStamped origin;
	int marker_id_;
	void printPose(tf::Transform& trans_msg);
	void printMsg(const geometry_msgs::PoseStamped pose);
	void printVector(const std::vector<double> v);
};


#endif /* AUTOPNP_TOOL_CHANGE_ */

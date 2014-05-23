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
#include <autopnp_tool_change/MoveToWagonAction.h>
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

static const std::string MOVE = "move";
static const std::string TURN = "turn";

static const std::string GO_TO_START_POSITION_ACTION_NAME = "go_to_start_position_action";
static const std::string MOVE_TO_CHOSEN_TOOL_ACTION_NAME = "move_to_chosen_tool_action";

static const double MAX_STEP_MIL = 0.001;
static const double MAX_STEP_CM = 0.01;
// 1 mm
static const double MAX_TOLERANCE = 0.001;

static const std::string PLANNING_GROUP_NAME = "arm";
static const std::string BASE_LINK = "base_link";
static const std::string EE_NAME = "arm_7_link";
//static const std::string EE_NAME = "arm_toolchanger_link";

static const tf::Vector3 UP = tf::Vector3(0.0, 0.0, -0.07);
static const tf::Vector3 FORWARD = tf::Vector3(-0.12, 0.0, 0.0);
static const tf::Vector3 DOWN = tf::Vector3(0.0, 0.0, 0.04);
static const tf::Vector3 BACK = tf::Vector3(0.05, 0.0, 0.0);
static const tf::Vector3 FIDUCIAL_DISTANCE = tf::Vector3(0.0, 0.05, 0.0);

/*
 * - Translation: [-0.080, 0.020, -0.018]
- Rotation: in Quaternion [0.675, 0.171, 0.676, 0.241]
 *
 */
static const tf::Vector3 FA_EE_OFFSET = tf::Vector3(-0.080, 0.020, -0.015);
static const tf::Quaternion FA_EE_ORIENTATION_OFFSET = tf::Quaternion(0.675, 0.171, 0.676, 0.241);

static const tf::Vector3 START_POINT_OFFSET = tf::Vector3(-0.10, -0.10, 0.35);

static const tf::Vector3 VAC_CLEANER_OFFSET = tf::Vector3(0.0, 0.0, 0.0);
static const tf::Vector3 ARM_STATION_OFFSET = tf::Vector3(0.0, 0.0, 0.0);

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
		struct fiducials cam_;
	};

	/// ROS node handle
	ros::NodeHandle node_handle_;

	/// SUBSCRIBERS
	ros::Subscriber input_color_camera_info_sub_;
	ros::Subscriber tf_sub;
	message_filters::Subscriber<cob_object_detection_msgs::DetectionArray> input_marker_detection_sub_;
	message_filters::Subscriber<sensor_msgs::JointState> joint_states_sub_;

	tf::TransformListener transform_listener_;

	///PUBLISHERS
	ros::Publisher vis_pub_;

	/// SERVER
	boost::scoped_ptr<actionlib::SimpleActionServer<autopnp_tool_change::GoToStartPositionAction> > as_go_to_start_position_;
	boost::scoped_ptr<actionlib::SimpleActionServer<autopnp_tool_change::GoToStartPositionAction> > as_move_to_chosen_tool_;
	boost::scoped_ptr<actionlib::SimpleActionServer<autopnp_tool_change::GoToStartPositionAction> > as_return_to_start_position_;

	/// CLIENTS
	ros::ServiceClient execute_known_traj_client_ ;
	ros::Time latest_time_;

	bool slot_position_detected_;
	bool move_action_state_;
	bool detected_all_fiducials_;

	///transformation data between the arm and the wagon slot
	tf::Transform transform_CA_FA_;
	tf::Transform transform_CA_FB_;
	tf::Transform transform_BA_CA_;
	geometry_msgs::PoseStamped current_ee_pose_;


	///CALLBACKS
	// Callback for the incoming point cloud data stream of fiducials.
	void markerInputCallback(const cob_object_detection_msgs::DetectionArray::ConstPtr& input_marker_detections_msg);
	///SERVER CALLBACK if goal received
	void goToStartPosition(const autopnp_tool_change::GoToStartPositionGoalConstPtr& goal);
	void moveToChosenTool(const autopnp_tool_change::GoToStartPositionGoalConstPtr& goal);

	//calculates translation and orientation distance between arm marker and board marker
	tf::Transform calculateArmBoardTransformation(const tf::Transform& board, const tf::Transform& arm);
	/// Computes an average pose from multiple detected markers.
	struct components computeMarkerPose(const cob_object_detection_msgs::DetectionArray::ConstPtr& input_marker_detections_msg);

    ///MOVEMENTS
	bool executeMoveCommand(const geometry_msgs::PoseStamped& pose);
	bool executeStraightMoveCommand(const tf::Vector3& vector, const double max_step);
	bool moveToStartPosition(const geometry_msgs::PoseStamped& start_pose);
	bool moveToWagonFiducial(const std::string& action);

	///PROCESS MOVEMENTS
	bool processMoveToChosenTool(const tf::Vector3& offset);
	bool processGoToStartPosition();

    //small functions to split the code
	void resetServers();
	void waitForMoveit();
	void clearFiducials();


	//HELPER VARIABLES AND FUNKTIONS TO PRINT AND DRAW IN RVIZ
	geometry_msgs::PoseStamped origin;
	int marker_id_;
	void printPose(tf::Transform& trans_msg);
	void printMsg(const geometry_msgs::PoseStamped pose);
	void printVector(const std::vector<double> v);
	void drawArrowX( double r, double g, double b, double a, const geometry_msgs::PoseStamped& pose);
	void drawLine( double r, double g, double b, double a, const geometry_msgs::PoseStamped& pose_start,
			const geometry_msgs::PoseStamped& pose_end);
	void drawSystem(const geometry_msgs::PoseStamped& pose);
};


#endif /* AUTOPNP_TOOL_CHANGE_ */

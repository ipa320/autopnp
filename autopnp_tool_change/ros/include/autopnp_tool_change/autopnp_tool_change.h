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

// ROS message includes
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/JointState.h>
#include <cob_object_detection_msgs/DetectionArray.h>
#include <cob_object_detection_msgs/Detection.h>

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
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>

/**
 * Brief description:
 *
 *  This program initializes and starts the server for
 *  the tool change operation.
 *  It subscribes to the topic /fiducials/detect_fiducials
 *  to get the data from input marker detection.
 *  The service callback waits till these data are processed and
 *  starts the arm motion only if the transformation between arm and board fiducials
 *  is found.
 *
 *  Arm motion procedure:
 *  - move arm to initial position (is given)
 *  - move arm to the position set with regard to
 *  detected fiducials
 */

///Static constant variables
static const int COUPLE = 1;
static const int DECOUPLE = 2;
static const std::string ARM = "tag_2";
static const std::string VAC_CLEANER = "tag_79";
static const std::string ARM_STATION = "tag_38";
static const std::string EXTRA_FIDUCIAL = "tag_73";

static const double MAX_STEP_MIL = 0.001;
static const double MAX_STEP_CM = 0.01;

static const std::string PLANNING_GROUP_NAME = "arm";
static const std::string BASE_LINK = "base_link";
static const std::string EE_NAME = "arm_7_link";

static const tf::Vector3 UP = tf::Vector3(0.0, 0.0, 0.07);
static const tf::Vector3 FORWARD = tf::Vector3(-0.05, 0.0, 0.0);
static const tf::Vector3 DOWN = tf::Vector3(0.0, 0.0, -0.07);
static const tf::Vector3 BACK = tf::Vector3(0.05, 0.0, 0.0);
static const tf::Vector3 FIDUCIAL_DISTANCE = tf::Vector3(0.0, 0.05, 0.0);

static const tf::Vector3 ARM_FIDUCIAL_OFFSET = tf::Vector3(0.0, 0.0, 0.0);
static const tf::Vector3 TOOL_FIDUCIAL_OFFSET = tf::Vector3(0.0, 0.0, 0.0);
//static const tf::Vector3 TOOL_FIDUCIAL_OFFSET = tf::Vector3(0.0, 0.0, 0.0);
//static const tf::Vector3 ARM_FIDUCIAL_OFFSET = tf::Vector3(0.0, 0.0, 0.0);

class ToolChange
{

public:

	ToolChange(ros::NodeHandle nh);
	~ToolChange();
	void changeTool(const autopnp_tool_change::MoveToWagonGoalConstPtr& goal);
	void init();

protected:


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

	/// instance of a subscriber for the camera calibration
	///action of incoming color image data
	ros::Subscriber input_color_camera_info_sub_;
	/// ROS node handle
	ros::NodeHandle node_handle_;

	/// SUBSCRIBERS
	message_filters::Subscriber<cob_object_detection_msgs::DetectionArray> input_marker_detection_sub_;
	message_filters::Subscriber<sensor_msgs::JointState> joint_states_sub_;
	///PUBLISHERS
	ros::Publisher vis_pub_;
	/// SERVER
	actionlib::SimpleActionServer<autopnp_tool_change::MoveToWagonAction> change_tool_server_;
	/// CLIENTS
	 ros::ServiceClient execute_known_traj_client_ ;

	/// messages that are used to published feedback/result
	autopnp_tool_change::MoveToWagonFeedback feedback_;
	autopnp_tool_change::MoveToWagonResult result_;

	bool slot_position_detected_;
	bool move_action_;
	bool detected_both_fiducials_;

	///container for the joint msgs
	std::vector<double> jointVelocities_;
	std::vector<double> jointPositions_;

	///transformation data between the arm and the wagon slot
	//tf::Transform transform_CA_EE_;
	//tf::Transform transform_CA_GO_;
	//tf::Transform transform_CA_FA_;
	//tf::Transform transform_CA_FB_;

	//tf::Transform transform_FB_GO_;
	//tf::Transform transform_FA_EE_;
	//tf::Transform transform_CA_FA_;
	//tf::Transform transform_CA_FB_;
	//tf::Transform transform_FA_FB_;
	tf::Transform transform_CA_FA_;
	tf::Transform transform_CA_FB_;
	tf::Transform arm_transform_;
	tf::Transform arm_board_transform_;
	geometry_msgs::PoseStamped current_ee_pose_;

	//define rotations
		tf::Quaternion rotate_Y_90_right;
		tf::Quaternion rotate_Y_90_left;
		tf::Quaternion rotate_3Z_pi_4_left;
		tf::Quaternion rotate_3Z_pi_4_right;
		tf::Quaternion rotate_X_90_right;
		tf::Quaternion rotate_X_90_left;
		tf::Quaternion rotate_Z_90_left;
		tf::Quaternion rotate_Z_90_right;

	//CALLBACKS
	/// Callback for the incoming data of joints' state.
	void jointInputCallback(const sensor_msgs::JointState::ConstPtr& input_joint_msg);
	/// Callback for the incoming point cloud data stream of fiducials.
	void markerInputCallback(const cob_object_detection_msgs::DetectionArray::ConstPtr& input_marker_detections_msg);

	//calculates translation and orientation distance between arm marker and board marker
	tf::Transform calculateArmBoardTransformation(const tf::Transform& board, const tf::Transform& arm);
	/// Computes an average pose from multiple detected markers.
	struct components computeMarkerPose(const cob_object_detection_msgs::DetectionArray::ConstPtr& input_marker_detections_msg);

	///processes the goal message
	bool processGoal(const geometry_msgs::PoseStamped& pose);
	///execution of the process in question
	bool coupleOrDecouple(const int command, const geometry_msgs::PoseStamped& pose);
	bool executeMoveCommand(const geometry_msgs::PoseStamped& pose, const double offset);
	bool executeStraightMoveCommand(const tf::Vector3& vector, const double max_step);
	void moveArm();
	void waitForMoveit();
	///executes the arm movement to the set initial position
	void moveToStartPose(const geometry_msgs::PoseStamped& start_pose);
	bool moveToStartPosition(const geometry_msgs::PoseStamped& start_pose);
	bool moveToWagonFiducial(const double offset);

	//HELPER VARIABLES AND FUNKTIONS TO PRINT AND DRAW IN RVIZ
	geometry_msgs::PoseStamped origin;
	int marker_id_;
	void printPose(tf::Transform& trans_msg);
	void printMsg(const geometry_msgs::PoseStamped pose);
	void printVector(const std::vector<double> v);
	void drawArrowX( double r,  double g, double b, double a, const geometry_msgs::PoseStamped& pose);
	void drawLine( double r,  double g, double b, double a, const geometry_msgs::PoseStamped& pose_start,
			const geometry_msgs::PoseStamped& pose_end);
	void drawSystem(const geometry_msgs::PoseStamped& pose);
};


#endif /* AUTOPNP_TOOL_CHANGE_ */

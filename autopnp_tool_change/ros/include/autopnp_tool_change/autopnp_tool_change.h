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
#include <autopnp_tool_change/ToolChangeAction.h>
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
static const std::string TAG_0 = "tag_0";
static const std::string VAC_CLEANER = "tag_79";
static const std::string ARM_STATION = "tag_38";
static const std::string EXTRA_FIDUCIAL = "tag_73";

static const std::string START_POSE_ARM = "/fiducial/start_pose_arm";
static const std::string START_POSE_COUPLE_ARM = "/fiducial/start_pose_couple_arm";

static const std::string START_POSE_VAC = "/fiducial/start_pose_vac";
static const std::string START_POSE_COUPLE_VAC = "/fiducial/start_pose_couple_vac";
static const std::string SLOT_POSE_ARM = "/fiducial/slot_pose_arm";
static const std::string SLOT_POSE_DOWN_ARM = "/fiducial/slot_pose_down_arm";
static const std::string SLOT_POSE_COUPLE_ARM = "/fiducial/slot_pose_couple_arm";
static const std::string SLOT_POSE_VAC = "/fiducial/slot_pose_vac";
static const std::string SLOT_POSE_COUPLE_VAC = "/fiducial/slot_pose_couple_vac";
static const std::string SLOT_POSE_DOWN_VAC = "/fiducial/slot_pose_down_vac";
static const std::string TAG_BOARD = "fiducial/tag_board";
static const std::string TAG_ARM = "/fiducial/tag_arm";
static const std::string ARM_7_LINK_REAL = "/arm_7_link_real";
static const std::string ARM_7_LINK = "/arm_7_link";
static const std::string CAM = "/head_cam3d_link";
static const std::string BASE = "/base_link";

static const std::string DEFAULT = "default";
static const std::string UP_AND_MOVE = "upAndMove";
static const std::string LIFT_AND_BACK = "liftAndBack";
static const std::string COUPLE = "couple";
static const std::string UNCOUPLE = "uncouple";

static const std::string PLANNING_GROUP_NAME = "arm";
static const std::string BASE_LINK = "base_link";
static const std::string EE_NAME = "arm_7_link";
static const std::string MOVE = "move";
static const std::string TURN = "turn";

static const std::string ARM_NAME = "gripper";
static const std::string VAC_NAME = "vac";

static const std::string GO_TO_START_POSITION_ACTION_NAME = "go_to_start_position_action";
static const std::string GO_TO_SLOT_ACTION_NAME = "go_to_slot_action";
static const std::string GO_BACK_TO_START_ACTION_NAME = "go_back_to_start_action";

static const double MAX_STEP_MIL = 0.001;
static const double MAX_STEP_MMIL = 0.0001;
static const double MAX_STEP_CM = 0.01;

/*fidu = 0
 *
 /base_link /fiducial/tag_board
At time 1405686724.306
- Translation: [-0.688, -0.011, 1.002]
- Rotation: in Quaternion [0.479, 0.520, 0.509, 0.491]
            in RPY [1.571, 0.024, 1.629]
 /base_link /arm_7_link
At time 1405686804.878
- Translation: [-0.459, -0.066, 0.934]
- Rotation: in Quaternion [0.998, 0.003, 0.021, 0.053]
            in RPY [3.036, -0.042, 0.008]
 /base_link /head_cam3d_link
At time 1405686846.106
- Translation: [-0.005, -0.016, 1.200]
- Rotation: in Quaternion [0.562, 0.593, -0.429, -0.386]
            in RPY [-1.912, 0.024, 1.641]



/// vac
  /base_link /fiducial/tag_board
At time 1405693784.849
- Translation: [-0.692, -0.011, 1.001]
- Rotation: in Quaternion [0.523, 0.466, 0.463, 0.543]
            in RPY [1.552, 0.022, 1.435]


*/
                                       /////   EE    //////

static const tf::Vector3 FA_EE_OFFSET = tf::Vector3(-0.004, -0.038, -0.085);
static const tf::Quaternion FA_EE_ORIENTATION_OFFSET = tf::Quaternion(0.643, 0.283, 0.664, 0.257);

static const double TOOL_CHANGER_OFFSET_ANGLE = - 0.29;

                                    //////    GRIPPER    /////
//3 fidu // static const tf::Vector3 START_POINT_OFFSET_ARM = tf::Vector3(-0.106, -0.075, 0.21);
static const tf::Vector3 START_POINT_OFFSET_ARM = tf::Vector3(-0.106, -0.075, 0.21);
//3fidu //static const tf::Vector3 SLOT_POINT_OFFSET_ARM = tf::Vector3(-0.105, -0.075, 0.1345);
static const tf::Vector3 SLOT_POINT_OFFSET_ARM = tf::Vector3(-0.106, -0.075, 0.135);

//inportant y-axes
//3 fidu // static const tf::Vector3 SLOT_POINT_DOWN_ARM = tf::Vector3(-0.106, -0.086, 0.134);
static const tf::Vector3 SLOT_POINT_DOWN_ARM = tf::Vector3(-0.108, -0.087, 0.135);


static const tf::Vector3 START_POINT_OFFSET_COUPLE_ARM = tf::Vector3(-0.106, -0.065, 0.21);
static const tf::Vector3 SLOT_POINT_OFFSET_COUPLE_ARM = tf::Vector3(-0.106, -0.070, 0.135);

                                  /////   VAC   //////


static const tf::Vector3 START_POINT_OFFSET_VAC = tf::Vector3(0.064, -0.075, 0.21);
static const tf::Vector3 SLOT_POINT_OFFSET_VAC = tf::Vector3(0.064, -0.075, 0.137);

static const tf::Vector3 START_POINT_OFFSET_COUPLE_VAC = tf::Vector3(0.064, -0.065, 0.21);
static const tf::Vector3 SLOT_POINT_OFFSET_COUPLE_VAC = tf::Vector3(0.064, -0.070, 0.137);

static const tf::Vector3 SLOT_POINT_DOWN_VAC = tf::Vector3(0.063, -0.087, 0.137);


////Fidu arm
/*
 /base_link /fiducial/tag_board
At time 1405683169.854
- Translation: [-0.694, -0.016, 1.003]
- Rotation: in Quaternion [0.477, 0.526, 0.510, 0.486]
            in RPY [1.578, 0.024, 1.644]


/base_link /fiducial/tag_board
At time 1405505474.417
- Translation: [-0.680, -0.010, 1.003]
- Rotation: in Quaternion [0.473, 0.524, 0.514, 0.487]
            in RPY [1.568, 0.026, 1.649]

 *
 * //Fidu vac


 */

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

	//TF ELEMENTS
	tf::TransformListener transform_listener_;
	tf::TransformBroadcaster br_;

	///PUBLISHERS
	//ros::Publisher vis_pub_;

	/// SERVER
	boost::scoped_ptr<actionlib::SimpleActionServer<autopnp_tool_change::ToolChangeAction> > as_go_to_start_position_;
	boost::scoped_ptr<actionlib::SimpleActionServer<autopnp_tool_change::ToolChangeAction> > as_go_to_slot_;
	boost::scoped_ptr<actionlib::SimpleActionServer<autopnp_tool_change::ToolChangeAction> > as_go_back_to_start_;

	/// CLIENTS
	ros::ServiceClient execute_known_traj_client_ ;
	ros::Time latest_time_;


	bool slot_position_detected_;
	bool move_action_state_;
	bool detected_all_fiducials_;



	///CALLBACK FUNKTION
	// Callback for the incoming point cloud data stream of fiducials.
	void markerInputCallback(const cob_object_detection_msgs::DetectionArray::ConstPtr& input_marker_detections_msg);

	///SERVER CALLBACKS
	void goToStartPosition(const autopnp_tool_change::ToolChangeGoalConstPtr& goal);
	void goToSlot(const autopnp_tool_change::ToolChangeGoalConstPtr& goal);
	void goBackToStart(const autopnp_tool_change::ToolChangeGoalConstPtr& goal);


	///PROCESS COMPONENTS
	//bool processGoToSlot(const tf::Vector3& movement1, const tf::Vector3& movement2, const tf::Vector3& movement3);
	bool executeMoveCommand(const geometry_msgs::PoseStamped& pose);
	bool executeTurn(const tf::Quaternion& quat);
	bool executeStraightMoveCommand(const tf::Vector3& vector, const double max_step);
	bool processMoveOrTurn(const std::string& action, const std::string& tool,const std::string& received_state);
	bool processGoToSlot(const std::string& tool, const std::string& state);
	bool processGoToStartPosition(const std::string& received_goal,const std::string& received_state);
	bool processGoBackLift(const std::string& tool);
	bool processGoBackNormal(const std::string& tool);
	bool optimizeTranslation(const std::string& source_frame, const std::string& target_frame);
	bool executeTranslationZ(const std::string& source_frame, const std::string& target_frame);
	bool executeGoToSlotSession(const std::string& tool_name, const tf::StampedTransform& transformation,
			const std::string& source_frame, const std::string& target_frame);
	tf::Transform executeGoToStartSession(const std::string& action,const tf::StampedTransform& reference,
			const tf::StampedTransform& goal_transformation);

	//small functions to split the code
	void resetServers();
	void waitForMoveit();
	/// Computes an average pose from multiple detected markers.
	void computeMarkerPose(const cob_object_detection_msgs::DetectionArray::ConstPtr& input_marker_detections_msg);



	//HELPER VARIABLES AND FUNKTIONS TO PRINT
	void printPose(tf::Transform& trans_msg);
	void printMsg(const geometry_msgs::PoseStamped pose);
	void printVector(const std::vector<double> v);

	//void test();
};


#endif /* AUTOPNP_TOOL_CHANGE_ */

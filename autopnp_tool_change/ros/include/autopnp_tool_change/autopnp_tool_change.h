/*
 * object_recording.h
 *
 *  Created on: 02.07.2013
 *      Author: rbormann
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
#include <std_msgs/ColorRGBA.h>

#include <vector>
#include <autopnp_tool_change/MoveToWagonAction.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>

/**
 * Brief description:
 *
 * ToolChange-> The program subscribes to the topic
 * /fiducials/detect_fiducials to get the data from input marker
 * detection.Then it processes the data to get the tool position
 * with respect to kinect coordinate system
 */

class ToolChange
{

public:

	ToolChange(ros::NodeHandle nh);
	~ToolChange();
	void moveToWagon(const autopnp_tool_change::MoveToWagonGoalConstPtr& goal);
	void init();


protected:

    /// array of two transform msgs
	struct fiducials;
	struct fiducials {
		tf::Transform translation;
	};
	struct components;
	/// array of two fiducial objects
	struct components {
		struct fiducials arm;
		struct fiducials board;
	};

	/// instance of a subscriber for the camera calibration
	///action of incoming color image data
	ros::Subscriber input_color_camera_info_sub;
	/// ROS node handle
	ros::NodeHandle node_handle;

	/// SUBSCRIBERS
	message_filters::Subscriber<cob_object_detection_msgs::DetectionArray> input_marker_detection_sub;
	message_filters::Subscriber<sensor_msgs::JointState> joint_states_sub;
	///PUBLISHERS
	ros::Publisher vis_pub;
	/// SERVER
	actionlib::SimpleActionServer<autopnp_tool_change::MoveToWagonAction> move_to_wagon_server;


	/// messages that are used to published feedback/result
	autopnp_tool_change::MoveToWagonFeedback feedback;
	autopnp_tool_change::MoveToWagonResult result;

	/// projection matrix of the calibrated camera that transforms points
	///from 3D to image plane in homogeneous coordinates:
	///[u,v,w]=P*[X,Y,Z,1]
	cv::Mat color_camera_matrix;
	bool camera_matrix_received;
	bool slot_position_detected;

	///container for the joint msgs
	std::vector<double> jointVelocities;
	std::vector<double> jointPositions;

	///transformation data between the arm and the wagon
	tf::Transform arm_board_transform;

    ///helper functions to print the data
	void printPose(tf::Transform& trans_msg);
	void printMsg(const geometry_msgs::PoseStamped pose);
	void printVector(const std::vector<double> v);

	void jointInputCallback(const sensor_msgs::JointState::ConstPtr& input_joint_msg);
	/// Callback for the incoming pointcloud data stream.
	void markerInputCallback(const cob_object_detection_msgs::DetectionArray::ConstPtr& input_marker_detections_msg);

	//calculates translation and orientation distance between arm marker
	// and board marker
	tf::Transform calculateTransformationToArm(const tf::Transform& board, const tf::Transform& arm);
	/// Converts a color image message to cv::Mat format.
	bool convertColorImageMessageToMat(const sensor_msgs::Image::ConstPtr& image_msg,
			cv_bridge::CvImageConstPtr& image_ptr, cv::Mat& image);
	/// Computes an average pose from multiple detected markers.
	struct components computeMarkerPose(const cob_object_detection_msgs::DetectionArray::ConstPtr& input_marker_detections_msg);
	/// Projects a 3D point into the coordinates of the color camera image.
	unsigned long ProjectXYZ(double x, double y, double z, int& u, int& v);
	/// Callback function for receiving the camera calibration.
	void calibrationCallback(const sensor_msgs::CameraInfo::ConstPtr& calibration_msg);
	void moveArm();
	void drawArrow( double r,  double g, double b, double a, const geometry_msgs::PoseStamped& pose);
	void drawLine( double r,  double g, double b, double a, const geometry_msgs::PoseStamped& pose_start,
			const geometry_msgs::PoseStamped& pose_end);
};


#endif /* AUTOPNP_TOOL_CHANGE_ */

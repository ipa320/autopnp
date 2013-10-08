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

#include <vector>
#include <autopnp_tool_change/MoveToSlotAction.h>
#include <actionlib/server/simple_action_server.h>




class ToolChange
{
public:

	ToolChange(ros::NodeHandle nh);
	~ToolChange();
	void moveToSlot(const autopnp_tool_change::MoveToSlotGoalConstPtr& goal);
	void init();
	void printPose(tf::Transform& trans_msg);

protected:

	ros::Subscriber input_color_camera_info_sub_;	///< camera calibration of incoming color image data
	ros::NodeHandle node_handle_;			///< ROS node handle
	bool camera_matrix_received_;
	// create messages that are used to published feedback/result
	autopnp_tool_change::MoveToSlotFeedback feedback_;
	autopnp_tool_change::MoveToSlotResult result_;
	///< projection matrix of the calibrated camera that transforms points from 3D to image plane in homogeneous coordinates:
	///[u,v,w]=P*[X,Y,Z,1]
	cv::Mat color_camera_matrix_;
	///< detection of coordinate system the object is placed on
	message_filters::Subscriber<cob_object_detection_msgs::DetectionArray> input_marker_detection_sub_;
	actionlib::SimpleActionServer<autopnp_tool_change::MoveToSlotAction> move_to_slot_server;

	void callback(const geometry_msgs::PoseStamped::ConstPtr&  msg);
	/// Callback for the incoming pointcloud data stream.
	void inputCallback(const cob_object_detection_msgs::DetectionArray::ConstPtr& input_marker_detections_msg);
	//calculates translation and orientation distance between arm marker
	// and board
	void calculateDistanceToArm(const cob_object_detection_msgs::DetectionArray::ConstPtr& input_marker_detections_msg);
	/// Converts a color image message to cv::Mat format.
	bool convertColorImageMessageToMat(const sensor_msgs::Image::ConstPtr& image_msg,
			cv_bridge::CvImageConstPtr& image_ptr, cv::Mat& image);
	/// Computes an average pose from multiple detected markers.
	tf::Transform computeMarkerPose(const cob_object_detection_msgs::DetectionArray::ConstPtr& input_marker_detections_msg);
	/// Projects a 3D point into the coordinates of the color camera image.
	unsigned long ProjectXYZ(double x, double y, double z, int& u, int& v);
	/// Callback function for receiving the camera calibration.
	void calibrationCallback(const sensor_msgs::CameraInfo::ConstPtr& calibration_msg);


};


#endif /* AUTOPNP_TOOL_CHANGE_ */

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

#include <message_filters/subscriber.h>

// boost
#include <boost/bind.hpp>

// opencv
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <vector>

class ToolChange
{
public:

//	ObjectRecording();
	ToolChange(ros::NodeHandle nh);

	~ToolChange();

protected:

	/// Callback for the incoming pointcloud data stream.
	void inputCallback(const cob_object_detection_msgs::DetectionArray::ConstPtr& input_marker_detections_msg);

	/// Converts a color image message to cv::Mat format.
	bool convertColorImageMessageToMat(const sensor_msgs::Image::ConstPtr& image_msg, cv_bridge::CvImageConstPtr& image_ptr, cv::Mat& image);

	/// Computes an average pose from multiple detected markers.
	tf::Transform computeMarkerPose(const cob_object_detection_msgs::DetectionArray::ConstPtr& input_marker_detections_msg);

	/// Projects a 3D point into the coordinates of the color camera image.
	unsigned long ProjectXYZ(double x, double y, double z, int& u, int& v);

	/// Callback function for receiving the camera calibration.
	void calibrationCallback(const sensor_msgs::CameraInfo::ConstPtr& calibration_msg);

	message_filters::Subscriber<cob_object_detection_msgs::DetectionArray> input_marker_detection_sub_;	///< detection of coordinate system the object is placed on
	ros::Subscriber input_color_camera_info_sub_;	///< camera calibration of incoming color image data

	ros::NodeHandle node_handle_;			///< ROS node handle

	bool camera_matrix_received_;
	cv::Mat color_camera_matrix_;	///< projection matrix of the calibrated camera that transforms points from 3D to image plane in homogeneous coordinates: [u,v,w]=P*[X,Y,Z,1]
};

#endif /* AUTOPNP_TOOL_CHANGE_ */

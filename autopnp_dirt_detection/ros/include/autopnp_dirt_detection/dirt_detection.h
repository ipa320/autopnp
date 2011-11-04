/*
 * image_display.h
 *
 *  Created on: 06.10.2011
 *      Author: rmb-hs
 */

#ifndef IMAGE_DISPLAY_H_
#define IMAGE_DISPLAY_H_

//##################
//#### includes ####

// standard includes
//--

// ROS includes
#include <ros/ros.h>

// ROS message includes
#include <sensor_msgs/Image.h>

// topics
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

// opencv
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

namespace ipa_ImageDisplay {


//####################
//#### node class ####
class ImageDisplay
{
protected:
	image_transport::ImageTransport* it_;
	image_transport::Subscriber color_camera_image_sub_; ///< Color camera image topic

	ros::NodeHandle node_handle_; ///< ROS node handle


public:

	ImageDisplay(ros::NodeHandle node_handle);

	~ImageDisplay();

	void init();

	void imageDisplayCallback(const sensor_msgs::ImageConstPtr& color_image_msg);

	void imageDisplayCallback_old_cv_code(const sensor_msgs::ImageConstPtr& color_image_msg);

	void imageDisplayCallback_new_cv_code(const sensor_msgs::ImageConstPtr& color_image_msg);

	void imageDisplayCallback_channel_combination(const sensor_msgs::ImageConstPtr& color_image_msg);

	void oneChannelTrafo(cv::Mat& one_channel_image, cv::Mat& result_image);


	unsigned long convertColorImageMessageToMat(const sensor_msgs::Image::ConstPtr& color_image_msg, cv_bridge::CvImageConstPtr& color_image_ptr, cv::Mat& color_image);


};

};


#endif /* IMAGE_DISPLAY_H_ */

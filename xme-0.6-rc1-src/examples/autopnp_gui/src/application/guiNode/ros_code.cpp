#include "ros_code.h"

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
//#include <opencv/cv.h>
//#include <opencv/highgui.h>
//#include <cv_bridge/cv_bridge.h>
//#include <sensor_msgs/image_encodings.h>

#include <string.h>

class ImageReceiver
{
public:
	ImageReceiver(ros::NodeHandle nh, std::string display_topic_name, void (*displayImageFunction)(unsigned int, unsigned int, unsigned int, const std::vector<unsigned char>&))
	{
		node_handle_ = nh;
		display_image_function_ptr_ = displayImageFunction;

		it_ = new image_transport::ImageTransport(node_handle_);
		//color_camera_image_sub_.subscribe(*it_, "/cam3d/rgb/image", 1);
		color_camera_image_sub_.subscribe(*it_, display_topic_name, 1);
		color_camera_image_sub_.registerCallback(boost::bind(&ImageReceiver::imageCallback, this, _1));
	}

private:
	image_transport::ImageTransport* it_;
	image_transport::SubscriberFilter color_camera_image_sub_;	///< Color camera image input topic

	void imageCallback(const sensor_msgs::ImageConstPtr& color_image_msg)
	{
		//ROS_INFO("Received image with width=%i, height=%i and first pixel=(%i,%i,%i)", color_image_msg->width, color_image_msg->height, color_image_msg->data[0], color_image_msg->data[1], color_image_msg->data[2]);

		(*display_image_function_ptr_)(color_image_msg->width, color_image_msg->height, color_image_msg->step, color_image_msg->data);
	}

	ros::NodeHandle node_handle_;
	void (*display_image_function_ptr_)(unsigned int, unsigned int, unsigned int, const std::vector<unsigned char>&);
};


void RosInit::init(int argc, char *argv[], void (*displayImageFunction)(unsigned int, unsigned int, unsigned int, const std::vector<unsigned char>&))
{
	ros::init(argc, argv, "xme_ros_gui");

	ros::NodeHandle nh;

	std::string topic_name = "";
//	topic_name = "/cam3d/rgb/image";	// kinect pnp with openni2
//	topic_name = "/cam3d/rgb/image_color";		// kinect pnp with openni
	topic_name = "/dirt_detection/map_with_dirt_detections";	// display dirt detections

//	Chromosome does not provide the real argv
//	std::cout << "argc=" << argc << "\n";
//	if (argc==0)
//		topic_name = "/dirt_detection/map_with_dirt_detections";
//	else
//	{
//		fprintf(stdout, argv[0]);
//		if (strcmp(argv[0], "pnp-openni") == 0)
//			topic_name = "/cam3d/rgb/image_color";
//		else if (strcmp(argv[0], "pnp-openni2") == 0)
//			topic_name = "/cam3d/rgb/image";
//	}


	ImageReceiver ir(nh, topic_name, displayImageFunction);

	ROS_INFO("Connection to robot initialized.");

	while (ros::ok() && terminateRosThread==0)
	{
		ros::spinOnce();
	}
}

RosInit::RosInit()
{

}

RosInit::~RosInit()
{

}

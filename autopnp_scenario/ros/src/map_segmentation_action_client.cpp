#include <iostream>

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <autopnp_scenario/MapSegmentationAction.h>



void updateMapCallback(const nav_msgs::OccupancyGridConstPtr& map_msg)
	{

		double map_resolution_;
		cv::Point2d map_origin_;	// in [m]
		cv::Mat map_;

		map_resolution_ = map_msg->info.resolution;
		map_origin_ = cv::Point2d(map_msg->info.origin.position.x, map_msg->info.origin.position.y);

		// create empty copy of map
		map_ = 255*cv::Mat::ones(map_msg->info.height, map_msg->info.width, CV_8UC1);

		// copy real static map into cv::Mat element-wise
		for (unsigned int v=0, i=0; v<map_msg->info.height; v++)
			{
				for (unsigned int u=0; u<map_msg->info.width; u++, i++)
					{
						if (map_msg->data[i] != 0)
						map_.at<unsigned char>(v,u) = 0;
					}
			}

		autopnp_scenario::MapSegmentationGoal goal;

		actionlib::SimpleActionClient<autopnp_scenario::MapSegmentationAction> ac("segment_map", true);

		ROS_INFO("Waiting for action server to start.");
		// wait for the action server to start

		ac.waitForServer(); //will wait for infinite time

		ROS_INFO("Action server started, sending goal.");

		// send a goal to the action
		cv_bridge::CvImage cv_image;
		cv_image.header.stamp = ros::Time::now();
		cv_image.encoding = "mono8";
		cv_image.image = map_;
		cv_image.toImageMsg(goal.input_map);

		goal.map_resolution = map_resolution_;
		goal.map_origin_x = map_origin_.x;
		goal.map_origin_y = map_origin_.y;
		goal.return_format_in_meter = true ;
		goal.return_format_in_pixel = true ;

		ac.sendGoal(goal);

		//wait for the action to return
		bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

		if (finished_before_timeout)
			{
				actionlib::SimpleClientGoalState state = ac.getState();
				ROS_INFO("Action finished: %s",state.toString().c_str());
			}

		 else
			ROS_INFO("Action did not finish before the time out.");
	}



int main(int argc, char **argv)
	{
	  ros::init(argc, argv, "test_segment_map");

	  ros::NodeHandle n;

	  ros::Subscriber sub = n.subscribe("/map", 1000, updateMapCallback);

	  ros::spin();

	  return 0;
	}


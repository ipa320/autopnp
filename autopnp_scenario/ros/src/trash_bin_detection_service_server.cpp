/*!
 *****************************************************************
 * \file
 *
 * \note
 * Copyright (c) 2013 \n
 * Fraunhofer Institute for Manufacturing Engineering
 * and Automation (IPA) \n\n
 *
 *****************************************************************
 *
 * \note
 * Project name: care-o-bot
 * \note
 * ROS stack name: autopnp
 * \note
 * ROS package name: autopnp_scenario
 *
 * \author
 * Author: Mohammad Muinul Islam(email:mohammad.islam@ipa.fraunhofer.de)
 *
 * \author
 * Supervised by: Richard Bormann(email:richard.bormann@ipa.fraunhofer.de)
 *
 * \date Date of creation: September 2013
 *
 * \brief
 * Trash Bin Detection-> The program subscribes to the topic
 * /fiducials/detect_fiducials to get the data from trash bin marker
 * detection.Then it process the data to get the Trash Bin position
 * with respect to map coordinate system
 *
 *****************************************************************
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer. \n
 * - Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution. \n
 * - Neither the name of the Fraunhofer Institute for Manufacturing
 * Engineering and Automation (IPA) nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission. \n
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License LGPL for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/

#include <cmath>
#include <vector>
#include <iostream>
#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <cob_object_detection_msgs/Detection.h>

// services - here you have to include the header file with exactly the same name as your message in the /srv folder (the Message.h is automatically generated from your Message.srv file during compilation)
#include <autopnp_scenario/TrashBinDetection.h>

//https://github.com/ipa320/cob_perception_common/blob/groovy_dev/cob_object_detection_msgs/msg/Detection.msg
//https://github.com/ipa320/cob_object_perception/tree/groovy_dev/cob_fiducials/ros/launch

//global variable declaration
bool fiducials_data_recieved_;
ros::Subscriber fiducials_msg_sub_;
std::string fiducials_frame_id_;
std::string image_detection_label_;
geometry_msgs::PoseStamped pose_with_respect_to_fiducials_frame_id_;
geometry_msgs::PoseStamped pose_with_respect_to_map_;
std::vector<geometry_msgs::PoseStamped> pose_array_;

//global function declaration
void fiducialsInit(ros::NodeHandle& nh);
void fiducialsDataCallback(const cob_object_detection_msgs::Detection::ConstPtr& fiducials_msg_data);
bool TrashBinDetectionCallback(autopnp_scenario::TrashBinDetection::Request &req, autopnp_scenario::TrashBinDetection::Response &res);
bool similarity_checker(geometry_msgs::PoseStamped &present_value, geometry_msgs::PoseStamped &past_value);
geometry_msgs::PoseStamped average_calculator(geometry_msgs::PoseStamped &present_value, geometry_msgs::PoseStamped &past_value);

//Initialize Trash bin detection to receive necessary raw data
void fiducialsInit(ros::NodeHandle& nh)
{
	fiducials_data_recieved_ = false;
	fiducials_msg_sub_ = nh.subscribe<cob_object_detection_msgs::Detection>("/fiducials/detect_fiducials", 1, fiducialsDataCallback);
	ROS_INFO("TrashBinDetectionServer: Waiting to receive data...");
	while (fiducials_data_recieved_ == false)
		ros::spinOnce();

	ROS_INFO("TrashBinDetectionCheck: data received.");
}

//fiducials topic data call-back
void fiducialsDataCallback(const cob_object_detection_msgs::Detection::ConstPtr& fiducials_msg_data)
{
	fiducials_frame_id_ = fiducials_msg_data->header.frame_id;
	image_detection_label_ = fiducials_msg_data->label;
	pose_with_respect_to_fiducials_frame_id_ = fiducials_msg_data->pose;
	fiducials_data_recieved_ = true;
	fiducials_msg_sub_.shutdown();
}

//This function process the data to calculate the trash bin location
bool TrashBinDetectionCallback(autopnp_scenario::TrashBinDetection::Request &req, autopnp_scenario::TrashBinDetection::Response &res)
{
	if (req.service_on_off_switch == true)
	{
		ROS_INFO("Received request to turn-on trash bin detection.....");
		if (image_detection_label_ == "0")
		{
			ROS_INFO("Trash bin detection is turned-on.");
			tf::TransformListener listener_;
			tf::Stamped<tf::Pose> original_pose(
					tf::Pose(
							tf::Quaternion(pose_with_respect_to_fiducials_frame_id_.pose.orientation.x, pose_with_respect_to_fiducials_frame_id_.pose.orientation.y,
									pose_with_respect_to_fiducials_frame_id_.pose.orientation.z, pose_with_respect_to_fiducials_frame_id_.pose.orientation.w),
							tf::Vector3(pose_with_respect_to_fiducials_frame_id_.pose.position.x, pose_with_respect_to_fiducials_frame_id_.pose.position.y,
									pose_with_respect_to_fiducials_frame_id_.pose.position.z)), ros::Time(0), fiducials_frame_id_);
			tf::Stamped<tf::Pose> transformed_pose;
			listener_.transformPose("map", original_pose, transformed_pose);

			pose_with_respect_to_map_.pose.position.x = transformed_pose.getOrigin().x();
			pose_with_respect_to_map_.pose.position.y = transformed_pose.getOrigin().y();
			pose_with_respect_to_map_.pose.position.z = transformed_pose.getOrigin().z();
			pose_with_respect_to_map_.pose.orientation.x = transformed_pose.getRotation().x();
			pose_with_respect_to_map_.pose.orientation.y = transformed_pose.getRotation().y();
			pose_with_respect_to_map_.pose.orientation.z = transformed_pose.getRotation().z();
			pose_with_respect_to_map_.pose.orientation.w = transformed_pose.getRotation().w();
			pose_array_.push_back(pose_with_respect_to_map_);
			unsigned int container_value_checker = pose_array_.size() - 1;
			if (container_value_checker > 1)
			{
				if (similarity_checker(pose_array_[container_value_checker], pose_array_[container_value_checker - 1]) == true)
				{
					pose_array_[container_value_checker] = average_calculator(pose_array_[container_value_checker], pose_array_[container_value_checker - 1]);
					res.trash_bin_locations.push_back(pose_array_[container_value_checker]);
				}
			}
			else res.trash_bin_locations.push_back(pose_array_[container_value_checker]);
		}
	}
	else
	{
		ROS_INFO("Received request to turn-off trash bin detection.....");
		ROS_INFO("Trash bin detection is turned-off.");
		pose_array_.clear();
	}

	return true;
}

//This function checks the similarity between the two consecutive pose
bool similarity_checker(geometry_msgs::PoseStamped &present_value, geometry_msgs::PoseStamped &past_value)
{
	if ((abs(present_value.pose.position.x - past_value.pose.position.x)) < 0.09 && (abs(present_value.pose.position.y - past_value.pose.position.y)) < 0.09
			&& (abs(present_value.pose.position.z - past_value.pose.position.z)) < 0.09 && (abs(present_value.pose.orientation.x - past_value.pose.orientation.x)) < 0.09
			&& (abs(present_value.pose.orientation.y - past_value.pose.orientation.y)) < 0.09 && (abs(present_value.pose.orientation.z - past_value.pose.orientation.z)) < 0.09
			&& (abs(present_value.pose.orientation.w - past_value.pose.orientation.w)) < 0.09)
	{
		return true;
	}
	else
		return false;
}

geometry_msgs::PoseStamped average_calculator(geometry_msgs::PoseStamped &present_value, geometry_msgs::PoseStamped &past_value)
{
	geometry_msgs::PoseStamped average_value;
	average_value.pose.position.x = (present_value.pose.position.x + past_value.pose.position.x) / 2;
	average_value.pose.position.y = (present_value.pose.position.x + past_value.pose.position.x) / 2;
	average_value.pose.position.z = (present_value.pose.position.x + past_value.pose.position.x) / 2;
	average_value.pose.orientation.x = (present_value.pose.orientation.x + past_value.pose.orientation.x) / 2;
	average_value.pose.orientation.y = (present_value.pose.orientation.y + past_value.pose.orientation.y) / 2;
	average_value.pose.orientation.z = (present_value.pose.orientation.z + past_value.pose.orientation.z) / 2;
	average_value.pose.orientation.w = (present_value.pose.orientation.w + past_value.pose.orientation.w) / 2;
	return average_value;
}

int main(int argc, char **argv)
{
	/**
	 * The ros::init() function needs to see argc and argv so that it can perform
	 * any ROS arguments and name remapping that were provided at the command line. For programmatic
	 * remappings you can use a different version of init() which takes remappings
	 * directly, but for most command-line programs, passing argc and argv is the easiest
	 * way to do it.  The third argument to init() is the name of the node.
	 *
	 * You must call one of the versions of ros::init() before using any other
	 * part of the ROS system.
	 */
	ros::init(argc, argv, "trash_bin_detection_server");
	/**
	 * NodeHandle is the main access point to communications with the ROS system.
	 * The first NodeHandle constructed will fully initialize this node, and the last
	 * NodeHandle destructed will close down the node.
	 */
	ros::NodeHandle nh;
	fiducialsInit(nh);
	ros::ServiceServer trash_bin_detection_check_server_ = nh.advertiseService("trash_bin_detection_check", TrashBinDetectionCallback);
	ROS_INFO("Trash Bin detection Server initialized.....");
	ros::spin();
	return 0;
}


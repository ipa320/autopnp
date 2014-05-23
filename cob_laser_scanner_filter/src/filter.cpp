/****************************************************************
 *
 * Copyright (c) 2010
 *
 * Fraunhofer Institute for Manufacturing Engineering	
 * and Automation (IPA)
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Project name: care-o-bot
 * ROS stack name: cob_driver
 * ROS package name: cob_sick_s300
 * Description:
 *								
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *			
 * Author: Florian Weisshardt, email:florian.weisshardt@ipa.fhg.de
 * Supervised by: Christian Connette, email:christian.connette@ipa.fhg.de
 *
 * Date of creation: Jan 2010
 * ToDo:
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *	 * Redistributions of source code must retain the above copyright
 *	   notice, this list of conditions and the following disclaimer.
 *	 * Redistributions in binary form must reproduce the above copyright
 *	   notice, this list of conditions and the following disclaimer in the
 *	   documentation and/or other materials provided with the distribution.
 *	 * Neither the name of the Fraunhofer Institute for Manufacturing 
 *	   Engineering and Automation (IPA) nor the names of its
 *	   contributors may be used to endorse or promote products derived from
 *	   this software without specific prior written permission.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as 
 * published by the Free Software Foundation, either version 3 of the 
 * License, or (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License LGPL for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public 
 * License LGPL along with this program. 
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/

//##################
//#### includes ####

// standard includes
//--

// ROS includes
#include <ros/ros.h>

// ROS message includes
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/String.h>

//####################
//#### node class ####
class NodeClass
{
	//
	public:
		  
		ros::NodeHandle nh;   
		ros::Publisher laser_pub_;
		ros::Subscriber laser_sub_, mode_sub_;
		size_t mode_;
		
		struct Param {
			std::string name_;
			struct Angles {
				double start_, stop_;
			};
			std::vector<Angles> angles_;
		};
		
		std::vector<Param> params_;

		// Constructor
		NodeClass()  : mode_(0)
		{
			// create a handle for this node, initialize node
			nh = ros::NodeHandle("~");
			
			{
				Param param;
				param.name_ = "default";
				params_.push_back(param);
			}

			//get params for each measurement
			XmlRpc::XmlRpcValue field_params;
			if(nh.getParam("fields",field_params) && field_params.getType() == XmlRpc::XmlRpcValue::TypeStruct)
			{
				for(XmlRpc::XmlRpcValue::iterator field=field_params.begin(); field!=field_params.end(); field++)
				{
					std::string name = field->first;
					
					if(!field->second.hasMember("start_angle"))
					{
						ROS_ERROR("Missing parameter start_angle");
						continue;
					}
					if(!field->second.hasMember("stop_angle"))
					{
						ROS_ERROR("Missing parameter stop_angle");
						continue;
					}
					
					Param::Angles a;
					a.start_ = field->second["start_angle"];
					a.stop_ = field->second["stop_angle"];

					bool found=false;
					for(size_t i=0; i<params_.size(); i++)
						if(params_[i].name_ == name) {
							params_[i].angles_.push_back(a);
							found = true;
							break;
						}
						
					if(!found) {
						Param param;
						param.name_ = name;
						param.angles_.push_back(a);
						params_.push_back(param);
					}

					ROS_DEBUG("params %s %f %f", name.c_str(), a.start_, a.stop_);
				}
			}
			else
			{
				ROS_WARN("No params set");
			}
			
			// implementation of topics to publish
			laser_pub_ = nh.advertise<sensor_msgs::LaserScan>("scan_out", 1);
			laser_sub_ = nh.subscribe("scan_in", 1, &NodeClass::laserCallback, this);
			mode_sub_ = nh.subscribe("mode", 1, &NodeClass::modeCallback, this);
		}
		
		// Destructor
		~NodeClass() 
		{
		}
		
		void modeCallback(const std_msgs::String &mode) {
			for(size_t i=0; i<params_.size(); i++)
				if(params_[i].name_ == mode.data) {
					mode_ = i;
					return;
				}
				
			ROS_ERROR("mode is not supported!");
		}
		
		void laserCallback(sensor_msgs::LaserScan scan) {
			if(mode_>=params_.size()) return;
			
			ROS_DEBUG("filter: %s %d", params_[mode_].name_.c_str(), (int)params_[mode_].angles_.size());
			
			for(size_t i=0; i<params_[mode_].angles_.size(); i++) {
				int a,b;
				
				a = (params_[mode_].angles_[i].start_-scan.angle_min)/(scan.angle_max-scan.angle_min)*(scan.ranges.size()-1);
				b = (params_[mode_].angles_[i].stop_-scan.angle_min)/(scan.angle_max-scan.angle_min)*(scan.ranges.size()-1);
				
				a = std::max(a, 0);
				b = std::min(b, (int)scan.ranges.size()-1);
				
				for(int j=a; j<=b; j++)
					scan.ranges[j] = 0.0;
			}
			
			laser_pub_.publish(scan);
		}
};

//#######################
//#### main programm ####
int main(int argc, char** argv)
{
	// initialize ROS, spezify name of node
	ros::init(argc, argv, "laser_scanner_filter");
	
	NodeClass nodeClass;
	ros::spin();
	
	return 0;
}

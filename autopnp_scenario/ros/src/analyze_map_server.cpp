#include <iostream>

#include <ros/ros.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <actionlib/server/simple_action_server.h>
#include <autopnp_scenario/AnalyzeMapAction.h>

#include <autopnp_scenario/exploration_algorithm_action_server_library.h>


class Analyze_Map_Server
	{
		protected:
			ros::NodeHandle nh_;
			actionlib::SimpleActionServer<autopnp_scenario::AnalyzeMapAction> ams_;
			std::string action_name_;
			autopnp_scenario::AnalyzeMapFeedback feedback_;
			autopnp_scenario::AnalyzeMapResult result_;

		public:
			Exploration exploration_obj;

			Analyze_Map_Server(std::string name):
				ams_(nh_,name,boost::bind(&Analyze_Map_Server::executeCB,this,_1),false),action_name_(name)
				{
					ams_.start();
				}

			~Analyze_Map_Server(void){}

			void executeCB(const autopnp_scenario::AnalyzeMapGoalConstPtr &goal)
				{
					ros::Rate r(1);

					cv_bridge::CvImagePtr cv_ptr;
					cv_ptr = cv_bridge::toCvCopy(goal->input_img, sensor_msgs::image_encodings::MONO8);
					cv::Mat original_img;
					original_img = cv_ptr->image;

					cv::Mat Segmented_map ;
					Segmented_map = exploration_obj.Image_Segmentation( original_img , goal->map_resolution );

					r.sleep();

					//Publish Result message:
					cv_bridge::CvImage cv_image;
					cv_image.header.stamp = ros::Time::now();
					cv_image.encoding = "mono8";
					cv_image.image = Segmented_map;
					cv_image.toImageMsg(result_.output_img);

					result_.room_center_x = exploration_obj.get_Center_of_Room_x();
					result_.room_center_y = exploration_obj.get_Center_of_Room_y();
					result_.map_resolution = goal->map_resolution;
					result_.Map_Origin_x = goal->Map_Origin_x;
					result_.Map_Origin_y = goal->Map_Origin_y;
					result_.room_min_x = exploration_obj.get_room_min_x();
					result_.room_min_y = exploration_obj.get_room_min_y();
					result_.room_max_x = exploration_obj.get_room_max_x();
					result_.room_max_y = exploration_obj.get_room_max_y();

					exploration_obj.get_Center_of_Room_x().clear();
					exploration_obj.get_Center_of_Room_y().clear();
					exploration_obj.get_room_min_x().clear();
					exploration_obj.get_room_max_x().clear();
					exploration_obj.get_room_min_y().clear();
					exploration_obj.get_room_max_y().clear();

					ams_.setSucceeded(result_);
				}
	};



int main(int argc,char** argv)
	{
		ros::init(argc,argv,"Analyze_Map");

		Analyze_Map_Server AMS(ros::this_node::getName());

		ros::spin();

		return 0;
	}


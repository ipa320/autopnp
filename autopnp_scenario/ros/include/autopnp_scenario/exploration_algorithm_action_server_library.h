#ifndef EXPLORATION_ALGORITHM_ACTION_SERVER_LIBRARY_HH
#define EXPLORATION_ALGORITHM_ACTION_SERVER_LIBRARY_HH

#include <iostream>
#include <string>
#include <complex>
#include <vector>
#include <cmath>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PoseStamped.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>

#include <nav_msgs/GridCells.h>
#include <geometry_msgs/Point.h>

struct Pose
	{
		float x;
		float y;
		float orientation; //in degree

		Pose()
			{
				x = 0;
				y = 0;
				orientation = 0;
			}

		Pose(float x_, float y_, float orientation_)
			{
				x = x_;
				y = y_;
				orientation = orientation_;
			}
	};


class Exploration
	{
		public:
			typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
			Exploration();

			double map_resolution_;		// in [m/cell]
			double robotRadius;			// in [m]

			int Center_Position_x ;
			int Center_Position_y ;

			std::vector < int > Room_min_x;
			std::vector < int > Room_max_x;
			std::vector < int > Room_min_y;
			std::vector < int > Room_max_y;

			std::vector<int> room_number;

			std::vector < cv::Point > Center_of_Room;
			std::vector < int > Center_of_Room_x;
			std::vector < int > Center_of_Room_y;

			std::vector<double> Inflation_data_X;
			std::vector<double> Inflation_data_Y;


			cv::Point random_location_point;

			cv::Point2d map_origin_;	// in [m]

			cv::Mat expanded_map_;
			cv::Mat Segmented_Map;
			cv::Mat Room_Inspection_Map_Show;
			cv::Mat Get_Map;

			tf::TransformListener listener;
			tf::StampedTransform transform;

			template <class T>
			T convertFromMeterToPixelCoordinates(const Pose& pose)
				{
					T val;
					val.x = (pose.x - map_origin_.x)/map_resolution_;
					val.y = (pose.y - map_origin_.y)/map_resolution_;
					return val;
				}

			cv::Mat Image_Segmentation( cv::Mat Original_Map , double map_resolution = 0.05 );

			void Find_Next_Room( cv::Mat FNR,
								 std::vector<cv::Point> center_of_room,
								 std::vector<int> center_of_room_x,
								 std::vector<int> center_of_room_y );

			std::string go_to_destination( cv::Mat FTL , int CenterPositionX , int CenterPositionY );


			cv::Mat random_location(cv::Mat RL,
									std::vector<int> room_Number,
									std::vector<int> room_min_x,
									std::vector<int> room_max_x,
									std::vector<int> room_min_y,
									std::vector<int> room_max_y,
									int unsuccessful_times);


			cv::Mat Room_Inspection(cv::Mat RI,
									std::vector<cv::Point> center_of_room,
									std::vector<int> room_Number,
									std::vector<int> center_of_room_x ,
									std::vector<int> center_of_room_y,
									std::vector<int> room_min_x,
									std::vector<int> room_max_x,
									std::vector<int> room_min_y,
									std::vector<int> room_max_y);

			bool Obstacle_free_Point( cv::Mat OFP , int x , int y );

			//Set and Get Methods for action server
			void Set_Map_Resoloution_and_Origin( double temp_Map_Res , cv::Point2d temp_map_origin );
			std::vector<int>& get_Center_of_Room_x();
			std::vector<int>& get_Center_of_Room_y();
			std::vector<int>& get_room_number();
			std::vector<int>& get_room_min_x();
			std::vector<int>& get_room_max_x();
			std::vector<int>& get_room_min_y();
			std::vector<int>& get_room_max_y();
			int get_center_position_x();
			int get_center_position_y();
			cv::Point get_random_location();

			//Move Base Action Server Methods
			move_base_msgs::MoveBaseGoal Move( double X , double Y , double Z );
			move_base_msgs::MoveBaseGoal Move_in_pixel( int X , int Y );
			move_base_msgs::MoveBaseGoal stay_forward( int X , int Y );

			//Inflation Data read and Call-back
			ros::NodeHandle Inflation_node;
			ros::Subscriber Inflation_sub;

			void InflationDataCallback(const nav_msgs::GridCells::ConstPtr& msg);
			void InflationInit(ros::NodeHandle nh);
	};


inline Exploration::Exploration()
	{
		map_resolution_ = 0 ;		// in [m/cell]
		robotRadius = 0.4 ;			// in [m]
	}


#endif

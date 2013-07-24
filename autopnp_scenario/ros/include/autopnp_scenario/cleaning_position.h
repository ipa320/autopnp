#ifndef CLEANING_POSITION_HH
#define CLEANING_POSITION_HH

#include <iostream>
#include <string>
#include <complex>
#include <vector>
#include <cmath>

#include <ros/ros.h>

#include <nav_msgs/OccupancyGrid.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PoseStamped.h>

#include <nav_msgs/GridCells.h>
#include <geometry_msgs/Point.h>

#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <boost/thread/mutex.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/timer.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>

#define PI 3.14159265

class Clean_Pose
	{
		public:
			Clean_Pose();

			typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

			std::vector<double> Inflation_data_X;
			std::vector<double> Inflation_data_Y;

			double map_resolution_;		// in [m/cell]
			double robotRadius;			// in [m]
			cv::Point2d map_origin_;	// in [m]

			cv::Mat map_;

			bool once;
			bool map_data_recieved_;

			ros::NodeHandle inflation_node_;

			tf::TransformListener listener;
			tf::StampedTransform transform;

			ros::Subscriber map_msg_sub_;

			message_filters::Subscriber<nav_msgs::GridCells> obstacles_sub_;
			message_filters::Subscriber<nav_msgs::GridCells> inflated_obstacles_sub_;

			typedef message_filters::sync_policies::ApproximateTime<nav_msgs::GridCells, nav_msgs::GridCells> InflatedObstaclesSyncPolicy;
			boost::shared_ptr < message_filters::Synchronizer < InflatedObstaclesSyncPolicy > > inflated_obstacles_sub_sync_; 	//< Synchronizer

			boost::mutex mutex_inflation_topic_;
			boost::condition_variable condition_inflation_topic_;

			void MapDataCallback( const nav_msgs::OccupancyGrid::ConstPtr& map_msg_data);

			void InflationDataCallback( const nav_msgs::GridCells::ConstPtr& obstacles_data,
										const nav_msgs::GridCells::ConstPtr& inflated_obstacles_data);

			void MapInit(ros::NodeHandle nh_map);

			void InflationInit(ros::NodeHandle nh);

			void cleaning_pose( int center_of_circle_x, int center_of_circle_y );

			void Set_Map_Resoloution_and_Origin( double temp_Map_Res , cv::Point2d temp_map_origin );

			move_base_msgs::MoveBaseGoal Move_in_pixel( int X , int Y );
			move_base_msgs::MoveBaseGoal stay_forward( int X , int Y );
			move_base_msgs::MoveBaseGoal stay_backward( int X , int Y , int CircleCenterX , int CircleCenterY );
	};


inline Clean_Pose::Clean_Pose()
	{
		robotRadius = 0.4 ;
		once =true;
	}


#endif

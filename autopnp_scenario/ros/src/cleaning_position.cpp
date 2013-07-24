#include <autopnp_scenario/cleaning_position.h>



void Clean_Pose::Set_Map_Resoloution_and_Origin( double temp_Map_Res , cv::Point2d temp_map_origin )
	{
		map_resolution_ = temp_Map_Res;
		map_origin_ = temp_map_origin;
	}



void Clean_Pose::MapDataCallback( const nav_msgs::OccupancyGrid::ConstPtr& map_msg_data)
	{
		map_resolution_ = map_msg_data->info.resolution;
		map_origin_ = cv::Point2d(map_msg_data->info.origin.position.x, map_msg_data->info.origin.position.y);
		std::cout<<"\nmap resolution: "<<map_msg_data->info.resolution<<std::endl;

		// create empty copy of map
		map_ = 255*cv::Mat::ones(map_msg_data->info.height, map_msg_data->info.width, CV_8UC1);

		// copy real static map into cv::Mat element-wise
		for (unsigned int v=0, i=0; v<map_msg_data->info.height; v++)
			{
				for (unsigned int u=0; u<map_msg_data->info.width; u++, i++)
					{
						if (map_msg_data->data[i] != 0)
						map_.at<unsigned char>(v,u) = 0;
					}
			}

		cv::imshow("Original Map", map_ );
		cv::waitKey();

		map_data_recieved_ = true;
		map_msg_sub_.shutdown();
		ROS_INFO("Map received.");
	}



void Clean_Pose::MapInit( ros::NodeHandle nh_map )
	{
		map_data_recieved_ = false;
		map_msg_sub_ = nh_map.subscribe<nav_msgs::OccupancyGrid>("/map", 1, &Clean_Pose::MapDataCallback, this);
		ROS_INFO("Waiting to receive map...");
		while (map_data_recieved_ == false)
		{
				ros::spinOnce();
		}
	}



void Clean_Pose::InflationDataCallback( const nav_msgs::GridCells::ConstPtr& obstacles_data,
										const nav_msgs::GridCells::ConstPtr& inflated_obstacles_data)
	{
		{
			boost::mutex::scoped_lock lock(mutex_inflation_topic_);

			Inflation_data_X.clear();
			Inflation_data_Y.clear();

			for(unsigned int i=0; i< obstacles_data->cells.size() ; i++)
				{
					Inflation_data_X.push_back(obstacles_data->cells[i].x);
					Inflation_data_Y.push_back(obstacles_data->cells[i].y);
				}

			for(unsigned int i=0; i< inflated_obstacles_data->cells.size() ; i++)
				{
					Inflation_data_X.push_back(inflated_obstacles_data->cells[i].x);
					Inflation_data_Y.push_back(inflated_obstacles_data->cells[i].y);
				}
		}

		condition_inflation_topic_.notify_one();

		//if(once)
			//{
				cleaning_pose(493,346);
				//once = false;
			//}

	}



void Clean_Pose::InflationInit(ros::NodeHandle nh)
	{
		inflation_node_ = nh;

		obstacles_sub_.subscribe(inflation_node_, "/move_base/local_costmap/obstacles", 1);
		inflated_obstacles_sub_.subscribe(inflation_node_, "/move_base/local_costmap/inflated_obstacles", 1);

		inflated_obstacles_sub_sync_ = boost::shared_ptr<message_filters::Synchronizer<InflatedObstaclesSyncPolicy> >(new message_filters::Synchronizer<InflatedObstaclesSyncPolicy>(InflatedObstaclesSyncPolicy(3)));
		inflated_obstacles_sub_sync_->connectInput(obstacles_sub_, inflated_obstacles_sub_);
		inflated_obstacles_sub_sync_->registerCallback(boost::bind(&Clean_Pose::InflationDataCallback, this, _1, _2));
	}



void Clean_Pose::cleaning_pose( int center_of_circle_x = 493 , int center_of_circle_y = 346 )
	{
		bool found_pose = false;

		cv::Point Clean_Point;

		MoveBaseClient ac("move_base", true);

		cv::Mat cleaning_pose_map = map_.clone();

		boost::mutex::scoped_lock lock(mutex_inflation_topic_);
		boost::system_time const timeout=boost::get_system_time() + boost::posix_time::milliseconds(5000);
/*
		if (condition_inflation_topic_.timed_wait(lock, timeout))
			{
				std::cout << "Got the inflation data." << std::endl;
			}

		else
			{
				std::cout << "Inflation data not available." << std::endl;
			}
*/
		for( unsigned int idx = 0; idx < Inflation_data_X.size(); idx++ )
			{
				cleaning_pose_map.at<unsigned char>( (Inflation_data_Y[idx]- map_origin_.y)/map_resolution_ , (Inflation_data_X[idx]- map_origin_.x)/map_resolution_ ) = 0;
			}

		cv::imshow("Inflated Map", cleaning_pose_map );
		cv::waitKey();

		for (int angle = 0; angle <= 360 && found_pose == false; angle+=10)
			{
				Clean_Point.x = center_of_circle_x + ((( robotRadius * 1.5 ) * cos (( angle * PI / 180.0 ))))/map_resolution_;
				Clean_Point.y = center_of_circle_y + ((( robotRadius * 1.5 ) * sin (( angle * PI / 180.0 ))))/map_resolution_;

				std::cout<<"\n Pixel value at clean point"<<(int)cleaning_pose_map.at<unsigned char>(Clean_Point);

				if( cleaning_pose_map.at<unsigned char>(Clean_Point)!= 0 )
					{
						found_pose = true;
					}
			}

		if (found_pose == true)
			{
				std::cout <<"\nCenter of room x value: "<<center_of_circle_x;
				std::cout <<"\nCenter of room y value: "<< center_of_circle_y;
				std::cout <<"\nClean Point of room x value: "<< Clean_Point.x;
				std::cout <<"\nClean Point of room y value: "<< Clean_Point.y<<std::endl;

				ac.waitForServer();
				ac.sendGoal( stay_forward( Clean_Point.x , Clean_Point.y ));
				ac.waitForResult();

				ac.waitForServer();
				ac.sendGoal( Move_in_pixel( Clean_Point.x , Clean_Point.y ));
				ac.waitForResult();

				ac.waitForServer();
				ac.sendGoal( stay_backward( Clean_Point.x , Clean_Point.y , center_of_circle_x , center_of_circle_y ));
				ac.waitForResult();
			}

		else
			std::cout << "\nNo valid pose found."<< std::endl;
	}



move_base_msgs::MoveBaseGoal Clean_Pose::Move_in_pixel( int X , int Y  )
	{
		ros::Rate rate(1);
		listener.lookupTransform("/map", "/base_link",ros::Time(0), transform);
		rate.sleep();

		move_base_msgs::MoveBaseGoal Goal;
		geometry_msgs::PoseStamped goal;

		double p = ( X * map_resolution_ ) + map_origin_.x ;
		double q = ( Y * map_resolution_ ) + map_origin_.y ;

		double m = transform.getOrigin().x();
		double n = transform.getOrigin().y();

		double angel = atan2((q-n),(p-m));

		goal.header.frame_id = "map";
		goal.header.stamp = ros::Time::now();

		goal.pose.position.x = p ;
		goal.pose.position.y = q;
		tf::Quaternion quat = tf::createQuaternionFromYaw(angel);
		tf::quaternionTFToMsg(quat, goal.pose.orientation);

		Goal.target_pose = goal;

		return Goal;
	}



move_base_msgs::MoveBaseGoal Clean_Pose::stay_forward( int X , int Y )
	{
		ros::Rate rate(1);
		listener.lookupTransform("/map", "/base_link",ros::Time(0), transform);
		rate.sleep();

		move_base_msgs::MoveBaseGoal Goal;
		geometry_msgs::PoseStamped goal;

		double p = ( X * map_resolution_ ) + map_origin_.x ;
		double q = ( Y * map_resolution_ ) + map_origin_.y ;

		double m = transform.getOrigin().x();
		double n = transform.getOrigin().y();

		double angel = atan2((q-n),(p-m));

		goal.header.frame_id = "map";
		goal.header.stamp = ros::Time::now();

		goal.pose.position.x = m ;
		goal.pose.position.y = n;
		tf::Quaternion quat = tf::createQuaternionFromYaw(angel);
		tf::quaternionTFToMsg(quat, goal.pose.orientation);

		Goal.target_pose = goal;

		return Goal;
	}



move_base_msgs::MoveBaseGoal Clean_Pose::stay_backward( int X , int Y , int CircleCenterX , int CircleCenterY )
	{
		move_base_msgs::MoveBaseGoal Goal;
		geometry_msgs::PoseStamped goal;

		double p = ( X * map_resolution_ ) + map_origin_.x ;
		double q = ( Y * map_resolution_ ) + map_origin_.y ;

		double b = ( CircleCenterX * map_resolution_ ) + map_origin_.x ;
		double v = ( CircleCenterY * map_resolution_ ) + map_origin_.y ;

		double angel = atan2((q - v),(p - b));

		goal.header.frame_id = "map";
		goal.header.stamp = ros::Time::now();

		goal.pose.position.x = p ;
		goal.pose.position.y = q;
		tf::Quaternion quat = tf::createQuaternionFromYaw(angel);
		tf::quaternionTFToMsg(quat, goal.pose.orientation);

		Goal.target_pose = goal;

		return Goal;
	}



int main(int argc, char** argv)
	{
		  ros::init(argc, argv, "Clean_Pose_Base");

		  ros::NodeHandle node2;

		  ROS_INFO("Sending goal");

		  Clean_Pose My;

		  My.MapInit(node2);

		  My.InflationInit(node2);

		  ros::spin();

		  return 0;
	}



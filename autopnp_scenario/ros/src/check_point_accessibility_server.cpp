#include <autopnp_scenario/move_base_location_on_perimeter.h>



void Clean_Pose::MapDataCallback( const nav_msgs::OccupancyGrid::ConstPtr& map_msg_data)
	{
//		{
//			boost::mutex::scoped_lock lock(mutex_inflation_topic_);

			map_resolution_ = map_msg_data->info.resolution;
			map_origin_ = cv::Point2d(map_msg_data->info.origin.position.x, map_msg_data->info.origin.position.y);
			std::cout<<"map resolution: "<<map_msg_data->info.resolution;

			original_map_ = 255*cv::Mat::ones(map_msg_data->info.height, map_msg_data->info.width, CV_8UC1);

			for (unsigned int v=0, i=0; v<map_msg_data->info.height; v++)
				{
					for (unsigned int u=0; u<map_msg_data->info.width; u++, i++)
						{
							if (map_msg_data->data[i] != 0)
							original_map_.at<unsigned char>(v,u) = 0;
						}
				}
//		}
//
//		condition_inflation_topic_.notify_one();

		double inflation_thickness_;
		ros::param::get("/move_base/local_costmap/robot_radius", inflation_thickness_);

		std::cout<<"\nInflation thickness: "<<inflation_thickness_<<std::endl;

		cv::Mat working_map = original_map_.clone();

		Black_Pixel_x.clear();
		Black_Pixel_y.clear();

		for(int y = 1; y < working_map.cols; y++)
			{
				for(int x = 1; x < working_map.rows; x++)
					{
						if (working_map.at<unsigned char>(x,y) == 0 &&
						   (working_map.at<unsigned char>(x+1,y) == 255||
						    working_map.at<unsigned char>(x,y+1) == 255||
						    working_map.at<unsigned char>(x+1,y+1) == 255||
						    working_map.at<unsigned char>(x-1,y) == 255||
						    working_map.at<unsigned char>(x,y-1) == 255||
						    working_map.at<unsigned char>(x-1,y-1) == 255))
							{
								Black_Pixel_x.push_back(x);
								Black_Pixel_y.push_back(y);
							}
					}
			}

		inflated_original_map_ = original_map_.clone();

		for( unsigned int idx = 0 ; idx < Black_Pixel_x.size(); idx++)
			{
				for(int i = 0; i < (int)(inflation_thickness_/map_resolution_ ); i++)
					{
						for(int j = 0; j < (int)(inflation_thickness_/map_resolution_ ); j++)
							{
								inflated_original_map_.at<unsigned char>( (Black_Pixel_x[idx] + j ) , (Black_Pixel_y[idx] + i) ) = 0;
								inflated_original_map_.at<unsigned char>( (Black_Pixel_x[idx] - j ) , (Black_Pixel_y[idx] + i) ) = 0;
								inflated_original_map_.at<unsigned char>( (Black_Pixel_x[idx] - j ) , (Black_Pixel_y[idx] - i) ) = 0;
								inflated_original_map_.at<unsigned char>( (Black_Pixel_x[idx] + j ) , (Black_Pixel_y[idx] - i) ) = 0;
							}
					}
			}

		//TODO: you can comment the following two lines to not pop up the inflated original map
		cv::imshow("Inflated Original Map", inflated_original_map_ );
		cv::waitKey();

		map_data_recieved_ = true;
		ROS_INFO("Map received.");
		map_msg_sub_.shutdown();
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

			inflated_map_ = inflated_original_map_.clone();
			for(unsigned int i=0; i< obstacles_data->cells.size() ; i++)
				{
					inflated_map_.at<uchar>((obstacles_data->cells[i].y-map_origin_.y)/map_resolution_, (obstacles_data->cells[i].x-map_origin_.x)/map_resolution_) = 0;
				}

			for(unsigned int i=0; i< inflated_obstacles_data->cells.size() ; i++)
				{
					inflated_map_.at<uchar>((inflated_obstacles_data->cells[i].y-map_origin_.y)/map_resolution_, (inflated_obstacles_data->cells[i].x-map_origin_.x)/map_resolution_) = 0;
				}
		}

		//TODO: you can comment the following two lines to not pop up the inflated original map
		cv::imshow("Inflated Map", inflated_map_ );
		cv::waitKey();

//		condition_inflation_topic_.notify_one();
	}



std::vector<bool> Clean_Pose::get_available_points_( std::vector<geometry_msgs::Pose2D>& checking_points )
	{
		std::vector<bool> obstacle_free_points(checking_points.size(), true);

		obstacle_free_points.clear();

		for(unsigned int i = 0 ; i < checking_points.size() ; i++ )
			{
				if (inflated_map_.at<uchar>( checking_points[i].y , checking_points[i].x ) == 0 )
					obstacle_free_points[i] = false;
			}

		return obstacle_free_points;
	}



void Clean_Pose::InflationInit(ros::NodeHandle nh)
	{
		inflation_node_ = nh;

		obstacles_sub_.subscribe(inflation_node_, "/move_base/local_costmap/obstacles", 1);
		inflated_obstacles_sub_.subscribe(inflation_node_, "/move_base/local_costmap/inflated_obstacles", 1);

		inflated_obstacles_sub_sync_ = boost::shared_ptr<message_filters::Synchronizer<InflatedObstaclesSyncPolicy> >(new message_filters::Synchronizer<InflatedObstaclesSyncPolicy>(InflatedObstaclesSyncPolicy(30)));
		inflated_obstacles_sub_sync_->connectInput(obstacles_sub_, inflated_obstacles_sub_);
		inflated_obstacles_sub_sync_->registerCallback(boost::bind(&Clean_Pose::InflationDataCallback, this, _1, _2));
	}



bool CheckPoints(autopnp_scenario::CheckPointAccessibility::Request &req,
				 autopnp_scenario::CheckPointAccessibility::Response &res)
	{
		Clean_Pose CP;
		ROS_INFO("Result: ");
		ROS_INFO("Goal Point x : %f",req.points_to_check[0].x);
		ROS_INFO("Goal Point y : %f",req.points_to_check[0].y);
		std::vector<bool> temp;
		temp.clear();
		temp = CP.get_available_points_( req.points_to_check );
		for(uchar i = 0 ; i < temp.size(); i++)
			{
				res.accessibility_flag[i] = temp [i];
			}
		ROS_INFO( "Result : %d",res.accessibility_flag[0]);
		return true;
	}



int main(int argc, char** argv)
	{
		ros::init(argc, argv, "Check_Points");

		ros::NodeHandle node2;

		Clean_Pose My;

		My.MapInit(node2);

		My.InflationInit(node2);

		//Service to provide the availability of points
		ros::ServiceServer service = node2.advertiseService("search_obstacle_free_point", CheckPoints );

		ROS_INFO("Ready to search for a obstacle free point.....");

		ros::spin();

		return 0;
	}



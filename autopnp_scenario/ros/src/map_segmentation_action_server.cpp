#include <autopnp_scenario/map_segmentation_algorithm.h>

segmentation_algorithm::segmentation_algorithm(std::string name_of_the_action) :
		analyze_map_action_server_variable_redefinition_(nh_, name_of_the_action, boost::bind(&segmentation_algorithm::execute_map_segmentation_server, this, _1), false), action_name_(
				name_of_the_action)
{
	analyze_map_action_server_variable_redefinition_.start();

	//Initialize the map resolution and container values
	map_resolution_ = 0;
	map_sampling_factor_ = 1.5;
	room_area_ = 0.0;
	room_area_factor_lower_limit_ = 3.0;
	room_area_factor_upper_limit_ = 40.0;

	room_number_.clear();
	minimum_x_coordinate_value_of_the_room_.clear();
	maximum_x_coordinate_value_of_the_room_.clear();
	minimum_y_coordinate_value_of_the_room_.clear();
	maximum_y_coordinate_value_of_the_room_.clear();
	center_of_room_.clear();
	x_coordinate_value_of_the_room_center_.clear();
	y_coordinate_value_of_the_room_center_.clear();
	temporary_contours_.clear();
	saved_contours_.clear();
	hierarchy_.clear();
	black_pixel_.clear();
	neighbourhood_pixel_.clear();
}

/* This is the map segmentation algorithm,does the following steps:
 * 1. collect the navigation data from analyze map action client
 * 2. erode the image until get the last room
 * 3. find contours
 * 4. for each contour check if contour fulfills criteria of a room
 * 5. if contour fulfill the criteria save the contour somewhere else
 * 6. exit the loop if there are no more contours
 * 7. copy original image
 * 8. copy the stored contours into the map and fill each with a unique
 *    id number repeat until convergence(i.e there are no more white pixels)
 */
cv::Mat segmentation_algorithm::Image_Segmentation_method(cv::Mat &Original_Map_from_subscription, double map_resolution_data_from_subscription)
{
	map_resolution_ = map_resolution_data_from_subscription;

	//1. collect the navigation data from analyze map action client
	temporary_map_to_get_the_contours_of_the_room_ = Original_Map_from_subscription.clone();

	new_map_to_draw_the_saved_contours_of_the_room_ = Original_Map_from_subscription.clone();

	//2. erode the image until get the last room
	for (int for_loop_counter = 0; for_loop_counter < map_sampling_factor_ / map_resolution_; for_loop_counter++)
	{
		cv::erode(temporary_map_to_get_the_contours_of_the_room_, expanded_map_, cv::Mat(), cv::Point(-1, -1), 1);
		temporary_map_to_get_the_contours_of_the_room_ = expanded_map_;
		contour_map_ = expanded_map_.clone();

		//*********************** Find contours and Draw Contours*******************

		//3. find contours
		cv::findContours(contour_map_, temporary_contours_, hierarchy_, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
		cv::drawContours(contour_map_, temporary_contours_, -1, cv::Scalar(128, 128, 128, 128), 2);

		for (int idx = 0; idx >= 0; idx = hierarchy_[idx][0])
		{
			room_area_ = map_resolution_ * map_resolution_ * cv::contourArea(temporary_contours_[idx]);

			//4. for each contour check if contour fulfills criteria of a room
			if (room_area_factor_lower_limit_ < room_area_ && room_area_ < room_area_factor_upper_limit_ && temporary_contours_.size() != 0)
			{
				//5. if contour fulfill the criteria save the contour somewhere else
				saved_contours_.push_back(temporary_contours_[idx]);

				//************Remove the Saved Contour or make the region Black************
				cv::drawContours(temporary_map_to_get_the_contours_of_the_room_, temporary_contours_, idx, cv::Scalar(0), CV_FILLED, 8, hierarchy_, 2);
				//************Remove the Saved Contour or make the region Black************
			}
		}
	} //6. exit the loop if there are no more contours

	//**********************Draw the saved Contours in the clone version of original image***************

	for (unsigned int idx = 0; idx < saved_contours_.size(); idx++)
	{
		cv::Scalar color_to_fill(rand() % 252 + 2);
		cv::drawContours(new_map_to_draw_the_saved_contours_of_the_room_, saved_contours_, idx, color_to_fill, -1);
	}

	//**********************Draw the saved Contours in the clone version of original image***************

	//*********** To draw the Obstacle in the modified map*********************

	new_map_with_obstacles_info_ = new_map_to_draw_the_saved_contours_of_the_room_.clone();

	for (int y_coordinate = 0; y_coordinate < Original_Map_from_subscription.cols; y_coordinate++)
	{
		for (int x_coordinate = 0; x_coordinate < Original_Map_from_subscription.rows; x_coordinate++)
		{
			if (Original_Map_from_subscription.at<unsigned char>(x_coordinate, y_coordinate) == 0)
			{
				black_pixel_.push_back(cv::Point(y_coordinate, x_coordinate));
			}
		}
	}

	for (unsigned int idx = 0; idx < black_pixel_.size(); idx++)
	{
		new_map_with_obstacles_info_.at<unsigned char>(black_pixel_[idx]) = 0;
	}

	//*********** To draw the Obstacle in the modified map*********************

	//************Replica-Padding to the Image region*************

	//7. copy original image
	complete_map_after_contour_extraction_and_labelling = new_map_with_obstacles_info_.clone();
	int map_column_length, map_row_length = 0;

	temporary_map_for_replica_padding_purpose = new_map_with_obstacles_info_.clone();

	// 8. copy the stored contours into the map and fill each with a unique
	//    id number repeat until convergence(i.e there are no more white pixels)
	for (int loop_counter = 0; loop_counter < 1000; loop_counter++)
	{
		for (map_column_length = 0; map_column_length < Original_Map_from_subscription.cols; map_column_length++)
		{
			for (map_row_length = 0; map_row_length < Original_Map_from_subscription.rows; map_row_length++)
			{
				if (complete_map_after_contour_extraction_and_labelling.at<unsigned char>(map_row_length, map_column_length) != 0
						&& complete_map_after_contour_extraction_and_labelling.at<unsigned char>(map_row_length, map_column_length) != 255)
				{
					//Check every Pixel where its neighborhood is already replaced or not
					if (complete_map_after_contour_extraction_and_labelling.at<unsigned char>(map_row_length - 1, map_column_length - 1) == 255
							&& temporary_map_for_replica_padding_purpose.at<unsigned char>(map_row_length - 1, map_column_length - 1)
									!= complete_map_after_contour_extraction_and_labelling.at<unsigned char>(map_row_length, map_column_length))
					{
						neighbourhood_pixel_.push_back(cv::Point(map_column_length - 1, map_row_length - 1));
					}

					if (complete_map_after_contour_extraction_and_labelling.at<unsigned char>(map_row_length - 1, map_column_length) == 255
							&& temporary_map_for_replica_padding_purpose.at<unsigned char>(map_row_length - 1, map_column_length)
									!= complete_map_after_contour_extraction_and_labelling.at<unsigned char>(map_row_length, map_column_length))
					{
						neighbourhood_pixel_.push_back(cv::Point(map_column_length, map_row_length - 1));
					}

					if (complete_map_after_contour_extraction_and_labelling.at<unsigned char>(map_row_length - 1, map_column_length + 1) == 255
							&& temporary_map_for_replica_padding_purpose.at<unsigned char>(map_row_length - 1, map_column_length + 1)
									!= complete_map_after_contour_extraction_and_labelling.at<unsigned char>(map_row_length, map_column_length))
					{
						neighbourhood_pixel_.push_back(cv::Point(map_column_length + 1, map_row_length - 1));
					}

					if (complete_map_after_contour_extraction_and_labelling.at<unsigned char>(map_row_length, map_column_length - 1) == 255
							&& temporary_map_for_replica_padding_purpose.at<unsigned char>(map_row_length, map_column_length - 1)
									!= complete_map_after_contour_extraction_and_labelling.at<unsigned char>(map_row_length, map_column_length))
					{
						neighbourhood_pixel_.push_back(cv::Point(map_column_length - 1, map_row_length));
					}

					if (complete_map_after_contour_extraction_and_labelling.at<unsigned char>(map_row_length, map_column_length + 1) == 255
							&& temporary_map_for_replica_padding_purpose.at<unsigned char>(map_row_length, map_column_length + 1)
									!= complete_map_after_contour_extraction_and_labelling.at<unsigned char>(map_row_length, map_column_length))
					{
						neighbourhood_pixel_.push_back(cv::Point(map_column_length + 1, map_row_length));
					}

					if (complete_map_after_contour_extraction_and_labelling.at<unsigned char>(map_row_length + 1, map_column_length - 1) == 255
							&& temporary_map_for_replica_padding_purpose.at<unsigned char>(map_row_length + 1, map_column_length - 1)
									!= complete_map_after_contour_extraction_and_labelling.at<unsigned char>(map_row_length, map_column_length))
					{
						neighbourhood_pixel_.push_back(cv::Point(map_column_length - 1, map_row_length + 1));
					}

					if (complete_map_after_contour_extraction_and_labelling.at<unsigned char>(map_row_length + 1, map_column_length) == 255
							&& temporary_map_for_replica_padding_purpose.at<unsigned char>(map_row_length + 1, map_column_length)
									!= complete_map_after_contour_extraction_and_labelling.at<unsigned char>(map_row_length, map_column_length))
					{
						neighbourhood_pixel_.push_back(cv::Point(map_column_length, map_row_length + 1));
					}

					if (complete_map_after_contour_extraction_and_labelling.at<unsigned char>(map_row_length + 1, map_column_length + 1) == 255
							&& temporary_map_for_replica_padding_purpose.at<unsigned char>(map_row_length + 1, map_column_length + 1)
									!= complete_map_after_contour_extraction_and_labelling.at<unsigned char>(map_row_length, map_column_length))
					{
						neighbourhood_pixel_.push_back(cv::Point(map_column_length + 1, map_row_length + 1));
					}
				}

				for (unsigned int idx = 0; idx < neighbourhood_pixel_.size(); idx++)
				{
					if (neighbourhood_pixel_.size() != 0)
						temporary_map_for_replica_padding_purpose.at<unsigned char>(neighbourhood_pixel_[idx]) = complete_map_after_contour_extraction_and_labelling.at<
								unsigned char>(map_row_length, map_column_length);
				}
				neighbourhood_pixel_.clear();
			}

		}
		//Go for Next Check where the Replaced Pixel are now set as original
		complete_map_after_contour_extraction_and_labelling = temporary_map_for_replica_padding_purpose.clone();
	}

	//************Replica-Padding to the Image region*************

	//************Bounding Box**********************************

	bounding_box_map_to_extract_room_info = temporary_map_for_replica_padding_purpose.clone();
	cv::Point point_at_the_min_end_of_xy_coordinate, point_at_the_max_end_of_xy_coordinate, center_of_the_individual_room;

	std::vector<int> min_y_value_of_the_room(255, 100000000);
	std::vector<int> max_y_value_of_the_room(255, 0);
	std::vector<int> min_x_value_of_the_room(255, 100000000);
	std::vector<int> max_x_value_of_the_room(255, 0);

	for (map_column_length = 0; map_column_length < Original_Map_from_subscription.cols; map_column_length++)
	{
		for (map_row_length = 0; map_row_length < Original_Map_from_subscription.rows; map_row_length++)
		{
			if (bounding_box_map_to_extract_room_info.at<unsigned char>(map_row_length, map_column_length) != 0
					&& bounding_box_map_to_extract_room_info.at<unsigned char>(map_row_length, map_column_length) != 255)
			{
				min_y_value_of_the_room[temporary_map_for_replica_padding_purpose.at<unsigned char>(map_row_length, map_column_length)] = std::min(map_column_length,
						min_y_value_of_the_room[temporary_map_for_replica_padding_purpose.at<unsigned char>(map_row_length, map_column_length)]);
				max_y_value_of_the_room[temporary_map_for_replica_padding_purpose.at<unsigned char>(map_row_length, map_column_length)] = std::max(map_column_length,
						max_y_value_of_the_room[temporary_map_for_replica_padding_purpose.at<unsigned char>(map_row_length, map_column_length)]);
				min_x_value_of_the_room[temporary_map_for_replica_padding_purpose.at<unsigned char>(map_row_length, map_column_length)] = std::min(map_row_length,
						min_x_value_of_the_room[temporary_map_for_replica_padding_purpose.at<unsigned char>(map_row_length, map_column_length)]);
				max_x_value_of_the_room[temporary_map_for_replica_padding_purpose.at<unsigned char>(map_row_length, map_column_length)] = std::max(map_row_length,
						max_x_value_of_the_room[temporary_map_for_replica_padding_purpose.at<unsigned char>(map_row_length, map_column_length)]);

			}
		}
	}

	for (unsigned int a = 0; a < min_y_value_of_the_room.size(); a++)
	{
		if (min_y_value_of_the_room[a] != 100000000 && min_x_value_of_the_room[a] != 100000000 && max_y_value_of_the_room[a] != 0 && max_x_value_of_the_room[a] != 0)
		{
			point_at_the_min_end_of_xy_coordinate.x = min_y_value_of_the_room[a];
			point_at_the_min_end_of_xy_coordinate.y = min_x_value_of_the_room[a];
			point_at_the_max_end_of_xy_coordinate.x = max_y_value_of_the_room[a];
			point_at_the_max_end_of_xy_coordinate.y = max_x_value_of_the_room[a];

			center_of_the_individual_room.x = min_y_value_of_the_room[a] + (max_y_value_of_the_room[a] - min_y_value_of_the_room[a]) / 2;
			center_of_the_individual_room.y = min_x_value_of_the_room[a] + (max_x_value_of_the_room[a] - min_x_value_of_the_room[a]) / 2;

			std::cout << "\nCenter of the bounding Box: [ " << center_of_the_individual_room.x << " , " << center_of_the_individual_room.y << " ]\n";

			minimum_x_coordinate_value_of_the_room_.push_back(min_x_value_of_the_room[a]);
			minimum_y_coordinate_value_of_the_room_.push_back(min_y_value_of_the_room[a]);
			maximum_x_coordinate_value_of_the_room_.push_back(max_x_value_of_the_room[a]);
			maximum_y_coordinate_value_of_the_room_.push_back(max_y_value_of_the_room[a]);

			center_of_room_.push_back(center_of_the_individual_room);
			x_coordinate_value_of_the_room_center_.push_back(center_of_the_individual_room.x);
			y_coordinate_value_of_the_room_center_.push_back(center_of_the_individual_room.y);

			cv::rectangle(bounding_box_map_to_extract_room_info, point_at_the_min_end_of_xy_coordinate, point_at_the_max_end_of_xy_coordinate, cv::Scalar(255), 1);
			cv::circle(bounding_box_map_to_extract_room_info, center_of_the_individual_room, 3, cv::Scalar(255), -1);
		}
	}

	cv::imshow("bounding box", bounding_box_map_to_extract_room_info);
	cv::waitKey(100);

	//************Bounding Box**********************************

	return temporary_map_for_replica_padding_purpose.clone();
}

void segmentation_algorithm::execute_map_segmentation_server(const autopnp_scenario::AnalyzeMapGoalConstPtr &goal)
{
	ros::Rate r(1);

	cv_bridge::CvImagePtr cv_ptr;
	cv_ptr = cv_bridge::toCvCopy(goal->input_img, sensor_msgs::image_encodings::MONO8);
	cv::Mat original_img;
	original_img = cv_ptr->image;

	cv::Mat Segmented_map;
	Segmented_map = Image_Segmentation_method(original_img, goal->map_resolution);

	r.sleep();

	//Publish Result message:
	cv_bridge::CvImage cv_image;
	cv_image.header.stamp = ros::Time::now();
	cv_image.encoding = "mono8";
	cv_image.image = Segmented_map;
	cv_image.toImageMsg(action_result_.output_img);

	action_result_.room_center_x = x_coordinate_value_of_the_room_center_;
	action_result_.room_center_y = y_coordinate_value_of_the_room_center_;
	action_result_.map_resolution = goal->map_resolution;
	action_result_.Map_Origin_x = goal->Map_Origin_x;
	action_result_.Map_Origin_y = goal->Map_Origin_y;
	action_result_.room_min_x = minimum_x_coordinate_value_of_the_room_;
	action_result_.room_min_y = minimum_y_coordinate_value_of_the_room_;
	action_result_.room_max_x = maximum_x_coordinate_value_of_the_room_;
	action_result_.room_max_y = maximum_y_coordinate_value_of_the_room_;

	analyze_map_action_server_variable_redefinition_.setSucceeded(action_result_);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "analyze_map");

	segmentation_algorithm Segmentation_Algorithm_obj(ros::this_node::getName());

	ros::spin();

	return 0;
}


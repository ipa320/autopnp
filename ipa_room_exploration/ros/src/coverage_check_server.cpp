#include <ipa_room_exploration/coverage_check_server.h>

// The default constructors
CoverageCheckServer::CoverageCheckServer()
{
}

CoverageCheckServer::CoverageCheckServer(ros::NodeHandle nh)
:node_handle_(nh)
{
	coverage_check_server_ = node_handle_.advertiseService("coverage_check", &CoverageCheckServer::checkCoverage, this);
	ROS_INFO("Server for coverage checking initialized.....");
}

// Callback function for the server.
bool CoverageCheckServer::checkCoverage(ipa_building_msgs::CheckCoverageRequest& request, ipa_building_msgs::CheckCoverageResponse& response)
{
	// convert the map msg to cv format
	cv_bridge::CvImagePtr cv_ptr_obj;
	cv_ptr_obj = cv_bridge::toCvCopy(request.input_map, sensor_msgs::image_encodings::MONO8);
	cv::Mat map = cv_ptr_obj->image;

	// convert field of view to Eigen format
	std::vector<Eigen::Matrix<float, 2, 1> > field_of_view;
	for(size_t i = 0; i < request.field_of_view.size(); ++i)
	{
		Eigen::Matrix<float, 2, 1> current_vector;
		current_vector << request.field_of_view[i].x, request.field_of_view[i].y;
		field_of_view.push_back(current_vector);
	}

	// convert path to cv format
	std::vector<cv::Point3d> path;
	for (size_t i=0; i<request.path.size(); ++i)
		path.push_back(cv::Point3d(request.path[i].x, request.path[i].y, request.path[i].theta));

	cv::Mat coverage_map, number_of_coverage_image;
	bool return_value = checkCoverage(map, request.map_resolution, cv::Point2d(request.map_origin.position.x, request.map_origin.position.y), path,
			field_of_view, request.coverage_radius, request.check_for_footprint, request.check_number_of_coverages,
			coverage_map, number_of_coverage_image);

	// convert the map with the covered area back to the sensor_msgs format
	ros::Time now = ros::Time::now();
	cv_bridge::CvImage cv_image;
	cv_image.header.stamp = now;
	cv_image.encoding = "mono8";
	cv_image.image = coverage_map;
	cv_image.toImageMsg(response.coverage_map);

	// if needed, return the image with number of coverages drawn in
	if(request.check_number_of_coverages==true)
	{
		cv_bridge::CvImage number_image;
		number_image.header.stamp = now;
		number_image.encoding = "32SC1";
		number_image.image = number_of_coverage_image;
		number_image.toImageMsg(response.number_of_coverage_image);
	}

	return return_value;
}
bool CoverageCheckServer::checkCoverage(const cv::Mat& map, const float map_resolution, const cv::Point2d& map_origin, const std::vector<cv::Point3d>& path,
		const std::vector<Eigen::Matrix<float, 2, 1> >& field_of_view, const float coverage_radius, const bool check_for_footprint, const bool check_number_of_coverages,
		cv::Mat& coverage_map, cv::Mat& number_of_coverage_image)
{
	// When checking for the field of view, find two points behind the end of the fov to get raycasting goals. These two points span
	// a line behind the fov, that provides the raycasting goals. By designing it this way, it is guaranteed to cover the whole fov with
	// this procedure. The raycasting allows to check if the view was blocked by an obstacle and thus not the whole given fov-polygon has
	// to be drawn into the map.
	//		Get points that define the edge-points of the line the raycasting should go to, by computing the intersection of two
	//		lines: the line defined by the robot pose and the fov-point that spans the highest angle and a line parallel to the
	//		front side of the fov with an offset.
	Eigen::Matrix<float, 2, 1> corner_point_1, corner_point_2;
	if(check_for_footprint==false)
	{
		// todo: replace this definition by a circle around the robot center with radius=largest fov point distance, define visibility sectors on that circle given by the fov points
		// to increase the general applicability to arbitrary fov definitions

		// get angles between robot_pose and fov-corners in relative coordinates to find the edge that spans the largest angle with
		// the robot-center --> the raycasting goals at least have to cover this angle
		float dot = field_of_view[0].transpose()*field_of_view[1];
		float abs = field_of_view[0].norm()*field_of_view[1].norm();
		float quotient = dot/abs;
		if(quotient > 1) // prevent errors resulting from round errors
			quotient = 1;
		else if(quotient < -1)
			quotient = -1;
		float angle_1 = std::acos(quotient);
		dot = field_of_view[2].transpose()*field_of_view[3];
		abs = field_of_view[2].norm()*field_of_view[3].norm();
		quotient = dot/abs;
		if(quotient > 1) // prevent errors resulting from round errors
			quotient = 1;
		else if(quotient < -1)
			quotient = -1;
		float angle_2 = std::acos(dot/abs);

		if(angle_1 > angle_2) // do a line crossing s.t. the corners are guaranteed to be after the fov
		{
			float border_distance = 7;
			Eigen::Matrix<float, 2, 1> pose_to_fov_edge_vector_1 = field_of_view[0];
			Eigen::Matrix<float, 2, 1> pose_to_fov_edge_vector_2 = field_of_view[1];

			// get vectors showing the directions for for the lines from pose to edge of fov
			Eigen::Matrix<float, 2, 1> normed_fov_vector_1 = field_of_view[0]/field_of_view[0].norm();
			Eigen::Matrix<float, 2, 1> normed_fov_vector_2 = field_of_view[1]/field_of_view[1].norm();

			// get the offset point after the end of the fov
			Eigen::Matrix<float, 2, 1> offset_point_after_fov = field_of_view[2];
			offset_point_after_fov(1, 0) = offset_point_after_fov(1, 0) + border_distance;

			// find the parameters for the two different intersections (for each corner point)
			float first_edge_parameter = (pose_to_fov_edge_vector_1(1, 0)/pose_to_fov_edge_vector_1(0, 0) * (field_of_view[0](0, 0) - offset_point_after_fov(0, 0)) + offset_point_after_fov(1, 0) - field_of_view[0](1, 0))/( pose_to_fov_edge_vector_1(1, 0)/pose_to_fov_edge_vector_1(0, 0) * (field_of_view[3](0, 0) - field_of_view[2](0, 0)) - (field_of_view[3](1, 0) - field_of_view[2](1, 0)) );
			float second_edge_parameter = (pose_to_fov_edge_vector_2(1, 0)/pose_to_fov_edge_vector_2(0, 0) * (field_of_view[1](0, 0) - offset_point_after_fov(0, 0)) + offset_point_after_fov(1, 0) - field_of_view[1](1, 0))/( pose_to_fov_edge_vector_2(1, 0)/pose_to_fov_edge_vector_2(0, 0) * (field_of_view[3](0, 0) - field_of_view[2](0, 0)) - (field_of_view[3](1, 0) - field_of_view[2](1, 0)) );

			// use the line equation and found parameters to actually find the corners
			corner_point_1 = first_edge_parameter * (field_of_view[3] - field_of_view[2]) + offset_point_after_fov;
			corner_point_2 = second_edge_parameter * (field_of_view[3] - field_of_view[2]) + offset_point_after_fov;
		}
		else
		{
			// follow the lines to the farthest points and go a little longer, this ensures that the whole fov is covered
			corner_point_1 = 1.3 * field_of_view[2];
			corner_point_2 = 1.3 * field_of_view[3];
		}
	}

	// create a map that stores the number of coverages during the execution, if wanted
	cv::Mat* image_pointer = NULL;
	if(check_number_of_coverages==true)
	{
		number_of_coverage_image = cv::Mat::zeros(map.rows, map.cols, CV_32SC1);
		image_pointer = &number_of_coverage_image;
		ROS_INFO("Checking number of coverages.");
	}

	// check if the coverage check should be done for the footprint or the field of view
	coverage_map = map.clone();
	if(check_for_footprint==false)
	{
		ROS_INFO("Checking coverage for fov.");
		drawSeenPoints(coverage_map, path, field_of_view, corner_point_1, corner_point_2, map_resolution, map_origin, image_pointer);
	}
	else
	{
		ROS_INFO("Checking coverage for footprint.");
		drawSeenPoints(coverage_map, path, coverage_radius, map_resolution, map_origin, image_pointer);
	}
	ROS_INFO("Finished coverage check.");

	return true;
}


// Function to draw the seen points into the given map, that shows the positions the robot can actually reach. This is done by
// going trough all given robot-poses and calculate where the field of view has been. The field of view is given in the relative
// not rotated case, meaning to be in the robot-frame, where x_robot shows into the direction of the front and the y_robot axis
// along its left side. The function then calculates the field_of_view_points in the global frame by using the given robot pose.
// After this the function does a raycasting to check if the field of view has been blocked by an obstacle and couldn't see
// what's behind it. This ensures that no Point is wrongly classified as seen.
void CoverageCheckServer::drawSeenPoints(cv::Mat& reachable_areas_map, const std::vector<cv::Point3d>& robot_poses,
			const std::vector<Eigen::Matrix<float, 2, 1> >& field_of_view_points, const Eigen::Matrix<float, 2, 1> raycasting_corner_1,
			const Eigen::Matrix<float, 2, 1> raycasting_corner_2, const float map_resolution, const cv::Point2d map_origin,
			cv::Mat* number_of_coverages_image)
{
	const float map_resolution_inverse = 1./map_resolution;

	// go trough each given robot pose
	for(std::vector<cv::Point3d>::const_iterator current_pose = robot_poses.begin(); current_pose != robot_poses.end(); ++current_pose)
	{
		// get the rotation matrix
		float sin_theta = std::sin(current_pose->z);
		float cos_theta = std::cos(current_pose->z);
		Eigen::Matrix<float, 2, 2> R;
		R << cos_theta, -sin_theta, sin_theta, cos_theta;

		// transform field of view points
		std::vector<cv::Point> transformed_fov_points;
		Eigen::Matrix<float, 2, 1> pose_as_matrix;
		pose_as_matrix << current_pose->x, current_pose->y;
		for(size_t point = 0; point < field_of_view_points.size(); ++point)
		{
			// linear transformation
			Eigen::Matrix<float, 2, 1> transformed_fov_point = pose_as_matrix + R * field_of_view_points[point];

			// save the transformed point as cv::Point, also check if map borders are satisfied and transform it into pixel
			// values
			cv::Point current_point = cv::Point((transformed_fov_point(0, 0) - map_origin.x)*map_resolution_inverse, (transformed_fov_point(1, 0) - map_origin.y)*map_resolution_inverse);
			current_point.x = std::max(current_point.x, 0);
			current_point.y = std::max(current_point.y, 0);
			current_point.x = std::min(current_point.x, reachable_areas_map.cols);
			current_point.y = std::min(current_point.y, reachable_areas_map.rows);
			transformed_fov_points.push_back(current_point);
//			std::cout << current_point << std::endl;
		}
//		std::cout << std::endl;

		// transform corners for raycasting
		Eigen::Matrix<float, 2, 1> transformed_corner_1 = pose_as_matrix + R * raycasting_corner_1;
		Eigen::Matrix<float, 2, 1> transformed_corner_2 = pose_as_matrix + R * raycasting_corner_2;

		// convert to openCV format
		cv::Point transformed_corner_cv_1 = cv::Point((transformed_corner_1(0, 0) - map_origin.x)*map_resolution_inverse, (transformed_corner_1(1, 0) - map_origin.y)*map_resolution_inverse);
		transformed_corner_cv_1.x = std::max(transformed_corner_cv_1.x, 0);
		transformed_corner_cv_1.y = std::max(transformed_corner_cv_1.y, 0);
		transformed_corner_cv_1.x = std::min(transformed_corner_cv_1.x, reachable_areas_map.cols);
		transformed_corner_cv_1.y = std::min(transformed_corner_cv_1.y, reachable_areas_map.rows);
		cv::Point transformed_corner_cv_2 = cv::Point((transformed_corner_2(0, 0) - map_origin.x)*map_resolution_inverse, (transformed_corner_2(1, 0) - map_origin.y)*map_resolution_inverse);
		transformed_corner_cv_2.x = std::max(transformed_corner_cv_2.x, 0);
		transformed_corner_cv_2.y = std::max(transformed_corner_cv_2.y, 0);
		transformed_corner_cv_2.x = std::min(transformed_corner_cv_2.x, reachable_areas_map.cols);
		transformed_corner_cv_2.y = std::min(transformed_corner_cv_2.y, reachable_areas_map.rows);

		// raycast the field of view to look what areas actually have been seen
		// get points between the edge-points to get goals for raycasting
		cv::LineIterator border_line(reachable_areas_map, transformed_corner_cv_1, transformed_corner_cv_2, 8); // opencv implementation of bresenham algorithm, 8: color, irrelevant
		std::vector<cv::Point> raycasting_goals(border_line.count);

		for(size_t i = 0; i < border_line.count; i++, ++border_line)
			raycasting_goals[i] = border_line.pos();

		// transform pose into OpenCV format
		cv::Point pose_cv((current_pose->x - map_origin.x)*map_resolution_inverse, (current_pose->y - map_origin.y)*map_resolution_inverse);

		// go trough the found raycasting goals and draw the field-of-view
		for(std::vector<cv::Point>::iterator goal = raycasting_goals.begin(); goal != raycasting_goals.end(); ++goal)
		{
			// use openCVs bresenham algorithm to find the points from the robot pose to the goal
			cv::LineIterator ray_points(reachable_areas_map, pose_cv , *goal, 8);

			// go trough the points on the ray and draw them if they are inside the fov, stop the current for-step when a black
			// pixel is hit (an obstacle stops the camera from seeing whats behind)
			bool hit_white = false;
			for(size_t point = 0; point < ray_points.count; point++, ++ray_points)
			{
				cv::Point current_point = ray_points.pos();

				// stop raycasting, when a black pixel gets hit after at least one white pixel was found
				if(reachable_areas_map.at<uchar>(current_point) == 0 && hit_white == true)
				{
					break;
				}
				else if (reachable_areas_map.at<uchar>(current_point) > 0 && cv::pointPolygonTest(transformed_fov_points, current_point, false) >= 0)
				{
					reachable_areas_map.at<uchar>(current_point) = 127;

					// mark that at least one white pixel was found
					hit_white = true;

					// if wanted, count the coverage
					if(number_of_coverages_image!=NULL)
					{
						number_of_coverages_image->at<int>(current_point) = number_of_coverages_image->at<int>(current_point)+1;
					}
				}
			}
		}
	}
}

// Function that takes the given robot poses and draws the footprint at these positions into the given map. Used when
// the server should plan a coverage path for the robot coverage area (a circle).
void CoverageCheckServer::drawSeenPoints(cv::Mat& reachable_areas_map, const std::vector<cv::Point3d>& robot_poses,
			const double coverage_radius, const float map_resolution,
			const cv::Point2d map_origin, cv::Mat* number_of_coverages_image)
{
	const float map_resolution_inverse = 1./map_resolution;

	cv::Mat map_copy = reachable_areas_map.clone(); // copy to draw positions that get later drawn into free space of the original map
	const int coverage_radius_pixel = coverage_radius*map_resolution_inverse;
	// iterate trough all poses and draw them into the given map
	for(std::vector<cv::Point3d>::const_iterator pose=robot_poses.begin(); pose!=robot_poses.end(); ++pose)
	{
//		// get the rotation matrix
//		float sin_theta = std::sin(pose->theta);
//		float cos_theta = std::cos(pose->theta);
//		Eigen::Matrix<float, 2, 2> R;
//		R << cos_theta, -sin_theta, sin_theta, cos_theta;

//		// transform footprint points
//		std::vector<cv::Point> transformed_footprint_points;
//		Eigen::Matrix<float, 2, 1> pose_as_matrix;
//		pose_as_matrix << pose->x, pose->y;
//		for(size_t point = 0; point < robot_footprint.size(); ++point)
//		{
//			// transform fov-point from geometry_msgs::Point32 to Eigen::Matrix
//			Eigen::Matrix<float, 2, 1> footprint_point;
//			footprint_point << robot_footprint[point].x, robot_footprint[point].y;
//
//			// linear transformation
//			Eigen::Matrix<float, 2, 1> transformed_vector = pose_as_matrix + R * footprint_point;
//
//			// save the transformed point as cv::Point, also check if map borders are satisfied and transform it into pixel
//			// values
//			cv::Point current_point = cv::Point((transformed_vector(0, 0) - map_origin.x)*map_resolution_inverse, (transformed_vector(1, 0) - map_origin.y)*map_resolution_inverse);
//			current_point.x = std::max(current_point.x, 0);
//			current_point.y = std::max(current_point.y, 0);
//			current_point.x = std::min(current_point.x, map_copy.cols);
//			current_point.y = std::min(current_point.y, map_copy.rows);
//			transformed_footprint_points.push_back(current_point);
//		}

		// draw the transformed robot footprint
		cv::Point current_point((pose->x-map_origin.x)*map_resolution_inverse, (pose->y-map_origin.y)*map_resolution_inverse);
		cv::circle(map_copy, current_point, coverage_radius_pixel, cv::Scalar(127), -1);
//		cv::fillConvexPoly(map_copy, transformed_footprint_points, cv::Scalar(127));

		// update the number of visits at this location, if wanted
		if(number_of_coverages_image!=NULL)
		{
			cv::Mat coverage_area = cv::Mat::zeros(map_copy.rows, map_copy.cols, CV_32SC1);
			cv::circle(coverage_area, current_point, coverage_radius_pixel, cv::Scalar(1), -1);
			*number_of_coverages_image = *number_of_coverages_image + coverage_area;
//			int current_value = number_of_coverages_image->at<int>(cv::Point((pose->x-map_origin.x)*map_resolution_inverse, (pose->y-map_origin.y)*map_resolution_inverse));
//			cv::fillConvexPoly(*number_of_coverages_image, transformed_footprint_points, cv::Scalar(current_value+1));
		}
	}

	// draw visited areas into free space of the original map
	for(size_t y=0; y<map_copy.rows; ++y)
		for(size_t x=0; x<map_copy.cols; ++x)
			if(reachable_areas_map.at<uchar>(y, x) == 255)
				reachable_areas_map.at<uchar>(y, x) = map_copy.at<uchar>(y, x);
}

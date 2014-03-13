/*
 * arm_fiducial_calibration.cpp
 *
 *  Created on: Mar 13, 2014
 *      Author: rmb-om
 */

// ROS includes
#include <ros/ros.h>
//#include <ros/package.h>
#include <tf/tf.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
// ROS message includes
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/JointState.h>
#include <cob_object_detection_msgs/DetectionArray.h>
#include <cob_object_detection_msgs/Detection.h>

#include <message_filters/subscriber.h>
#include <autopnp_tool_change/ArmFiducialCalibrationAction.h>
#include <visualization_msgs/Marker.h>
#include <moveit/move_group_interface/move_group.h>
#include <math.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/JointState.h>
#include <pr2_controllers_msgs/JointTrajectoryControllerState.h>
#include <moveit_msgs/ExecuteKnownTrajectory.h>
#include <vector>
#include <boost/bind.hpp>

static const std::string ARM = "tag_2";
static const std::string CAM = "tag_3";
static const std::string PLANNING_GROUP_NAME = "arm";
static const std::string BASE_LINK = "base_link";
static const std::string EE_NAME = "arm_7_link";

class ArmFiducialCalibration
{
public:

	/// array of two transform msgs
	struct fiducials;
	struct fiducials
	{
		tf::Transform translation;
	};
	/// array of two fiducial objects
	struct components;
	struct components
	{
		struct fiducials arm;
		struct fiducials cam;
	};

	bool detected_all_fiducials_;
	bool calibration_detected_;
	bool pose_detected_;

	tf::Transform fiducial_;
	tf::Transform cam_;

	actionlib::SimpleActionServer<autopnp_tool_change::ArmFiducialCalibrationAction> arm_fiducial_calibration_server;
	message_filters::Subscriber<cob_object_detection_msgs::DetectionArray> input_marker_detection_sub_;
	ros::Publisher vis_pub_;
	ros::NodeHandle node_handle_;
	autopnp_tool_change::ArmFiducialCalibrationFeedback feedback_;
	autopnp_tool_change::ArmFiducialCalibrationResult result_;



	ArmFiducialCalibration(ros::NodeHandle nh) :arm_fiducial_calibration_server
			(nh, "arm_fiducial_calibration", boost::bind(&ArmFiducialCalibration::calibrate, this, _1), false)
	{
		node_handle_ = nh;
		calibration_detected_ = false;
		pose_detected_ = false;
		//vis_pub_ = node_handle_.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
		input_marker_detection_sub_.subscribe(node_handle_, "/fiducials/detect_fiducials", 1);
		input_marker_detection_sub_.registerCallback(boost::bind(&ArmFiducialCalibration::callback, this, _1));

	}

	tf::Transform getEePose()
	{
		geometry_msgs::PoseStamped ee_pose;
		tf::Transform ee_pose_tf;

		moveit::planning_interface::MoveGroup group(PLANNING_GROUP_NAME);
		group.setPoseReferenceFrame(BASE_LINK);
		//get the position of the end effector (= arm_7_joint)
		ee_pose.pose = group.getCurrentPose(EE_NAME).pose;
		tf::poseMsgToTF(ee_pose.pose, ee_pose_tf);
		return ee_pose_tf;
	}

	void calibrate(const autopnp_tool_change::ArmFiducialCalibrationGoalConstPtr& goal) {
		ROS_INFO("calibrate");
		geometry_msgs::PoseStamped result_pose;
		result_pose.pose = computeTransform().pose;
		computeTransform();
		// this command sends a feedback message to the caller
		autopnp_tool_change::ArmFiducialCalibrationFeedback feedback;
		feedback.result.pose = result_pose.pose;
		arm_fiducial_calibration_server.publishFeedback(feedback);
		// this sends the response back to the caller
		autopnp_tool_change::ArmFiducialCalibrationResult res;
		res.feedback.pose = result_pose.pose;
		arm_fiducial_calibration_server.setSucceeded(res);
	}

	geometry_msgs::PoseStamped computeTransform() {
		tf::Transform ee_pose_tf;

		ee_pose_tf = getEePose();

		tf::Quaternion q;
		q.setRPY(0.0, 0.0, -M_PI);
		tf::Transform transform_CA_FA;
		transform_CA_FA.setOrigin(fiducial_.getOrigin());
		transform_CA_FA.setRotation(fiducial_.getRotation());
		tf::Transform transform_CA_EE;
		tf::Transform transform_EE_FA;
		tf::Transform transform_BA_FA;
		tf::Transform transform_BA_FB;
		tf::Transform transform_FA_FB;
		tf::Transform transform_EE_FB;
		tf::Transform transform_CA_BA;
		tf::Transform transform_EE_GO;

		transform_CA_BA.setOrigin(cam_.getOrigin());
		transform_CA_BA.setRotation(cam_.getRotation() * q);

		//base-cam-fiducial A
		transform_BA_FA.mult(transform_CA_BA.inverse(), transform_CA_FA);

   tf::Transform result;
   geometry_msgs::PoseStamped result_msg;
   result.mult(ee_pose_tf.inverse(), transform_BA_FA);
   tf::poseTFToMsg(result, result_msg.pose);

   return result_msg;


	}
	void callback(const cob_object_detection_msgs::DetectionArray::ConstPtr& input_marker_detections_msg)
	{
		struct components result_components;

		if (input_marker_detections_msg->detections.size() != 0 )
		{

			//set marker components if such detected
			result_components = computeMarkerPose(input_marker_detections_msg);

			if(detected_all_fiducials_ == true)
			{

				fiducial_ = result_components.arm.translation;
				cam_ = result_components.arm.translation;
				//printPose(fiducial);
				//printPose(cam);
			}
		}
	}

	/*
	 * Computes mean coordinate system if multiple markers detected
	 * and saves the data as an array of two fiducial objects
	 * {@value arm, @value board} with the transform data
	 * {@value translation} respectively.
	 */
	struct components computeMarkerPose(
			const cob_object_detection_msgs::DetectionArray::ConstPtr& input_marker_detections_msg)
	{
		components result;
		detected_all_fiducials_ = false;
		bool detected_arm_fiducial = false;
		bool detected_cam_fiducial = false;
		tf::Point translation;
		tf::Quaternion orientation;

		for (unsigned int i = 0; i < input_marker_detections_msg->detections.size(); ++i)
		{
			//retrieve the number of label and format the string message to an int number
			std::string fiducial_label = input_marker_detections_msg->detections[i].label;

			//convert translation and orientation Points msgs to Points respectively
			tf::pointMsgToTF(input_marker_detections_msg->detections[i].pose.pose.position, translation);
			tf::quaternionMsgToTF(input_marker_detections_msg->detections[i].pose.pose.orientation, orientation);


			//if(fiducial_label_num == arm_marker_number)
			if (fiducial_label.compare(ARM)==0)
			{
				detected_arm_fiducial = true;
				result.arm.translation.setOrigin(translation);
				result.arm.translation.setRotation(orientation);
				result.arm.translation.getRotation().normalize();
			}
			if (fiducial_label.compare(CAM)==0)
			{
				detected_cam_fiducial = true;
				result.cam.translation.setOrigin(translation);
				result.cam.translation.setRotation(orientation);
				result.cam.translation.getRotation().normalize();
			}
		}

		detected_all_fiducials_ = detected_arm_fiducial && detected_cam_fiducial;

		return result;
	}

	/**
	 * A helper function to print the results of
	 * a transform message.
	 */
	void printPose(tf::Transform& trans_msg)
	{

		ROS_INFO("translation [%f, %f, %f] , orientation [%f, %f, %f, %f]",
				(float) trans_msg.getOrigin().m_floats[0],
				(float) trans_msg.getOrigin().m_floats[1],
				(float) trans_msg.getOrigin().m_floats[2],

				(float) trans_msg.getRotation().getX(),
				(float) trans_msg.getRotation().getY(),
				(float) trans_msg.getRotation().getZ(),
				(float) trans_msg.getRotation().getW());

	}
	void init()
	{
		arm_fiducial_calibration_server.start();
	}
};



int main (int argc, char** argv)
{
	// Initialize ROS, specify name of node
	ros::init(argc, argv, "arm_fiducial_calibration_server");

	// Create a handle for this node, initialize node
	ros::NodeHandle nh;

	// Create and initialize an instance of Object
	ArmFiducialCalibration calibration(nh);
	calibration.init();

	ros::spin();

	return (0);
}


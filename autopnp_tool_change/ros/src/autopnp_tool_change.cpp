#include <autopnp_tool_change/autopnp_tool_change.h>
#include <math.h>
#include <tf/transform_broadcaster.h>
#include <moveit/move_group_interface/move_group.h>

/*
 * Initializing the server before start and waiting for data :
 * Starts all needed subscriptions and publications.
 * Makes the server sleep while the data in question
 * have not been received jet. The spinOnce operation
 * allows the calculation of the background processes
 * and sleeping functions to run simultaneously .
 */
ToolChange::ToolChange(ros::NodeHandle nh)
: change_tool_server_(nh, "tool_change", boost::bind(&ToolChange::changeTool, this, _1), false),
go_client_(nh, "go_to_start_position_client", true)
{
	node_handle_ = nh;
	std::cout << "Starting autopnp_tool_change server" << std::endl;

	//wait for pickup client
	    while(!go_client_.waitForServer(ros::Duration(2.0)) && nh.ok())
	    {
	      ROS_INFO_STREAM("Waiting for action client " );
	    }
	    if (!nh.ok()) exit(0);

	 //wait for detection client
	  while ( !ros::service::waitForService("go_to_start_position_server", ros::Duration(2.0)) && nh.ok() ) {
	    ROS_INFO("Waiting for service to come up");
	  }
	  if (!nh.ok()) exit(0);

	//  go_srv_ = nh.serviceClient<autopnp_tool_change::GoToStartPositionAction>("go_to_start_position_server", true);

}

/*
 * TO DO: set the garbage container free !!!!!!
 */
ToolChange::~ToolChange()
{
}

/*
 * Starting server after initialization process.
 */
void ToolChange::init()
{
	change_tool_server_.start();

}

/*
 * This callback function is executed each time a request (= goal message)
 * comes to this server (eventually this will be an initial
 * position message for the arm or a number of the tool
 * to be taken from the wagon). Starting move commands for the arm from here.
 *
 */
void ToolChange::changeTool(const autopnp_tool_change::MoveToWagonGoalConstPtr& goal) {

	ROS_INFO("EXECUTE changeTool");
   	autopnp_tool_change::MoveToWagonResult result;
   	autopnp_tool_change::GoToStartPositionGoal go_goal;
    go_goal.goal = "arm";
    go_client_.sendGoal(go_goal);
    while (!go_client_.waitForResult(ros::Duration(10.0)))
         {
           ROS_INFO("Waiting for the go action...");
         }
    autopnp_tool_change::GoToStartPositionResult go_result =
           *(go_client_.getResult());
         if (go_client_.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
         {
           ROS_ERROR("The go action has failed  ");
         }

    ROS_INFO("FINISHED changeTool");
	change_tool_server_.setSucceeded(result);
}

int main (int argc, char** argv)
{
	// Initialize ROS, specify name of node
	ros::init(argc, argv, "autopnp_tool_change_server");

	// Create a handle for this node, initialize node
	ros::NodeHandle nh;

	// Create and initialize an instance of Object
	ToolChange toolChange(nh);

	//toolChange.init();
	ros::spin();

	return (0);
}



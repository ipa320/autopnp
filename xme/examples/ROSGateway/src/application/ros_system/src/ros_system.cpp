#include <pthread.h>

#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"

#include "ros_system/AddTwoInts.h"
#include "ros_system/SubstractTwoInts.h"

static ros::Publisher status_pub;
static ros::ServiceClient add_client;

bool substractRequest(ros_system::SubstractTwoInts::Request &req, ros_system::SubstractTwoInts::Response &res)
{
  res.difference= req.a - req.b;

  ROS_INFO("Received request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
  ROS_INFO("Sending back response: %ld", (long int)res.difference);

  return true;
}

void messageCallback(const std_msgs::String::ConstPtr& msg)
{
  static bool lastValue = false;
  std_msgs::Bool statusMsg;
	
  ROS_INFO("Received message: %s", msg->data.c_str());

  lastValue = !lastValue;
  statusMsg.data = lastValue;

  ROS_INFO("Send status: %d", statusMsg.data);

  status_pub.publish(statusMsg);
}

static void* requestService(void* lpParam)
{
  while (ros::ok())
  {
    ros_system::AddTwoInts srv;
  
    srv.request.a = 1;
    srv.request.b = 5;
    ROS_INFO("Send request");
    if (add_client.call(srv))
    {
      ROS_INFO("Sum: %ld", (long int)srv.response.sum);
    }
    else
    {
      ROS_ERROR("Failed to call add_two_ints!");
      return NULL;
    }
  }

  return NULL;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ros_system");

  ros::NodeHandle nodeHandle;
  pthread_t rosThreadHandle;

  ros::Subscriber message_sub = nodeHandle.subscribe("message", 1000, messageCallback);
  status_pub = nodeHandle.advertise<std_msgs::Bool>("status", 1000);
  ros::ServiceServer difference_service = nodeHandle.advertiseService("substract_two_ints", substractRequest);
  
  ros::Rate loop_rate(10);

  sleep(10);
  add_client = nodeHandle.serviceClient<ros_system::AddTwoInts>("add_two_ints");
  if (pthread_create(&rosThreadHandle, NULL, requestService, NULL))
  {
    fprintf(stderr, "Error creating requestService\n");
    return -1;
  }

  while (ros::ok())
  {
    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}


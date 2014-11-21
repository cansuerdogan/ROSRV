#include "ros/ros.h"
#include "std_msgs/String.h"

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */

void chatter_cb(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");

  ros::NodeHandle n;
  std::string topic;
  n.getParam( "/chatter_topic", topic );
  ros::Subscriber sub = n.subscribe(topic, 1000, chatter_cb);

  ros::spin();

  return 0;
}

/**
 * @file listener.cpp
 * @author Hrushikesh Budhale (hbudhale@umd.edu)
 * @brief First subscriber node in cpp
 * @version 0.1
 * @date 2021-10-28
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "first_publisher_subscriber/modify_str.h"

int count = 1;
ros::ServiceClient client;

void chatterCallback(const std_msgs::String::ConstPtr& msg) {
  ROS_DEBUG_STREAM("[Listener] I heard: " << msg->data);

  count++;
  if (count%10 == 0) {
    first_publisher_subscriber::modify_str srv;
    srv.request.data = "Modified log message from listener";
    if (client.call(srv))
      ROS_INFO_STREAM("[Listener] Received response: "
                                                    << srv.response.new_data);
    else
      ROS_ERROR_STREAM("[Listener] Did not receive any response");
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "listener", ros::init_options::AnonymousName);
  ros::NodeHandle n;

  client = n.serviceClient
                    <first_publisher_subscriber::modify_str>("modify_output");

  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);

  ros::spin();

  return 0;
}

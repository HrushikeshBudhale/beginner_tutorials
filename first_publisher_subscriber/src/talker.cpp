/**
 * @file talker.cpp
 * @author Hrushikesh Budhale (hbudhale@umd.edu)
 * @brief First publisher in cpp
 * @version 0.1
 * @date 2021-10-28
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <sstream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "first_publisher_subscriber/modify_str.h"

std_msgs::String log_msg;

bool modify_output(first_publisher_subscriber::modify_str::Request &req,
                   first_publisher_subscriber::modify_str::Response &res) {
  ROS_DEBUG_STREAM("[Talker] Received req: " << req.data);
  log_msg.data = req.data;
  res.new_data = "Publish message updated";
  res.status = true;
  ROS_DEBUG_STREAM("[Talker] Responding with resp: " << res.new_data);
  return true;
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
  ros::ServiceServer service = n.advertiseService("modify_output",
                                                    modify_output);

  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok()) {
    std_msgs::String msg;
    std::stringstream ss;

    ss << "My custom message." << count;
    msg.data = ss.str();

    ROS_INFO_STREAM("[Talker] " << msg.data);

    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }
  return 0;
}

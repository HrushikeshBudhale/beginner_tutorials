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

/*
beginner_tutorials
Copyright © 2021
Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
of the Software, and to permit persons to whom the Software is furnished to do
so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include <sstream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "first_publisher_subscriber/modify_str.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/TransformStamped.h"

std_msgs::String log_msg;   // global variable to set message string

/**
 * @brief Service call back to set new message string
 * 
 * @param req Request containing string of new message
 * @param res Response with boolean status and update message
 * @return true 
 * @return false 
 */
bool modify_output(first_publisher_subscriber::modify_str::Request &req,
                   first_publisher_subscriber::modify_str::Response &res) {
  ROS_DEBUG_STREAM("[Talker] Received req: " << req.data);
  log_msg.data = req.data;                    // Updates message string
  res.new_data = "Publish message updated";
  res.status = true;
  ROS_DEBUG_STREAM("[Talker] Responding with resp: " << res.new_data);
  return true;
}


void publish_pose(std::string frame_name) {
  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped;

  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = "/world";
  transformStamped.child_frame_id = frame_name;
  transformStamped.transform.translation.x = 1.0;
  transformStamped.transform.translation.z = 2.0;
  transformStamped.transform.translation.y = 3.0;
  tf2::Quaternion q;
  q.setRPY(0, 0, 3.142);
  transformStamped.transform.rotation.x = q.x();
  transformStamped.transform.rotation.y = q.y();
  transformStamped.transform.rotation.z = q.z();
  transformStamped.transform.rotation.w = q.w();

  br.sendTransform(transformStamped);
}


int main(int argc, char **argv) {
  int freq = 1;
  log_msg.data = "Default log message";

  if (argc > 1) {
    std::istringstream ss(argv[1]);  // Read argument passed by launch file
    if (!(ss >> freq)) {
      ROS_FATAL_STREAM("[Talker] Received invalid frequency" << argv[1]);
      ROS_FATAL_STREAM("[Talker] Unable to start the talker node");
      return 0;
    } else {
        ROS_WARN_STREAM("[Talker] Starting talker with publish freq: " << freq
                                                                    << " Hz");
    }
  } else {    // if no argument passed
    ROS_INFO_STREAM("[Talker] Starting talker node with default freq: " << freq
                                                                    << " Hz");
  }
  ros::init(argc, argv, "talker", ros::init_options::AnonymousName);
  ros::NodeHandle n;

  // Advertise topic to publish on
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

  // Advertise new service
  ros::ServiceServer service = n.advertiseService("modify_output",
                                                    modify_output);

  ros::Rate loop_rate(freq);

  int count = 0;
  while (ros::ok()) {
    std_msgs::String msg;
    std::stringstream ss;

    ss << log_msg.data << " " << count;
    msg.data = ss.str();

    ROS_INFO_STREAM("[Talker] " << msg.data);

    chatter_pub.publish(msg);
    publish_pose("/talk");
    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }
  return 0;
}

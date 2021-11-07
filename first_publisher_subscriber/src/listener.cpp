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

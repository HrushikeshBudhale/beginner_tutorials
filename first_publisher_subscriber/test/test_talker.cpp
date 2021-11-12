/**
 * @file test_talker.cpp
 * @author Hrushikesh Budhale (hbudhale@umd.edu)
 * @brief Cpp file to test working of talker node in 
 *          first_publisher_subscriber package 
 * @version 0.1
 * @date 2021-11-11
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

#include <gtest/gtest.h>
#include <ros/ros.h>
#include "std_msgs/String.h"
#include "first_publisher_subscriber/modify_str.h"

/**
 * @brief Global variales
 * 
 */
std::shared_ptr<ros::NodeHandle> nh;
bool message_received = false;
std_msgs::String message;

/**
 * @brief Custom function to wait for messages
 * 
 * @param message_received Boolean variable to be monitored
 * @param timeout time to wait before returning false. Default value 5 sec
 * @return true if message received within timeout
 * @return false if no message received within timeout 
 */
bool WaitForMessage(const bool& message_received, double timeout = 5) {
    ros::Time start = ros::Time::now();
    ros::Time now;
    while (!message_received) {
        ros::spinOnce();
        now = ros::Time::now();
        if ((now - start).toSec() > timeout) {
            return false;
        }
    }
    return true;
}

/**
 * @brief Topic callback to update message
 * 
 * @param msg 
 */
void msg_callback(const std_msgs::String::ConstPtr& msg) {
    message_received = true;
    message.data = msg->data;
}


/**
 * @brief Test to check proper working of message publisher
 * 
 */
TEST(test_talker, test_publisher) {
    ros::Subscriber sub = nh->subscribe("chatter", 1000, msg_callback);
    std_msgs::String expected_msg;
    expected_msg.data = "Default log message 1";
    ASSERT_TRUE(WaitForMessage(message_received, 3));
    EXPECT_EQ(message.data, expected_msg.data);
}

/**
 * @brief Test to check proper working of service server
 * 
 */
TEST(test_talker, test_service_server) {
    ros::ServiceClient client = nh->serviceClient
                    <first_publisher_subscriber::modify_str>("modify_output");
    bool exists(client.waitForExistence(ros::Duration(1)));
    ASSERT_TRUE(exists);

    first_publisher_subscriber::modify_str srv;
    srv.request.data = "Test string";
    client.call(srv);

    EXPECT_EQ(srv.response.status, true);
    EXPECT_EQ(srv.response.new_data, "Publish message updated");
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "TestingNode");
    nh.reset(new ros::NodeHandle);
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

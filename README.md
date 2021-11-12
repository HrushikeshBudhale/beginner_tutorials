
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

# beginner_tutorials
---

### Description
 Repository containing 'first_publisher_subscriber' package with one talker and one listener node. Talker node publishes message and provides service named 'modify_output'. Listener node subscribes to the messages from talker and calls 'modify_output' after fixed interval of time. Both the nodes can be launched using single launch file.

The launch file provides additional functionality to record the rosbag while launching the nodes.

### Dependencies
- ROS version:
    - Melodic
- Packages:
    - catkin
    - roscpp
    - std_msgs
    - message_generation
    - geometry_msgs
    - tf2_ros
    - rostest

### Assumptions
 It is assumed that the user has installed ROS Melodic full desktop version and has created a catkin workspace folder with src folder inside it.

### Steps to install

 Open terminal in the 'src/' directory of your 'catkin_ws/' and run following commands
```
git clone https://github.com/HrushikeshBudhale/beginner_tutorials.git
cd ..
catkin_make --pkg first_publisher_subscriber
source devel/setup.bash
```

### Steps to launch listener and talker node
 
 In your 'catkin_ws/', enter following command to launch both talker and listener nodes.

 ```
 source devel/setup.bash
 roslaunch first_publisher_subscriber beginner_tutorial.launch
 ```

 To launch the talker node with user defined frequency, enter following command by replacing ```<value>``` by a number
 ```
 roslaunch first_publisher_subscriber beginner_tutorial.launch pub_freq:=<value>
 ```

### Calling rosservice
 To call ros service through command line enter following command by replacing ```<value>``` by a custom message string
 ```
 rosservice call /modify_output "data: '<value>'"
 ```
 This will modify the message being published by the talker node.


### Launch nodes with rosbag record
 To start recording rosbag on launching the nodes, enter following command.
 ```
roslaunch first_publisher_subscriber beginner_tutorial.launch record_bag:=true
 ```
The generated rosbag will be saved in the results directory of this package.


### Steps to run rostest

Open terminal in 'csatkin_ws/' and enter following command
```
catkin_make run_tests_first_publisher_subscriber
```
This will execute 2 existing tests in test_talker.cpp
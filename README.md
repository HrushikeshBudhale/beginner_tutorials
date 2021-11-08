
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

# beginner_tutorials
---

### Description
 Repository containing 'first_publisher_subscriber' package with one talker and one listener node. Talker node publishes message and provides service named 'modify_output'. Listener node subscribes to the messages from talker and calls 'modify_output' after fixed interval of time. Both the nodes can be launched using single launch file.


### Dependencies
- ROS version:
    - Melodic
- Packages:
    - catkin
    - roscpp
    - std_msgs
    - message_generation

### Assumptions
 It is assumed that the user has installed ROS Melodic full desktop version. And has created a catkin_ws folder with src folder inside it.

### Steps to install

 Open terminal in the 'src/' directory of your 'catkin_ws/' and run following commands
```
git clone https://github.com/HrushikeshBudhale/beginner_tutorials.git
cd ..
catkin_make first_publisher_subscriber
source devel/setup.bash
```
### Steps to run
 
 In your 'catkin_ws/', enter following command to launch both talker and listener nodes.

 ```
 source devel/setup.bash
 roslaunch first_publisher_subscriber beginner_tutorial.launch
 ```

 To launch the talker node with user defined frequency, enter following command by replacing ```<value>``` by a number
 ```
 roslaunch first_publisher_subscriber beginner_tutorial.launch pub_freq:=<value>
 ```

 To call ros service through command line enter following command by replacing ```<value>``` by a string
 ```
 rosservice call /modify_output "data: '<value>'"
 ```
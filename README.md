# beginner_tutorials
---

### Description
 Repository containing 'first_publisher_subscriber' package with one publisher and one subscriber.


### Dependencies
- ROS version:
    - Melodic
- Packages:
    - catkin
    - roscpp
    - std_msgs

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
 
 Run ```roscore``` in separate terminal and then in your 'catkin_ws/' directory open 2 terminals.
 In first terminal type
 ```
 source devel/setup.bash
 rosrun first_publisher_subscriber talker
 ```

 In second terminal type
 ```
 source devel/setup.bash
 rosrun first_publisher_subscriber listener
 ```
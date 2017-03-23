# README 

## Create ROS Workspace
* Ensure that environment variables like ROS_ROOT and ROS_PACKAGE_PATH are set
```
$ printenv | grep ROS
```

```
ROS_ROOT=/opt/ros/indigo/share/ros
ROS_PACKAGE_PATH=/opt/ros/indigo/share:/opt/ros/indigo/stacks
ROS_MASTER_URI=http://localhost:11311
ROSLISP_PACKAGE_DIRECTORIES=
ROS_DISTRO=indigo
ROS_ETC_DIR=/opt/ros/indigo/etc/ros
```

```
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make
```

```
$ source devel/setup.bash
```

```
$ echo $ROS_PACKAGE_PATH
   /home/baron/catkin_ws/src:/opt/ros/indigo/share:/opt/ros/indigo/stacks
 ```
  
 * rospack command
 Usage
 ```
 $ rospack find [package_name]
 ```
Example
```
$ rospack find roscpp
```

# README 

## Basic Command
* Ensure that environment variables like ROS_ROOT and ROS_PACKAGE_PATH are set
```
$ printenv | grep ROS
ROS_ROOT=/opt/ros/indigo/share/ros
ROS_PACKAGE_PATH=/opt/ros/indigo/share:/opt/ros/indigo/stacks
ROS_MASTER_URI=http://localhost:11311
ROSLISP_PACKAGE_DIRECTORIES=
ROS_DISTRO=indigo
ROS_ETC_DIR=/opt/ros/indigo/etc/ros
```
* Create ROS workspace
```
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make
```

```
$ source devel/setup.bash
$ echo $ROS_PACKAGE_PATH
/home/baron/catkin_ws/src:/opt/ros/indigo/share:/opt/ros/indigo/stacks
```
  
### rospack
rospack allows you to get information about packages

Usage  
 ```
$ rospack find [package_name]
 ```
 
Example  
```
$ rospack find roscpp
/opt/ros/indigo/share/roscpp
```

### roscd
roscd is part of the rosbash suite. It allows you to change directory (cd) directly to a package or a stack.

Usage  
 ```
$ roscd [locationname[/subdir]]
 ```
 
Example  
```
$ roscd roscpp
$ pwd
/opt/ros/indigo/share/roscpp
```
```
$ roscd roscpp/cmake
$ pwd
/opt/ros/indigo/share/roscpp/cmake
```
roscd log will take you to the folder where ROS stores log files
```
$ roscd log
```

### rosls
rosls is part of the rosbash suite. It allows you to ls directly in a package by name rather than by absolute path.   

Usage 
```
$ rosls [locationname[/subdir]]
```
Example
```
$ rosls roscpp_tutorials
cmake  launch  package.xml  srv
```

## Create ROS Package

### catkin_create_pkg
catkin_create_pkg requires that you give it a package_name and optionally a list of dependencies on which that package depends <br />
Usage:  
```
$ catkin_create_pkg <package_name> [depend1] [depend2] [depend3]
```
Use the catkin_create_pkg script to create a new package called 'beginner_tutorials' which depends on std_msgs, roscpp, and rospy: 
```
$ cd ~/catkin_ws/src
$ catkin_create_pkg beginner_tutorials std_msgs rospy roscpp
Created file beginner_tutorials/CMakeLists.txt
Created file beginner_tutorials/package.xml
Created folder beginner_tutorials/include/beginner_tutorials
Created folder beginner_tutorials/src
Successfully created files in /home/baron/catkin_ws/src/beginner_tutorials. Please adjust the values in package.xml.
```

Build the packages in the catkin workspace
```
$ cd ~/catkin_ws
$ catkin_make
```
After the workspace has been built it has created a similar structure in the devel subfolder as you usually find under /opt/ros/$ROSDISTRO_NAME.  

To add the workspace to your ROS environment you need to source the generated setup file
```
$ . ~/catkin_ws/devel/setup.bash
```
When using catkin_create_pkg earlier, a few package dependencies were provided. These first-order dependencies can now be reviewed with the rospack tool. 
```
$ rospack depends1 beginner_tutorials
roscpp
rospy
std_msgs
```
As you can see, rospack lists the same dependencies that were used as arguments when running catkin_create_pkg. These dependencies for a package are stored in the package.xml file:
```
$ roscd beginner_tutorials
$ cat package.xml
```
```
<package>
...
  <buildtool_depend>catkin</buildtool_depend>
  <build_depend>roscpp</build_depend>
  <build_depend>rospy</build_depend>
  <build_depend>std_msgs</build_depend>
...
</package>
```

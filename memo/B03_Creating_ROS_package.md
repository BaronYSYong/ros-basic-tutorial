# Step 3. Creating a ROS Package

## 1. Creating a catkin package
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

## 2. Building a catkin workspace and sourcing the setup file
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

## 3. Package dependencies
### First-order dependencies
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

### Indirect dependencies

In many cases, a dependency will also have its own dependencies. For instance, rospy has other dependencies. 
```
$ rospack depends1 rospy
genpy
roscpp
rosgraph
rosgraph_msgs
roslib
std_msgs
```

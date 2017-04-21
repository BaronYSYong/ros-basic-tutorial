# Step 1. Installing and configuring ROS environment

## 1. Install ROS
http://wiki.ros.org/indigo/Installation/Ubuntu

```
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
$ sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net --recv-key 0xB01FA116
$ sudo apt-get install ros-indigo-desktop-full
$ rosdep update
$ sudo rosdep init
$ echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
$ sudo apt-get install python-rosinstall
```

## 2. Managing environment
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

## 3. Create ROS workspace
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

# Step 3. Roslaunch tips for large projects

Large applications on a robot typically involve several interconnected nodes, each of which have many parameters. 2d navigation is a good example. The 2dnav_pr2 application consists of the move_base node itself, localization, ground plane filtering, the base controller, and the map server. Collectively, there are also a few hundred ROS parameters that affect the behavior of these nodes. Finally, there are constraints such as the fact that ground plane filtering should run on the same machine as the tilt laser for efficiency.

A roslaunch file allows us to say all this. Given a running robot, launching the file 2dnav_pr2.launch in the 2dnav_pr2 package will bring up everything required for the robot to navigate.

In this case, moving between physically identical robots can be done without changing the launch files at all. Even a change such as moving from the robot to a simulator can be done with only a few changes. 

Install 
```
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
$ sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
$ sudo apt-get update
$ echo "hddtemp hddtemp/daemon boolean false" | sudo debconf-set-selections
$ apt-cache search ros-indigo-pr2-navigation
$ sudo apt-get install ros-indigo-pr2-navigation
```
# Step 3. Roslaunch tips for large projects

Large applications on a robot typically involve several interconnected nodes, each of which have many parameters. 2d navigation is a good example. The 2dnav_pr2 application consists of the move_base node itself, localization, ground plane filtering, the base controller, and the map server. Collectively, there are also a few hundred ROS parameters that affect the behavior of these nodes. Finally, there are constraints such as the fact that ground plane filtering should run on the same machine as the tilt laser for efficiency.

A roslaunch file allows us to say all this. Given a running robot, launching the file 2dnav_pr2.launch in the 2dnav_pr2 package will bring up everything required for the robot to navigate.

In this case, moving between physically identical robots can be done without changing the launch files at all. Even a change such as moving from the robot to a simulator can be done with only a few changes. 

refer to:

* http://wiki.ros.org/pr2_2dnav
* http://wiki.ros.org/ROS/Tutorials/Roslaunch%20tips%20for%20larger%20projects
* https://github.com/PR2/pr2_navigation_apps
* http://wiki.ros.org/pr2_simulator/Tutorials/StartingPR2Simulation
* http://docs.ros.org/indigo/api/moveit_tutorials/html/doc/ros_visualization/visualization_tutorial.html
* http://wiki.ros.org/pr2_simulator/Tutorials
* http://wiki.ros.org/simulator_gazebo/Tutorials
* http://wiki.ros.org/navigation


# Step 8. rqt_console and roslaunch

## 1. rqt_console and rqt_logger_level
* rqt_console attaches to ROS's logging framework to display output from nodes. 
* rqt_logger_level allows us to change the verbosity level (DEBUG, WARN, INFO, and ERROR) of nodes as they run.
```
$ rosrun rqt_console rqt_console
$ rosrun rqt_logger_level rqt_logger_level
```

### logger levels
Logging levels are prioritized in the following order: 
```
Fatal
Error
Warn
Info
Debug
```
* Fatal has the highest priority and Debug has the lowest. 
* By setting the logger level, you will get all messages of that priority level or higher. 
* For example, by setting the level to Warn, you will get all Warn, Error, and Fatal logging messages. 

### roslaunch
roslaunch starts nodes as defined in a launch file. 
```
$ roslaunch [package] [filename.launch]
```
```
$ cd ~/catkin_ws
$ source devel/setup.bash
$ roscd beginner_tutorials
$ mkdir launch
$ cd launch
$ touch turtlemimic.launch
```
paste the following to  turtlemimic.launch
```
<launch>

  <group ns="turtlesim1">
    <node pkg="turtlesim" name="sim" type="turtlesim_node"/>
  </group>

  <group ns="turtlesim2">
    <node pkg="turtlesim" name="sim" type="turtlesim_node"/>
  </group>

  <node pkg="turtlesim" name="mimic" type="mimic">
    <remap from="input" to="turtlesim1/turtle1"/>
    <remap from="output" to="turtlesim2/turtle1"/>
  </node>

</launch>
```
### launch file explanation
let's break the launch xml down. 
```
<launch>
```
Here we start the launch file with the launch tag, so that the file is identified as a launch file. 
```
  <group ns="turtlesim1">
    <node pkg="turtlesim" name="sim" type="turtlesim_node"/>
  </group>

  <group ns="turtlesim2">
    <node pkg="turtlesim" name="sim" type="turtlesim_node"/>
  </group>
```
Here we start two groups with a namespace tag of turtlesim1 and turtlesim2 with a turtlesim node with a name of sim. This allows us to start two simulators without having name conflicts. 
```
  <node pkg="turtlesim" name="mimic" type="mimic">
    <remap from="input" to="turtlesim1/turtle1"/>
    <remap from="output" to="turtlesim2/turtle1"/>
  </node>
```
Here we start the mimic node with the topics input and output renamed to turtlesim1 and turtlesim2. This renaming will cause turtlesim2 to mimic turtlesim1. 
```
</launch>
```
This closes the xml tag for the launch file. 

### roslaunching
Let's roslaunch the launch file: 
```
$ roslaunch beginner_tutorials turtlemimic.launch
```
Run the command below in a new terminal
```
$ rostopic pub /turtlesim1/turtle1/cmd_vel geometry_msgs/Twist -r 1 -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, -1.8]'
```
use rqt_graph to check the architecture
```
$ rqt_graph
```

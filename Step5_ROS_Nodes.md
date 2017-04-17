# Step 5. ROS Nodes

## Overview of Graph Concepts
![](/home/baron/bitbucket/ros-practice/publish_subscribe_concept.png) 

* Nodes: A node is an executable that uses ROS to communicate with other nodes.
* Messages: ROS data type used when subscribing or publishing to a topic.
* Topics: Nodes can publish messages to a topic as well as subscribe to a topic to receive messages.
* Master: Name service for ROS (i.e. helps nodes find each other)
* rosout: ROS equivalent of stdout/stderr
* roscore: Master + rosout + parameter server (parameter server will be introduced later) 

## Nodes
ROS nodes use a ROS client library to communicate with other nodes. Nodes can publish or subscribe to a Topic. Nodes can also provide or use a Service.

## Client libraries
ROS client libraries allow nodes written in different programming languages to communicate:

* rospy = python client library
* roscpp = c++ client library 

## roscore
roscore is a collection of nodes and programs that are pre-requisites of a ROS-based system. You must have a roscore running in order for ROS nodes to communicate. It is launched using the roscore command.
```
$ roscore
```
roscore will start up:

* ROS Master
* ROS Parameter Server
* rosout logging node 

## rosnode
rosnode displays information about the ROS nodes that are currently running. The rosnode list command lists these active nodes: 
```
$ rosnode list
/rosout
```
The rosnode info command returns information about a specific node. 
```
$ rosnode info /rosout
```

## rosrun
rosrun allows you to use the package name to directly run a node within a package (without having to know the package path).

Usage: 
```
$ rosrun [package_name] [node_name]
```
Example:
```
$ rosrun turtlesim turtlesim_node
```
Run below in new terminal
```
$ rosnode list
/rosout
/turtlesim
```
Close the turtlesim window to stop the node (or go back to the rosrun turtlesim terminal and use ctrl-C). Now let's re-run it, but this time use a Remapping Argument to change the node's name: 
```
$ rosrun turtlesim turtlesim_node __name:=my_turtle
```
```
$ rosnode list
/my_turtle
/rosout
```
Note: If you still see /turtlesim in the list, it might mean that you stopped the node in the terminal using ctrl-C instead of closing the window. Try cleaning the rosnode list with: 
```
$ rosnode cleanup 
```
Use another rosnode command, ping, to test new /my_turtle node
```
$ rosnode ping my_turtle
rosnode: node is [/my_turtle]
pinging /my_turtle with a timeout of 3.0s
xmlrpc reply from http://baron-Endeavor-NA512E:33218/	time=0.345945ms
xmlrpc reply from http://baron-Endeavor-NA512E:33218/	time=2.249002ms
xmlrpc reply from http://baron-Endeavor-NA512E:33218/	time=2.254009ms
```

## Review
* roscore = ros+core : master (provides name service for ROS) + rosout (stdout/stderr) + parameter server (parameter server will be introduced later)
* rosnode = ros+node : ROS tool to get information about a node.
* rosrun = ros+run : runs a node from a given package. 
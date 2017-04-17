# Summary of ROS commands

## rospack
```
$ rospack find [package_name]
$ rospack find roscpp
```

## roscd
```
$ roscd [locationname[/subdir]]
$ roscd roscpp
```

## rosls
```
$ rosls [locationname[/subdir]]
$ rosls roscpp_tutorials
```

## roscore
```
$ roscore
```

## rosrun
```
$ rosrun [package_name] [node_name]
$ rosrun turtlesim turtlesim_node
$ rosrun turtlesim turtlesim_node __name:=my_turtle
$ rosrun turtlesim turtle_teleop_key
$ rosrun rqt_graph rqt_graph
$ rosrun rqt_plot rqt_plot
```

## rosnode
```
$ rosnode list
$ rosnode info /rosout
$ rosnode info /turtlesim
$ rosnode cleanup 
$ rosnode ping my_turtle
```

## rostopic
```
$ rostopic -h
$ rostopic echo [topic]
$ rostopic echo /turtle1/cmd_vel
$ rostopic list -h
$ rostopic list -v
$ rostopic type [topic]
$ rostopic type /turtle1/cmd_vel
$ rostopic pub [topic] [msg_type] [args]
$ rostopic pub -1 /turtle1/cmd_vel geometry_msgs/Twist -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, 1.8]'
$ rostopic pub /turtle1/cmd_vel geometry_msgs/Twist -r 1 -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, -1.8]'
$ rostopic hz [topic]
$ rostopic hz /turtle1/pose
$ rostopic type /turtle1/cmd_vel | rosmsg show
```
## rosmsg
```
$ rosmsg show geometry_msgs/Twist
$ rostopic type /turtle1/cmd_vel | rosmsg show
```
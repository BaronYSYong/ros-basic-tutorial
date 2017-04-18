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
$ rostopic pub /turtle1/cmd_vel geometry_msgs/Twist -r 1 -- '{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}'
$ rostopic pub /turtlesim1/turtle1/cmd_vel geometry_msgs/Twist -r 1 -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, -1.8]'
```
## rosmsg
```
$ rosmsg show geometry_msgs/Twist
$ rostopic type /turtle1/cmd_vel | rosmsg show
```

## rosservice
```
rosservice list         print information about active services
rosservice call         call the service with the provided args
rosservice type         print service type
rosservice find         find services by service type
rosservice uri          print service ROSRPC uri
```
```
$ rosservice type [service]
$ rosservice type /clear
$ rosservice call [service] [args]
$ rosservice call /clear
$ rosservice type /spawn| rossrv show
$ rosservice call /spawn 2 2 0.2 ""
```

## rosparam
```
rosparam set            set parameter
rosparam get            get parameter
rosparam load           load parameters from file
rosparam dump           dump parameters to file
rosparam delete         delete parameter
rosparam list           list parameter names
```
```
$ rosparam set [param_name]
$ rosparam get [param_name]
$ rosparam set /background_r 150
$ rosservice call /clear
$ rosparam get /background_g
$ rosparam get /
$ rosparam dump [file_name] [namespace]
$ rosparam load [file_name] [namespace]
$ rosparam dump params.yaml
$ rosparam load params.yaml copy
$ rosparam get /copy/background_b
```

## rqt
```
$ rosrun rqt_graph rqt_graph
$ rosrun rqt_plot rqt_plot
$ rosrun rqt_console rqt_console
$ rosrun rqt_logger_level rqt_logger_level
```

## roslaunch
```
$ roslaunch beginner_tutorials turtlemimic.launch
```

## rosed
```
$ rosed [package_name] [filename]
$ rosed roscpp Logger.msg
```
# Step 4: Running ROS across multiple machines

## 1. Start the master
In local PC
```
$ roscore
```

## 2. Start the listener
In local PC
```
$ export ROS_MASTER_URI=http://192.168.133.94:11311
$ export ROS_IP=192.168.133.94
$ rosrun beginner_tutorials listener.py
```

## 3. Start the talker
in TX1
```
$ ssh ubuntu@tx1
$ export ROS_MASTER_URI=http://192.168.133.94:11311
$ export ROS_IP=192.168.133.122
$ rosrun beginner_tutorials talker.py
```

## 4. rostopic
```
$ rostopic list
/chatter
/rosout
/rosout_agg
$ rostopic echo /chatter
```
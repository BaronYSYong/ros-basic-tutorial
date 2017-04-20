# Step 14. Service and Client Python

## 1. Service Node
Create the service ("add_two_ints_server") node which will receive two ints and return the sum. 
```
$ roscd beginner_tutorials/scripts
$ touch add_two_ints_server.py
```
Don't forget to make the node executable: 
```
$ chmod +x scripts/add_two_ints_server.py
```

### Explanation of code
There's very little to writing a service using rospy. We declare our node using init_node() and then declare our service:
```
s = rospy.Service('add_two_ints', AddTwoInts, handle_add_two_ints)
```
This declares a new service named add_two_ints with the AddTwoInts service type. All requests are passed to handle_add_two_ints function. handle_add_two_ints is called with instances of AddTwoIntsRequest and returns instances of AddTwoIntsResponse. 

## 2. Client Node
```
$ roscd beginner_tutorials/scripts
$ touch add_two_ints_client.py
```
Don't forget to make the node executable: 
```
$ chmod +x scripts/add_two_ints_client.py
```

### Explanation of code
For clients you don't have to call init_node(). We first call: 
```
rospy.wait_for_service('add_two_ints')
```
This is a convenience method that blocks until the service named add_two_ints is available. Next we create a handle for calling the service: 
```
add_two_ints = rospy.ServiceProxy('add_two_ints', AddTwoInts)
```
We can use this handle just like a normal function and call it:
```
resp1 = add_two_ints(x, y)
return resp1.sum
```
Because we've declared the type of the service to be AddTwoInts, it does the work of generating the AddTwoIntsRequest object for you (you're free to pass in your own instead). The return value is an AddTwoIntsResponse object. If the call fails, a rospy.ServiceException may be thrown, so you should setup the appropriate try/except block. 

## 3. Building nodes
```
# In your catkin workspace
$ cd ~/catkin_ws
$ catkin_make
```
## 4. Examining the Simple Service and Client

In a new terminal, run
```
$ cd ~/catkin_ws
$ . devel/setup.bash
$ rosrun beginner_tutorials add_two_ints_server.py
```
In a new terminal, run
```
$ cd ~/catkin_ws
$ . devel/setup.bash
$ rosrun beginner_tutorials add_two_ints_client.py 
/home/baron/bitbucket/ros-practice/catkin_ws/src/beginner_tutorials/scripts/add_two_ints_client.py [x y]
```
Then run
```
$ rosrun beginner_tutorials add_two_ints_client.py 4 5
```
client:
```
Requesting 4+5
4 + 5 = 9
```

server:
```
Returning [4 + 5 = 9]
```

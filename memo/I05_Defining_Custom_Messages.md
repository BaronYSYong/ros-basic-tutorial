# Defining Custom Messages

## 1. Generating Messages
Generating a message is easy. Simply place a .msg file inside the msg directory in a package.  (don't forget to choose build system type at the top of the page there). 

## 2. Including or Importing Messages
### C++
```
#include <std_msgs/String.h>

std_msgs::String msg;
```

### Python
```
from std_msgs.msg import String

msg = String()
```

## 3. Dependencies
If you are using the new custom message defined in a different package, remember to add:

to package.xml:
```
<build_depend>name_of_package_containing_custom_msg</build_depend>
<run_depend>name_of_package_containing_custom_msg</run_depend>
```
```
<build_depend>message_generation</build_depend>
<run_depend>message_runtime</run_depend>
```
to CMakeList.txt: 
```
findPackage(message_generation)
catkin_package(CATKIN_DEPENDS message_runtime)
add_message_files(FILES your_msg_file.msg)
```

http://wiki.ros.org/ROSNodeTutorialPython
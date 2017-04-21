# Step 9. Using rosed to edit files in ROS

## rosed
rosed is part of the rosbash suite. It allows you to directly edit a file within a package by using the package name rather than having to type the entire path to the package. 
```
$ rosed [package_name] [filename]
```
Example:
```
$ rosed roscpp Logger.msg
```

## rosed with tab completion
Usage:
```
$ rosed [package_name] <tab><tab>
```
Example:
```
$ rosed roscpp <tab><tab>
```

## Editor
The default editor for rosed is vim. 
To set the default editor to nano by editing ~/.bashrc file
```
export EDITOR='nano -w'
```
To set the default editor to emacs you can edit your ~/.bashrc file 
```
export EDITOR='emacs -nw'
```


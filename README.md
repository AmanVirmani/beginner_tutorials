# ENPM808X - ROS Beginner Tutorial

[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](https://opensource.org/licenses/MIT)
---

# Overview
This repository contains basic ROS C++ package with a subscriber node and a publisher node. The code from the tutorial mentioned on [ROS website](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29). 
- Takler (src/talker.cpp): publisher
- Listener (src/listener.cpp): subscriber

# Dependencies 
This project requires following dependencies to be installed.
- ROS kinetic : find installation instructions [here](http://wiki.ros.org/kinetic/Installation/Ubuntu) 
- catkin : find installation instructions [here](http://wiki.ros.org/catkin?distro=indigo#Installing_catkin)

The package has been tested on ubuntu 16.04 LTS

# Build Instructions 
Assuming dependencies are met. Follow below commands:


```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
cd src/
git clone https://github.com/AmanVirmani/beginner_tutorials.git
cd ..
catkin_make
```

# Run Instructions 
Source the following command in bashrc

```
nano ~/.bashrc
source ~/catkin_ws/devel/setup.bash
source ~/.bashrc
```
To launch nodes, can either use a launch file or rosrun. Launch file will let user to launch several nodes in one command. Instructions for both types are below: 

#### Using Launch File
To use launch file type the following command in the terminal:
```
roslaunch begineer_tutorials week10HW.launch
```
User can change the frequency at which the loop operates by the following command;
```
roslaunch begineer_tutorials week10HW.launch frequency:=7
```

#### Using rosrun
In a new terminal[1], type 
```
roscore
```
In an another terminal[2], type
```
cd ~/catkin_ws
rosrun beginner_tutorials talker
```
- To change the frequency of publishing message, use this command in terminal[2]. Note that frequency must be an integer and greater than zero.
```
cd ~/catkin_ws
rosrun beginner_tutorials talker <frequency>
```

In an another new terminal[3], type 
```
cd ~/catkin_ws
rosrun beginner_tutorials listener
```

# Service

Change the output string of Talker using the given command:
```
rosservice call /modifyTalkerMessage "sample text"
```

# Logging
Invoke rqt_console GUI using the following command in a new terminal: 
```
rqt_console
```

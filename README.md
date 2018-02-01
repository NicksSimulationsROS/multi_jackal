# Multi-Jackal Simulator using Gazebo ROS

# Overview
These packages make use of the robotic simulator Gazebo, along with the Jackal 
robot description. Multiple Jackals are spawned and are able to be moved 
independently. The difference between this package and the [Jackal package](https://github.com/jackal/jackal), 
is that multiple Jackals are able to be spawned without causing overlap. 
Significant amounts of code are from Clearpaths Jackal package, this just has 
minor changes.

# Gazebo notes
ROS uses an older version of Gazebo. At some point Gazebo wanted to move to 
newer standards and types, and ROS didn't want to (or wasn't able to) keep up. 
Gazebo still has a few bits and pieces of ROS tutorials and such, but it isn't 
as clearly separated from the versions that don't play well with ROS. This means 
that some capabilities appear that they should work in ROS, but don't.

As for the Jackal, if you only want to simulate one, then follow the 
[guide]((https://www.clearpathrobotics.com/assets/guides/jackal/simulation.html). 
The problem is that it isn't scalable. They use the same transformation tree and 
some message names. You can see the problem yourself if you spawn two and have a 
look at the topics and TF tree.

# Files
## my_jackal_gazebo
The starting point for simulating the robots. Contains launch and config files.
Starts up a Gazebo session and launches robots using `my_jackal_base`.
Example: `roslaunch my_jackal_gazebo one_jackal.launch`.

## my_jackal_base
Contains a single launch file that calls all other jackal components.

## my_jackal_control
Launches the velocity controller plugin and robot controls.

## my_jackal_description
Creates a plugin for publishing robot states and transformations. Loads a 
parameter that describes the robot for use in Gazebo.

## my_jackal_nav
Creates the localisation and move_base nodes. This is set up to use the custom
[my_robot_localization](https://github.com/Nick-Sullivan/my_robot_localization), 
but can easily be changed to use the standard [robot_localization](http://wiki.ros.org/robot_localization) 
by altering the launch file. 

# Running
Make sure the file `my_jackal_description/scripts/env_run` is executable.

Example launch files can be found in `my_jackal_gazebo/launch`. Gazebo and RVIZ 
can be viewed with `gzclient` and `roslaunch my_jackal_gazebo rviz.launch`.

# TODO
When using an EKF instead of a TF for map to base_link, a warning is displayed
`Transform from jackal0/base_link to map was unavailable for the time requested.
Using latest instead.`

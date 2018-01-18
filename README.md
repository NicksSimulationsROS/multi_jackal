Multi-Jackal Simulator using the Robot Operating System

Written by Nick Sullivan at the University of Adelaide, 2016
Contact me: nicholas.sullivan@adelaide.edu.au


OVERVIEW
These packages make use of the robotic simulator Gazebo, along with the Jackal robot description.
Multiple Jackals are spawned and are able to be moved independently. The difference between this
package and the Jackal package (https://github.com/jackal/jackal), is that multiple Jackals are
able to be spawned without causing overlap. Significant amounts of code are from Clearpaths Jackal
package, this just has minor changes.


LEGALITIES
 - This guide will assume the user has some experience with ROS. 
 - Verified to work for ROS Indigo on a Ubuntu 14.04 Virtual Machine.
 - Distributed with the Creative Commons Licence CC BY.


A BIT OF BACKGROUND
ROS uses an older version of Gazebo. At some point Gazebo wanted to move to newer standards and types, and ROS
didn't want to (or wasn't able to) keep up. Gazebo still has a few bits and pieces of ROS tutorials and such, but
it isn't as clearly separated from the versions that don't play well with ROS. This means that some 
capabilities appear that they should work in ROS, but don't.

As for the Jackal, if you only want to simulate one, then follow the guide here: 
(https://www.clearpathrobotics.com/assets/guides/jackal/simulation.html). The problem is that it isn't scalable.
They use the same transformation tree and some message names. You can see the problem yourself if you spawn two 
and have a look at the topics and TF tree.


HOW DO I RUN THE THING?

To run, copy the custom Jackal packages and multi Gazebo folders into your catkin workspace. To successfully compile,
install the following dependent packages (using sudo apt-get install ros-indigo-package-name):
 - controller_manager
 - diff_drive_controller
 - gazebo_ros_control
 - hector_gazebo_plugins
 - interactive_marker_twist_server
 - joint_state_controller
 - lms1xx
 - move_base
 - pointgrey_camera_description
 - robot_localization

Alternatively, if you compile without installing the dependencies it will prompt you to say that it can't find the
specific package. Then go into the folder jackal/my_jackal_description/scripts and make the env_run file
executable.

To run it, we need three terminals (remember to source devel/setup.bash). In the first, run:
	roslaunch my_jackal_gazebo one_jackal.launch
The second:
	gzclient
The third:
	roslaunch my_jackal_gazebo rviz.launch

It's possible to do all these three in the same script. But for some reason, Gazebo occasionally crashes while starting 
up, so I prefer launching in this way. I don't know what the cause is, so if you find a solution, please let me know. 
The first time you run Gazebo, it will take a few seconds to download common models. The controller spawners may die
while waiting, just run it again once the models have been downloaded.


FILE DIFFERENCES

The base, control, and nav launch files have been altered to use a namespace as a prefix for all nodes and topics. There
is one case where a topic had frame information that couldn't be altered this way, so a node was made to subscribe
and republish this information with corrected frames. The file 'jackal.gazebo' was altered to use a namespace as well.
Display the TF while running multiple jackals to see the independent trees.

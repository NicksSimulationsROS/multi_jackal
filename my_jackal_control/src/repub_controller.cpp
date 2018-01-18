/* Written by Nick Sullivan, the University of Adelaide, 2016. */
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"

using namespace std;

/* Control settings specified in 'control.yaml'. The 'diff_drive_controller' 
   being used to control the wheels and publish odometry uses the wrong parent 
   and child frame in its odometry publishing. It does not currently support 
   dynamically altering the child_frame_id, and locks the frame as 'odom', so 
   we need to republish it ourselves. So the 
   'jackal0/jackal_velocity_controller/odom' will become
   'jackal0/jackal_velocity_controller/odom_fixed', making it usable for the 
   localisation fusion.*/
	     
string ns;                  // The namespace to add to the frames.
ros::Publisher odom_pub;    // The publisher to re-publish the messages.

// Callback function. Called whenever we get a new odometry message.
void velocityOdomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
  nav_msgs::Odometry new_msg = *msg;
  // Change the values.
  std::stringstream ss;
  ss << ns << "/odom";
  new_msg.header.frame_id = ss.str();
  std::stringstream ss2;
  ss2 << ns << "/base_link";
  new_msg.child_frame_id = ss2.str();
  // Publish.
  odom_pub.publish(new_msg);
}

// Main. Start via ROS launch.
int main(int argc, char **argv) {
  // Initialise ROS.
  ros::init(argc, argv, "repub_controller");
  ros::NodeHandle n;
  // Get the parameter for namespace.
  ros::NodeHandle n_priv("~");
  if (n_priv.hasParam("namespace")){
    n_priv.getParam("namespace", ns);
  } else {
    cout << "Parameter 'namespace' must be set." << endl;
    return 0;
  }
  // Subscribe to targets being tracked.
  ros::Subscriber sub = n.subscribe("jackal_velocity_controller/odom", 1000, velocityOdomCallback);
  // Advertises the targets being tracked currently, as well as a topic to test blocking.
  odom_pub = n.advertise<nav_msgs::Odometry>("jackal_velocity_controller/odom_fixed", 1000);
  // Let ROS take over.
  ros::spin();
}

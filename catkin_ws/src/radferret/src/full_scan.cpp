#include <ros/ros.h>
#include <std_msgs/Byte.h>
#include <tf2/LinearMath/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <laser_assembler/AssembleScans2.h>
#include <sensor_msgs/PointCloud2.h>
#include "roboteq_msgs/Command.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <iostream>
 
ros::Publisher pub_cmd;
ros::Publisher pub_mode;
ros::ServiceClient client;

// Current angle the assembly is at
double curAngle;

// Callback for odometry message from motor encoder
void odomCallback(const nav_msgs::Odometry& msg){ 
  geometry_msgs::Quaternion gq = msg.pose.pose.orientation;
  tf2::Quaternion bt(gq.x, gq.y, gq.z, gq.w);
  // Update current assembly angle
  curAngle = bt.getAngle();
}

// Performs single scan from homed position
void singleLaserScan() {
  
  ros::Rate r(100);

  // Define roboteq command message
  roboteq_msgs::Command cmd;
   
  // Set to speed mode and set speed to 5% in CW direction
  cmd.setpoint = 50.0;  
  cmd.mode = 6;
  pub_cmd.publish(cmd);

  // Run until full turn has been made
  while( curAngle < 6.27 ) {
	pub_cmd.publish(cmd);
    // Update callbacks
    ros::spinOnce();
    r.sleep();
  }

  // Stop motor
  cmd.setpoint = 0.0;
  cmd.mode = 6;
  pub_cmd.publish(cmd);
}

void takeHDRImages() {
  
}

int main(int argc, char** argv){
  ros::init(argc, argv, "yay");
    
  ros::NodeHandle node;
  
  curAngle = 0;

  // Start roboteq command publisher
  pub_cmd = node.advertise<roboteq_msgs::Command>("/roboteq/cmd", 10);

  // Start roboteq mode publisher
  pub_mode = node.advertise<std_msgs::Byte>("/roboteq/mode", 10);

  // Subscribe to roboteq odometry messages
  ros::Subscriber sub = node.subscribe("/roboteq/odom", 10, &odomCallback);
  ROS_INFO("Running Scan");
  singleLaserScan();
  takeHDRImages();

  ros::spin();
  return 0;
}

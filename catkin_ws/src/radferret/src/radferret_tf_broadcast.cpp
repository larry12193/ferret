#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <iostream>

void odomCallback(const nav_msgs::Odometry& msg){
  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped;
  
  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = "odom";
  transformStamped.child_frame_id = msg.child_frame_id;
  transformStamped.transform.translation.x = 0;
  transformStamped.transform.translation.y = 0.0;
  transformStamped.transform.translation.z = 0.0;

  transformStamped.transform.rotation.x = msg.pose.pose.orientation.x;
  transformStamped.transform.rotation.y = msg.pose.pose.orientation.y;
  transformStamped.transform.rotation.z = msg.pose.pose.orientation.z;
  transformStamped.transform.rotation.w = msg.pose.pose.orientation.w;

  br.sendTransform(transformStamped);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "radferret_tf_broadcaster");
  ROS_INFO("*******************************************************************************");
  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe("/roboteq/odom", 10, &odomCallback);

  ros::spin();
  return 0;
};

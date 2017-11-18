#include <ros/ros.h>
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
// #include <pcl/point_types.h>

ros::Publisher pub;
ros::ServiceClient client;

void odomCallback(const nav_msgs::Odometry& msg){ 
  geometry_msgs::Quaternion gq = msg.pose.pose.orientation;
  tf2::Quaternion bt(gq.x, gq.y, gq.z, gq.w);

  static roboteq_msgs::Command cmd;
  static double prev_angle = bt.getAngle();
  static ros::Time begin(0.0);
  static ros::Time end(0.0);

  double curr_angle = bt.getAngle();
  std::cout << bt.getAngle() << std::endl;
  if (bt.getAngle() == prev_angle) {
    if (bt.getAngle() < 0.05) {
      cmd.setpoint = 50.0;
      begin = ros::Time::now();
      std::cout << "begin gathering!" << std::endl;
    }
    else if (bt.getAngle() > 6.27) {
      cmd.setpoint = 0.0;
      laser_assembler::AssembleScans2 srv;
      srv.request.begin = begin;
      srv.request.end = ros::Time::now();
      std::cout << "end gathering!" << std::endl;

      // Make the service call
      if (client.call(srv)) {
        std::stringstream ss;
        ss << "/media/odroid/data/" << srv.response.cloud.header.stamp << ".pcd";
        ROS_INFO ("Saving data to %s", ss.str ().c_str ());
        pcl::io::savePCDFile( ss.str(), srv.response.cloud);
		ROS_INFO ("Data saved. Exiting...", ss.str ().c_str ());
      } else {
        ROS_ERROR("Error making service call\n") ;
      }

      ros::shutdown();
    }
  } else {
    // cmd.setpoint = -100.0;
  }

  prev_angle = curr_angle;
  cmd.mode = 6;
  pub.publish(cmd);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "yay");
    
  ros::NodeHandle node;

  ROS_INFO("Waiting for [build_cloud] to be advertised");
  ros::service::waitForService("build_cloud");
  ROS_INFO("Found build_cloud! Starting the snapshotter");
  client = node.serviceClient<laser_assembler::AssembleScans2>("assemble_scans2");

  pub = node.advertise<roboteq_msgs::Command>("/roboteq/cmd", 10);
  ros::Subscriber sub = node.subscribe("/roboteq/odom", 10, &odomCallback);

  ros::spin();
  return 0;
}

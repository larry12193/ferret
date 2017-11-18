#include "roboteq_driver/controller.h"
#include "roboteq_driver/channel.h"
#include <inttypes.h>
#include <std_msgs/Byte.h>
#include "ros/ros.h"
#include "serial/serial.h"
#include <iostream>


// Define and initialize motor controller mode status flag
uint8_t curMode  = CLOSED_LOOP_SPEED;
uint8_t lastMode = CLOSED_LOOP_SPEED;

void modeCallback(const std_msgs::Byte& msg) {
  lastMode = curMode;
  curMode  = (uint8_t)msg.data;
  std::cout << "Changing mode" << std::endl;
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "~");
  ros::NodeHandle nh("~");

  std::string port = "/dev/ttyUSB0";
  int32_t baud = 115200;
  nh.param<std::string>("port", port, port);
  nh.param<int32_t>("baud", baud, baud);

  // Interface to motor controller.
  roboteq::Controller controller(port.c_str(), baud);
  // Default configuration is a single channel in the node's namespace.
  controller.addChannel(new roboteq::Channel(1, "~", &controller));

  // Setup roboteq mode subscriber
  ros::Subscriber sub_mode = nh.subscribe("/roboteq/mode", 10, &modeCallback);

  // Attempt to connect and run.
  while (ros::ok()) {
    ROS_DEBUG("Attempting connection to %s at %i baud.", port.c_str(), baud);
    controller.connect();
    if (controller.connected()) {
      // Set to closed loop speed for homing
      controller.setMode(CLOSED_LOOP_SPEED);
      // Start script
      controller.startScript();
      // Ensure data was sent to roboteq
      controller.flush();
      ros::AsyncSpinner spinner(1);
      spinner.start();
      while (ros::ok()) {
        if( curMode != lastMode ) {
          controller.setMode(curMode);
        }
        // Continuously reads any serial data being sent from roboteq and parces it
        controller.spinOnce();
      }
      spinner.stop();
    } else {
      ROS_DEBUG("Problem connecting to serial device.");
      ROS_ERROR_STREAM_ONCE("Problem connecting to port " << port << ". Trying again every 1 second.");
      sleep(1);
    }  
  }

  return 0;
}

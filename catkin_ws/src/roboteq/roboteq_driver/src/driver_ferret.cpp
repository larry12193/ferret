#include "roboteq_driver/controller.h"
#include "roboteq_driver/channel.h"
#include <inttypes.h>
#include <std_msgs/UInt8.h>
#include "ros/ros.h"
#include "serial/serial.h"
#include <iostream>


// Define and initialize motor controller mode status flag
uint8_t curMode  = CLOSED_LOOP_SPEED;
uint8_t lastMode = CLOSED_LOOP_SPEED;
uint8_t restartScriptMode = 0;

void modeCallback(const std_msgs::UInt8& msg) {
  lastMode = curMode;
  curMode  = msg.data;
}

void restartScriptCallback(const std_msgs::UInt8::ConstPtr& msg) {
    restartScriptMode = msg->data;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "~");
  ros::NodeHandle nh("~");

  std::string port = "/dev/ttyUSB0";
  int32_t baud = 115200;
  nh.param<std::string>("port", port, port);
  ROS_INFO("%s", port.c_str());
  nh.param<int32_t>("baud", baud, baud);

  // Interface to motor controller.
  roboteq::Controller controller(port.c_str(), baud);

  // Default configuration is a single channel in the node's namespace.
  controller.addChannel(new roboteq::Channel(1, "~", &controller));

  // Setup roboteq mode subscriber
  ros::Subscriber sub_mode = nh.subscribe("/roboteq/mode", 10, &modeCallback);
  ros::Subscriber reset_script = nh.subscribe("/roboteq/restartScript", 1, &restartScriptCallback);
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

      // Create asynchronous spinner to handle serail data
      ros::AsyncSpinner spinner(1);
      // Start spinner
      spinner.start();

      while (ros::ok()) {
        // Catch changing of motor control mode
        if( curMode != lastMode ) {
          controller.setMode(curMode);
          controller.flush();
          lastMode = curMode;
        }

        // Catch signal to restart the MicroBasic script
        if( restartScriptMode ) {
            ROS_INFO("Restarting script...");
            controller.stopScript();
            controller.flush();
            ros::Duration(2).sleep();
            controller.restartScript();
            controller.flush();
            restartScriptMode = 0;
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

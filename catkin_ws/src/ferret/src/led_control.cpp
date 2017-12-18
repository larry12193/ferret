/*******************************************************************************
 * led_control.cpp - Provides control in ROS for ferret LEDs controlled by
 *                   the arduino
 *
 * Carnegie Mellon University
 * Author: Lawrence Papincak
 *
 *
 ******************************************************************************/

#include <ros/ros.h>
#include <ros/xmlrpc_manager.h>
#include <std_msgs/UInt8.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>     // UNIX standard function definitions
#include <fcntl.h>      // File control definitions
#include <errno.h>      // Error number definitions
#include <termios.h>    // POSIX terminal control definitions
#include <sys/ioctl.h>
#include <string.h>
#include <signal.h>
#include <fstream>

volatile uint8_t pwmDutyCycle;
volatile uint8_t shutdown_request;
volatile bool setPwm;

/* @brief Callback for LED PWM message

 */
void ledControlCallback(const std_msgs::UInt8::ConstPtr& msg) {
    pwmDutyCycle = msg->data;
    setPwm = true;
}

/* @brief Replacement SIGINT handler
 */
void sigIntHandler(int sig) {
    shutdown_request = 1;
}

/* @brief Replacement "shutdown" XMLRPC callback
 */
void shutdownCallback(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result) {
    int nParams = 0;
    if( params.getType() == XmlRpc::XmlRpcValue::TypeArray ) {
        nParams = params.size();
    }
    if( nParams > 1 ) {
        std::string reason = params[1];
        ROS_WARN("Executive shutdown request recieved. Reason [%s]",reason.c_str());
        shutdown_request = 1;
    }

    result = ros::xmlrpc::responseInt(1,"",0);
}


int main(int argc, char **argv) {

	ros::init(argc, argv, "led_control_node", ros::init_options::NoSigintHandler);
    ros::NodeHandle n;
	ros::NodeHandle n_private("~");
    signal(SIGINT, sigIntHandler);

    // Override XMLRPC shutdown
    ros::XMLRPCManager::instance()->unbind("shutdown");
    ros::XMLRPCManager::instance()->bind("shutdown",shutdownCallback);

    std::string pwm_set_topic, port, data;

    n_private.param<std::string>("pwm_set_topic", pwm_set_topic, "/led_pwm");
    n_private.param<std::string>("arduino_port", port, "/dev/arduinoMicro");

    ros::Subscriber pwm_sub = n.subscribe<std_msgs::UInt8>(pwm_set_topic,1,ledControlCallback);

    shutdown_request = 0;

    ros::Rate lrate(10);
    std::ostringstream ss;

    while( ros::ok() && !shutdown_request ) {
        if( setPwm ) {
            setPwm = false;
            //ss.str("");
            //ss.clear();
            //ss << (int)pwmDutyCycle;
            //data = ss.str();
            //ROS_INFO("Writing - %s",data.c_str());
            //write(fd,data.c_str(),strlen(data.c_str()));
            if(  pwmDutyCycle == 0 ) {
                system("echo -ne \"0\" > /dev/arduinoMicro");
		ROS_INFO("Setting to 0");
            } else {
                system("echo -ne \"255\" > /dev/arduinoMicro");
                ROS_INFO("Setting to 200");
	    }
        }
        lrate.sleep();
        ros::spinOnce();
    }

}

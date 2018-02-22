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

    // Define port settings struct
    struct termios tty;
    int fd, flag;

    fd = open(port.c_str(), O_RDWR | O_NONBLOCK | O_NDELAY );

    if( fd < 0 ) {
        ROS_ERROR("Could not connect to led controller");
        return -1;
    } else {
        // Setting other Port Stuff
        tty.c_lflag = 0;        // Input modes
        tty.c_oflag = 0;        // Output modes
        tty.c_iflag = 0;        // Control modes
        tty.c_cflag = 0;        // Local modes
        tty.c_cc[VMIN]  = 0;    // Read a minimum of zero bytes at a time
        tty.c_cc[VTIME] = 1;    // 0.1s timeout
        tty.c_cflag = CS8 | CREAD | ~HUPCL;
        cfsetospeed(&tty, B115200);

        // Push settings to port
        flag = tcsetattr(fd, TCSANOW, &tty);
        fcntl(fd, F_SETFL, 0);
    }

    char buf[8];

    while( ros::ok() && !shutdown_request ) {
        // Clear buffer
        memset(buf,0,sizeof(buf));
        // Write value to buffer
        sprintf(buf,"%d\r\n",pwmDutyCycle);
        // Write data buffer to serial port
        write(fd,buf,sizeof(buf));

        // Throttle loop
        ros::spinOnce();
        lrate.sleep();
    }
    return 0;
}

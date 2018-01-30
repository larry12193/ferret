/*******************************************************************************
 * executive_node.cpp - Executive state machine that runs operational sequence
 *                      of the ferret
 *
 * Carnegie Mellon University
 * Author: Lawrence Papincak
 *
 *
 ******************************************************************************/

#include <ros/ros.h>
#include <ros/console.h>
#include <ros/xmlrpc_manager.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Image.h>
#include <roboteq_msgs/Command.h>
#include <roboteq_msgs/FerretRotator.h>
#include <roboteq_driver/controller.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include "executive/executive_node.hpp"

#include <inttypes.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <sstream>
#include <time.h>
#include <sys/stat.h>
#include <signal.h>
#include <opencv2/highgui/highgui.hpp>
#include <boost/filesystem.hpp>

volatile uint8_t LIDAR_READY;       // State of LIDAR node
volatile uint8_t CAMERA_READY;      // State of camera node
volatile uint8_t IMU_READY;         // State of IMU node
volatile uint8_t HOMING_COMPLETE;   // State of motor homing sequence
volatile uint8_t FORWARD_LIMIT;     // State of forward limit switch
volatile uint8_t REVERSE_LIMIT;     // State of reverse limit switch
volatile int32_t CUR_COUNTER;       // Current counter value
volatile int32_t LAST_COUNTER;      // Last counter value
volatile int32_t DIFF_COUNTER;      // Difference between counter values
volatile uint8_t DYN_CONFIG_READY;  // State of dynamic reconfigure node
volatile uint8_t HDR_STARTED;       // Whether HDR image sequence started
volatile uint8_t HDR_ENDED;         // Whether HDR image sequence ended

volatile uint16_t POSITION_NUMBER;
volatile double   TARGET_POSITION;
volatile uint8_t  IMAGE_NUMBER;
volatile uint8_t  IMAGE_TRIGGER;
volatile uint8_t  shutdown_request;

bool LOGGING_ENABLED;           // Whether logging is enabled during run

int LIDAR_NODE_ID;              // LIDAR node id number
int CAMERA_NODE_ID;             // Camera node id number
int IMU_NODE_ID;                // IMU node id number
int HDR_NODE_ID;                // HDR node id number
int DYN_NODE_ID;                // Dynamic reconfigure node ID
int HDR_START_ID;               // HDR sequence start ID
int HDR_END_ID;                 // HDR sequence end ID
int CONFIG_SET_ID;              // Dynamic reconfigure setting ID

double SERVICE_SPEED;           // Speed (rad/s) during non-scan motion
double SCAN_SPEED;              // Speed (rad/s) during scan motion
double GEAR_RATIO;              // Gearbox ratio
double EXT_RATIO;               // External gear ratio
double CPR;                     // Counts per revolution of the motor

uint8_t current_command = 0;    // Currnet command from user
uint8_t last_command;           // Last user command

uint8_t logging = 0;            // State of rosbag logger
uint8_t logging_background = 0; // State of background logger

std::string root_launch_dir;    // Path to ferret launch directory

std::string base_directory;     // Log directory (ssd static directory)
std::string dataset_directory;  // Specific dataset directory
std::string scan_directory;     // Scan data subdirectory
std::string hdr_directory;      // HDR image subdirectory
std::string hdr_position_dir;   // Single position of HDR images subdirectory

double twoPi;

ros::Publisher exec_resp_pub;           // Publisher for executive response
ros::Publisher roboteq_mode_pub;        // Publisher for setting roboteq mode
ros::Publisher roboteq_setpoint_pub;    // Publisher for setpoint
ros::Publisher roboteq_script_restart;  // Publisher for restarting roboteq script

// Define states that the robot can be in
//  waiting  - waiting to be commanded
//  done     - finished commanded task, cleaning up for next command
//  scanning - performing 360 degree LIDAR scan
//  hdr      - taking HDR images at set angular locations
enum State {waiting, done, estop, start_scan, scanning, hdr_init, hdr_sequence, homing};
State current_state = waiting, prior_state = done, entering_state = done;

// Define motor modes
enum Mode {closed_loop_velocity, closed_loop_count_pos};
Mode current_mode = closed_loop_velocity;

// Define motion states of the rotator
enum Motion {clockwise, counter_clockwise, stopped};
Motion current_motion = stopped;

/* @breif Spins for a specific amount of seconds

 */
void spinFor(int sec) {
    ros::Rate lrate(10);
    for( int i = 0; i < sec*10; i++ ) {
        ros::spinOnce();
        lrate.sleep();
    }
}

/* @brief Sends command to roboteq driver to restart the MicroBasic script that
          handles homing sequence and status reporting
 */
void startHomingSequence() {
    std_msgs::UInt8 msg;
    msg.data = 1;
    roboteq_script_restart.publish(msg);
}

/* @brief Upates state of homing based on communication from roboteq

*/
void homingCallback(const std_msgs::UInt8::ConstPtr& msg) {
    HOMING_COMPLETE = msg->data;
}

/* @brief Updates rotator limit switch information

*/
void rotatorStatusCallback(const roboteq_msgs::FerretRotator::ConstPtr& msg) {
    FORWARD_LIMIT = msg->forward_limit;
    REVERSE_LIMIT = msg->reverse_limit;

    LAST_COUNTER = CUR_COUNTER;
    CUR_COUNTER = msg->count;
    DIFF_COUNTER = CUR_COUNTER - LAST_COUNTER;
}

/* @brief Changes the motor operational mode

 */
void setOperationalMode(Mode mode) {
    std_msgs::UInt8 msg;
    if( mode == closed_loop_count_pos ) {
        msg.data = CLOSED_LOOP_POS;
        roboteq_mode_pub.publish(msg);
        current_mode = closed_loop_count_pos;
    } else if( mode == closed_loop_velocity ) {
        msg.data = CLOSED_LOOP_SPEED;
        roboteq_mode_pub.publish(msg);
        current_mode = closed_loop_velocity;
    }
}

/* @brief Sets the rotational speed setpoint of the motor

*/
void setRotationSpeed(float speed, int8_t dir) {
    roboteq_msgs::Command cmd;

    // Ensure that the motor is in the proper mode
    if( current_mode != closed_loop_velocity ) {
        setOperationalMode(closed_loop_velocity);
        spinFor(5);
    }
    // Apply direction and convert RPM to rad/s
    cmd.setpoint = ((float)dir)*(speed*GEAR_RATIO*EXT_RATIO*(twoPi/60.0));
    cmd.mode = cmd.MODE_VELOCITY;
    roboteq_setpoint_pub.publish(cmd);
    // Spin a few times to get the command executed
    spinFor(1);
}

/* @brief Sets the angular position setpoint of the motor

*/
void setAngularPosition(double position) {
    roboteq_msgs::Command cmd;

    // Ensure that the motor is in the proper mode
    if( current_mode != closed_loop_count_pos ) {
        setOperationalMode(closed_loop_count_pos);
        spinFor(5);
    }
    // Apply direction and convert RPM to rad/s
    cmd.setpoint = position*CPR/twoPi;
    ROS_INFO("Position setpoint: %f",cmd.setpoint );
    cmd.mode = cmd.MODE_POSITION;
    roboteq_setpoint_pub.publish(cmd);
    // Spin a few times to get the command executed
    spinFor(1);
}

/* @brief Processes commands sent from control computer

*/
void commandCallback(const std_msgs::UInt8::ConstPtr& msg) {
    last_command = current_command;
    current_command = msg->data;
}

/* @brief Helper function to log comments

*/
void publish_comment(const std::string& comment)
{
    std_msgs::String msg;
    msg.data = comment;
    ROS_INFO("Executive Comment - %s",msg.data.c_str());

    ros::NodeHandle n;
    ros::Publisher executive_comments_pub = n.advertise<std_msgs::String>("executive_comments", 5);
    executive_comments_pub.publish(msg);
}

/* @brief Returns whether or not all necessary nodes are ready

*/
uint8_t nodesReady() {
    return (LIDAR_READY &&
            CAMERA_READY &&
            IMU_READY &&
            DYN_CONFIG_READY);
}

/* Processes messages from nodes when they are ready

*/
void nodeReadyCallback(const std_msgs::UInt8::ConstPtr& msg) {

    // Process which node is ready
    if( msg->data == LIDAR_NODE_ID ) {
        LIDAR_READY = 1;
    } else if( msg->data == CAMERA_NODE_ID ) {
        CAMERA_READY = 1;
    } else if( msg->data == IMU_NODE_ID ) {
        IMU_READY = 1;
    } else if( msg->data == DYN_NODE_ID) {
        DYN_CONFIG_READY = 1;
    } else if( msg->data == HDR_START_ID ) {
        HDR_STARTED = 1;
    } else if( msg->data == HDR_END_ID) {
        HDR_ENDED = 1;
    }

    // Republish message on executive response topic to ack
    exec_resp_pub.publish(msg);
}

/* @brief Processes incoming commands and sets the appropriate action in motion

*/
void processCommands() {
    switch( current_command ) {
        case STOP_COMMAND:
            current_state = estop;
            ROS_DEBUG("Command: Recieved stop command.");
            break;
        case START_SCAN_COMMAND:
            ROS_DEBUG("Command: Recieved start scan command.");
            current_state = start_scan;
            break;
        case START_HDR_COMMAND:
            ROS_DEBUG("Command: Recieved start HDR command.");
            current_state = hdr_init;
            break;
        case STATUS_COMMAND:
            ROS_DEBUG("Command: Recieved status command.");
            break;
        default:
            ROS_DEBUG("Command: Recieved unknown command.");
            break;
    }
}

void stop_bag();

/* @brief Starts recording the rosbag with the appropriate topics

 */
void start_bag(uint8_t TYPE) {
    if( !logging && LOGGING_ENABLED ) {
        publish_comment("Starting logger...");

        // Make sure no tmux bag logger session is running
        stop_bag();

        std::string sysCall;
        // Construct system call string, running new instance in a detached screen terminal
        if( TYPE == BAG_TYPE_SCAN ) {
            sysCall = "tmux new -s record_bag -d 'roslaunch " + root_launch_dir + "/individual_launch/ferret_scan_logger.launch dir:=" + scan_directory + "'";
        } else if( TYPE == BAG_TYPE_HDR ) {
            sysCall = "tmux new -s record_bag -d 'roslaunch " + root_launch_dir + "/individual_launch/ferret_hdr_logger.launch dir:=" + hdr_directory + "'";
        }

        // Make system call to run command
        system(sysCall.c_str());

        // Set logging flags
        logging = 1;
    }
}

/* @brief Stops recording rosbag
s
 */
void stop_bag() {
    // Check if logging is currently running and enabled
    if( logging && LOGGING_ENABLED ) {
        publish_comment("Stopping logger...");
        system("tmux kill-session -t record_bag ");
        logging = 0;
    }
}

/* @brief Starts recording the background rosbag

 */
void start_background_bag() {
    if( !logging_background && LOGGING_ENABLED ) {
        publish_comment("Starting background logger...");

        // Construct system call string, running new instance in a detached screen terminal
        std::string sysCall = "tmux new -s background_logger -d 'roslaunch " + root_launch_dir + "/individual_launch/ferret_background_logger.launch dir:=" + dataset_directory + "'";

        // Make system call to run command
        system(sysCall.c_str());

        // Set background logger flag
        logging_background = 1;
    }
}

/* @brief Stops recording background rosbag

 */
void stop_background_bag() {
    // Check if logging is currently running and enabled
    if( logging_background && LOGGING_ENABLED ) {
        publish_comment("Stopping background logger...");
        system("tmux kill-session -t background_logger ");

        // Reset baground logger flag
        logging_background = 0;
    }
}

/* @brief Initializes the data set directory that all scan and HDR data will be recorded to

 */
void initializeDirectories()
{
    // Get the current date and time to append to folder
    time_t     now = time(0);
    struct tm  tstruct;
    char       buf[80];
    tstruct = *localtime(&now);
    strftime(buf, sizeof(buf), "%Y-%m-%d-%X", &tstruct);

    // Append date and time to data folder name
    dataset_directory = base_directory + "/dataset_" + buf;
    // Replace all colons with dashes
    std::replace( dataset_directory.begin(), dataset_directory.end(), ':', '-');

    // Create data directory
    boost::filesystem::path dir(dataset_directory.c_str());
    if(!boost::filesystem::create_directory(dir)) {
        ROS_ERROR("Unable to create data base directory at %s", dataset_directory.c_str());
    }
    ROS_DEBUG("Writing run data to %s",dataset_directory.c_str());
}

/* @brief Initializes the scan directory that the current scan data will be stored in

 */
void initializeNewScanDirectory()
{
    // Get the current date and time to append to folder
    time_t     now = time(0);
    struct tm  tstruct;
    char       buf[80];
    tstruct = *localtime(&now);
    strftime(buf, sizeof(buf), "%Y-%m-%d-%X", &tstruct);

    // Append date and time to scan folder name
    scan_directory = dataset_directory + "/scan_" + buf;
    // Replace all colons with dashes
    std::replace( scan_directory.begin(), scan_directory.end(), ':', '-');

    // Create scan directory
    boost::filesystem::path dir(scan_directory.c_str());
    if(!boost::filesystem::create_directory(dir)) {
        ROS_ERROR("Unable to create scan directory at %s", dataset_directory.c_str());
    }
    publish_comment("Writing scan data to " + scan_directory);
}

/* @brief Initializes the scan directory that the current scan data will be stored in

 */
void initializeNewHDRDirectory()
{
    // Get the current date and time to append to folder
    time_t     now = time(0);
    struct tm  tstruct;
    char       buf[80];
    tstruct = *localtime(&now);
    strftime(buf, sizeof(buf), "%Y-%m-%d-%X", &tstruct);

    // Append date and time to scan folder name
    hdr_directory = dataset_directory + "/hdr_" + buf;
    // Replace all colons with dashes
    std::replace( hdr_directory.begin(), hdr_directory.end(), ':', '-');

    // Create scan directory
    boost::filesystem::path dir(hdr_directory.c_str());
    if(!boost::filesystem::create_directory(dir)) {
        ROS_ERROR("Unable to create scan directory at %s", hdr_directory.c_str());
    }
    ROS_DEBUG("Writing HDR data to %s", hdr_directory.c_str());
}

/* @brief Initializes the scan directory that the current scan data will be stored in

 */
void initializeNewHDRPositionDirectory()
{
    // Get the current date and time to append to folder
    time_t     now = time(0);
    struct tm  tstruct;
    char       buf[80];
    tstruct = *localtime(&now);
    strftime(buf, sizeof(buf), "%Y-%m-%d-%X", &tstruct);

    // Convert int to string
    std::ostringstream ss;
    ss << POSITION_NUMBER;

    // Append date and time to position folder name
    hdr_position_dir = hdr_directory + "/" + ss.str();

    // Create scan directory
    boost::filesystem::path dir(hdr_position_dir.c_str());
    if(!boost::filesystem::create_directory(dir)) {
        ROS_ERROR("Unable to create hdr position directory at %s", hdr_position_dir.c_str());
    }
}

/* @breif Callback for image trigger callback

 */
void imageTriggerCallback(const std_msgs::UInt8::ConstPtr& msg) {
    IMAGE_TRIGGER = 1;
}

/* @brief Callback for camera images

 */
void imageCallback(const sensor_msgs::Image::ConstPtr& msg) {
    if( IMAGE_TRIGGER ) {
        // Reset trigger flag for next image
        IMAGE_TRIGGER = 0;

        // Convert image number to string and format image name
        std::ostringstream ss;
        ss << (int)IMAGE_NUMBER;
        std::string name;
        name = "/hdr_image_" + ss.str() + ".jpg";
        std::string whole;
        whole = hdr_position_dir + name;

        // Write image to file
        ROS_DEBUG("Writing image to %s", whole.c_str());
        imwrite( hdr_position_dir + name, cv_bridge::toCvShare(msg, "bgr8")->image);
        // Increment image number
        IMAGE_NUMBER++;

        // Publish image capture ack
        std_msgs::UInt8 msg;
        msg.data = HDR_NODE_ID;
        exec_resp_pub.publish(msg);
    }
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

    ros::init(argc, argv, "executive_node", ros::init_options::NoSigintHandler);
    ros::NodeHandle n;
    ros::NodeHandle n_private("~");
    signal(SIGINT, sigIntHandler);

    // Override XMLRPC shutdown
    ros::XMLRPCManager::instance()->unbind("shutdown");
    ros::XMLRPCManager::instance()->bind("shutdown",shutdownCallback);
    shutdown_request = 0;

    int rate;
    bool LIGHTS_ON = false;
    double hdr_angle_offset;
    std::string exec_start_topic, exec_resp_topic, exec_command_topic;
    std::string status_topic, img_trig_topic, pwm_set_topic;

    // Configure directory strings
    n_private.param<std::string>("log_dir", base_directory, "");
    initializeDirectories();

    n.param<int>("lidar_node_id",  LIDAR_NODE_ID,  0);
    n.param<int>("camera_node_id", CAMERA_NODE_ID, 1);
    n.param<int>("imu_node_id",    IMU_NODE_ID,    2);
    n.param<int>("hdr_node_id",    HDR_NODE_ID,    3);
    n.param<int>("dyn_config_id",  DYN_NODE_ID,    4);
    n.param<int>("hdr_start_id",   HDR_START_ID,   10);
    n.param<int>("hdr_end_id",     HDR_END_ID,   11);
    n.param<int>("config_set_id",  CONFIG_SET_ID,  12);
    n.param<double>("scan_speed",    SCAN_SPEED,    0.5);
    n.param<double>("service_speed", SERVICE_SPEED, 2.0);
    n.param<double>("gear_ratio",    GEAR_RATIO,    0);
    n.param<double>("ext_ratio",     EXT_RATIO,     0);
    n.param<double>("rotator_cpr",   CPR,           0);
    n.param<double>("hdr_offset",    hdr_angle_offset, 0);
    n.param<double>("twoPi", twoPi, 0);

    n_private.param<int>("exec_rate",        rate,            10);
    n_private.param<bool>("enable_logging",  LOGGING_ENABLED, true);
    n_private.param<std::string>("log_dir",          base_directory,     "");
    n_private.param<std::string>("launch_dir",       root_launch_dir,    "~/ferret/launch");
    n_private.param<std::string>("exec_start_topic", exec_start_topic,   "/executive/start");
    n_private.param<std::string>("exec_resp_topic",  exec_resp_topic,    "/executive/response");
    n_private.param<std::string>("exec_cmd_topic",   exec_command_topic, "/executive/command");
    n_private.param<std::string>("status_topic",     status_topic,       "/node/status");
    n_private.param<std::string>("img_trig_topic",   img_trig_topic,     "/executive/img_trigger");
    n_private.param<std::string>("pwm_set_topic",    pwm_set_topic,      "/led_pwm");

    ros::Subscriber node_status_sub = n.subscribe(status_topic,       10,  nodeReadyCallback);
    ros::Subscriber command_sub     = n.subscribe(exec_command_topic, 1,   commandCallback);
    ros::Subscriber rotator_sub     = n.subscribe("/roboteq/rotator", 100, rotatorStatusCallback);
    ros::Subscriber homing_sub      = n.subscribe("/roboteq/home",    1,   homingCallback);
    ros::Subscriber img_trig_sub    = n.subscribe(img_trig_topic,     1,   imageTriggerCallback);
    ros::Subscriber img_sub         = n.subscribe("/camera/image_raw",1,   imageCallback);

    ros::Publisher start_pub = n.advertise<std_msgs::UInt8>(exec_start_topic, 10);
    ros::Publisher pub_rate  = n.advertise<std_msgs::Float32>("camera/frame_rate",1);
    ros::Publisher pwm_pub   = n.advertise<std_msgs::UInt8>(pwm_set_topic,1);

    roboteq_mode_pub       = n.advertise<std_msgs::UInt8>("/roboteq/mode", 1);
    exec_resp_pub          = n.advertise<std_msgs::UInt8>(exec_resp_topic, 100);
    roboteq_setpoint_pub   = n.advertise<roboteq_msgs::Command>("/roboteq/cmd",1);
    roboteq_script_restart = n.advertise<std_msgs::UInt8>("/roboteq/restartScript",1);

    // Start background logger
    logging = 0;
    logging_background = 0;
    start_background_bag();


    // Set loop rate
    ros::Rate loop_rate(10);

    std_msgs::UInt8 start_msg;
    std_msgs::UInt8 pwm_msg;

    // Wait for all nodes to boot into ros::ok() loops and homing to complete
    // while( !nodesReady() && !HOMING_COMPLETE ) {
    //     ros::spinOnce();
    //     loop_rate.sleep();
    // }

    // Enter main loop
    while( ros::ok() && !shutdown_request) {
        // check for new messages
        ros::spinOnce();

        // Process any commands
        if( current_command != last_command ) {
            last_command = current_command;
            processCommands();
        }

        // Act according to current state
        switch( current_state) {

            // Waiting for command
            case waiting:
                if( current_state != prior_state ) {
                    publish_comment("State: waiting");
                }

                prior_state = waiting;
                break;

            // Completed an action, wrap up and reinitialize
            case done:
                if( current_state != prior_state ) {
                    publish_comment("State: done");

                    // Stop data recording
                    if( LOGGING_ENABLED ) {
                        stop_bag();
                    }
    
                    // Make sure LEDs are off
                    pwm_msg.data = LED_OFF;
                    pwm_pub.publish(pwm_msg);

                    publish_comment("Resetting roator to home position...");
                    // Command motor to rotate towards home position
                    setRotationSpeed(SERVICE_SPEED, CW);
                }

                // Stop rotation at home position
                if( REVERSE_LIMIT ) {
                    setRotationSpeed(0, CW);
                    publish_comment("Rotator reset to home position. Waiting for next command...");
                    current_state = waiting;
                }

                prior_state = done;
                break;

            // Stops all action
            case estop:
                if( current_state != prior_state ) {
                    publish_comment("State: estop");
                }

                // Stop all motion
                setRotationSpeed(0, CW);

                // Make sure LEDs are off
                pwm_msg.data = LED_OFF;
                pwm_pub.publish(pwm_msg);

                prior_state = estop;
                current_state = waiting;
                break;

            // Initializes system for scanning
            case start_scan:
                if( current_state != prior_state ) {
                    publish_comment("State: start_scan");

                    // Ensure the device was homed before operation
                    if( !HOMING_COMPLETE ) {
                        ROS_WARN("Homing not comleted, initiating homing sequence!");
                        current_state = homing;
                        prior_state = start_scan;
                        entering_state = start_scan;
                        break;
                    } else {
                        // Create new scan directory to save the data in
                        initializeNewScanDirectory();
                    }

                    // Change camera frame rate to 20 fps
                    std_msgs::Float32 frate_msg;
                    frate_msg.data = 20.0;
                    pub_rate.publish(frate_msg);
                }

                // Ensure the scan is initiated from the reverse limit
                if( !REVERSE_LIMIT && current_motion != clockwise ) {
                    // Command motor to rotate towards home position
                    setRotationSpeed(SERVICE_SPEED, CW);
                    current_motion = clockwise;
                // Wait for reverse limit switch to be active
                } else if( REVERSE_LIMIT ) {
                    // Stop motion
                    setRotationSpeed(0, CW);
                    // Wait for motion to settle
                    spinFor(1);
                    // Start logging
                    if( LOGGING_ENABLED ) {
                        start_bag(BAG_TYPE_SCAN);
                    }
                    // Wait before starting scan
                    spinFor(1);
                    // Start scan rotation
                    ROS_DEBUG("Setting scan speed, %f", CCW*SCAN_SPEED);
                    setRotationSpeed(SCAN_SPEED, CCW);

                    // Command scaning speed
                    current_state = scanning;
                }

                prior_state = start_scan;
                break;

            // Monitor 360 degree lidar scan
            case scanning:
                if( current_state != prior_state ) {
                    publish_comment("State: scanning");
                }

                if( FORWARD_LIMIT ) {
                    // Stop scan rotation
                    ROS_DEBUG("Stopping at end of scan.");
                    setRotationSpeed(0, CW);
                    current_state = done;

                    if( LOGGING_ENABLED ) {
                        stop_bag();
                    }
                }

                prior_state = scanning;
                break;

            // Initiate sequence of HDR images at different angular offsets
            case hdr_init:
                if( current_state != prior_state ) {
                    publish_comment("State: hdr_init");
                    // Initialize state variables
                    HDR_STARTED = 0;
                    HDR_ENDED = 0;
                    TARGET_POSITION = 0;
                    IMAGE_NUMBER = 0;
                    POSITION_NUMBER = 0;

                    // Create new HDR directory
                    initializeNewHDRDirectory();
                    initializeNewHDRPositionDirectory();

                    // Change camera frame rate to 1 fps
                    std_msgs::Float32 frate_msg;
                    frate_msg.data = 10.0;
                    pub_rate.publish(frate_msg);
                }

                // Ensure the hdr image sequence is initiated from the reverse limit
                if( !REVERSE_LIMIT && current_motion != clockwise ) {
                    // Command motor to rotate towards home position
                    setRotationSpeed(SERVICE_SPEED, CW);
                    current_motion = clockwise;
                } else if( REVERSE_LIMIT ) {
                    // Stop motion
                    setRotationSpeed(0, CW);
                    // Wait for motion to settle
                    spinFor(1);
                    // Start logging
                    if( LOGGING_ENABLED ) {
                        start_bag(BAG_TYPE_HDR);
                    }
                    // Set position mode
                    setAngularPosition(TARGET_POSITION);
                    spinFor(2);
                    current_state = hdr_sequence;
                }

                prior_state = hdr_init;
                break;

            case hdr_sequence:
                if( current_state != prior_state ) {
                    publish_comment("State: hdr_sequence");
                }

                // Run while not at the endstop
                if( CUR_COUNTER < CPR && !FORWARD_LIMIT ) {
                    // Only take images when at the desired postion
                    if( abs(DIFF_COUNTER) <= 1 ) {
                        // Start HDR image sequence at position if not done so
                        if( !HDR_STARTED ) {
                            ROS_DEBUG("HDR sequence started");
                            // Transmit hdr start command
                            start_msg.data = HDR_START_ID;
                            start_pub.publish(start_msg);

                            if( !LIGHTS_ON ) {
                                // Make sure LEDs are on
                                pwm_msg.data = LED_ON;
                                pwm_pub.publish(pwm_msg);
                                LIGHTS_ON = true;
                            }

                        // Handle end of image sequence
                        } else if( HDR_ENDED ) {
                            ROS_DEBUG("HDR sequence ended");

                            // Make sure LEDs are off
                            pwm_msg.data = LED_OFF;
                            pwm_pub.publish(pwm_msg);
                            LIGHTS_ON = false;

                            // Move to next position
                            TARGET_POSITION += hdr_angle_offset;
                            ROS_DEBUG("Setting position to %f", TARGET_POSITION);
                            setAngularPosition(TARGET_POSITION);

                            // Sleep for a second to ensure movment has commenced before continuing
                            while( DIFF_COUNTER <= 1 ) {
                                spinFor(1);
                            }

                            HDR_ENDED = 0;
                            HDR_STARTED = 0;
                            IMAGE_NUMBER = 0;
                            POSITION_NUMBER++;
                            initializeNewHDRPositionDirectory();
                        }
                    }
                } else {
                    start_msg.data = HDR_END_ID;
                    start_pub.publish(start_msg);
                    current_state = done;

                    // Stop logging
                    if( LOGGING_ENABLED ) {
                        stop_bag();
                    }
                }

                prior_state = hdr_sequence;
                break;

            case homing:
                if( current_state != prior_state ) {
                    publish_comment("State: homing");

                    // Send command to start the homing sequence
                    startHomingSequence();
                }

                if( HOMING_COMPLETE ) {
                    if( entering_state == start_scan ) {
                        ROS_INFO("Homing complete, resuming scan");
                        current_state = start_scan;
                    }
                }

                prior_state = homing;
                break;


        } // swtich( current_state )

        ros::spinOnce();
        loop_rate.sleep();
    } // while( ros::ok() )

    std::cout << "Closing background logger..." << std::endl;
    stop_background_bag();
}

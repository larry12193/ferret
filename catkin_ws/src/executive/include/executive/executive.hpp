/*******************************************************************************
 * executive_node.hpp - Executive state machine header file
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
#include "executive/p_recorder.h"

#include <inttypes.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <sstream>
#include <fstream>
#include <time.h>
#include <sys/stat.h>
#include <signal.h>
#include <opencv2/highgui/highgui.hpp>
#include <boost/filesystem.hpp>


#ifndef EXECUTIVE_H
#define EXECUTIVE_H

#define CW  -1
#define CCW  1

#define LED_OFF 0
#define LED_ON  255

#define STOP_COMMAND        0   // Command ID to stop all processes
#define START_SCAN_COMMAND  1   // Command ID to start LIDAR scan
#define START_HDR_COMMAND   2   // Command ID to start HDR image sequence
#define STATUS_COMMAND      3   // Command ID to return scan status

// Define states that the robot can be in
//  waiting  - waiting to be commanded
//  done     - finished commanded task, cleaning up for next command
//  scanning - performing 360 degree LIDAR scan
//  hdr      - taking HDR images at set angular locations
enum State {waiting, done, estop, start_scan, scanning, hdr_init, hdr_sequence, homing};

// Define motor modes
enum Mode {closed_loop_velocity, closed_loop_count_pos};

// Define motion states of the rotator
enum Motion {clockwise, counter_clockwise, stopped};

class Executive {

public:
    Executive(ros::NodeHandle* n, ros::NodeHandle* np);
    ~Executive();

    void run();

private:

    void spinFor(int sec);

    void startHomingSequence();
    void homingCallback(const std_msgs::UInt8::ConstPtr& msg);
    void rotatorStatusCallback(const roboteq_msgs::FerretRotator::ConstPtr& msg);

    void setOperationalMode(Mode mode);
    void setRotationSpeed(float speed, int8_t dir);
    void setAngularPosition(double position);

    void publish_comment(const std::string& comment);

    uint8_t nodesReady();
    void nodeReadyCallback(const std_msgs::UInt8::ConstPtr& msg);

    void processCommands();
    void commandCallback(const std_msgs::UInt8::ConstPtr& msg);

    void start_hdr_bag();
    void stop_hdr_bag();
    void start_scan_bag();
    void stop_scan_bag();
    void start_background_bag();
    void stop_background_bag();

    void intializeLoggers();
    void extractTopicNames(PRecorderOptions* opts, std::string const* config);
    void initializeDirectories();
    void initializeNewScanDirectory();
    void initializeNewHDRDirectory();
    void initializeNewHDRPositionDirectory();

    void imageTriggerCallback(const std_msgs::UInt8::ConstPtr& msg);
    void imageCallback(const sensor_msgs::Image::ConstPtr& msg);

private:

    ros::NodeHandle _n;
    ros::NodeHandle _np;

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
    int rate;

    double SERVICE_SPEED;           // Speed (rad/s) during non-scan motion
    double SCAN_SPEED;              // Speed (rad/s) during scan motion
    double GEAR_RATIO;              // Gearbox ratio
    double EXT_RATIO;               // External gear ratio
    double CPR;                     // Counts per revolution of the motor
    double hdr_angle_offset;        // Angle offset between hdr sequences
    double twoPi;

    bool LIGHTS_ON = false;         // Whether the lights have been turned on

    uint8_t current_command = 0;    // Currnet command from user
    uint8_t last_command;           // Last user command

    std::string root_launch_dir;    // Path to ferret launch directory

    std::string base_directory;     // Log directory (ssd static directory)
    std::string dataset_directory;  // Specific dataset directory
    std::string scan_directory;     // Scan data subdirectory
    std::string hdr_directory;      // HDR image subdirectory
    std::string hdr_position_dir;   // Single position of HDR images subdirectory

    std::string exec_start_topic;   // Topic used to signal a node to start
    std::string exec_resp_topic;    // Topic used to respond to exec from node
    std::string exec_command_topic; // Topic used by operator to command robot action
    std::string status_topic;       // Node status topic
    std::string img_trig_topic;     // Trigger from HDR node for exec to save image
    std::string pwm_set_topic;      // Topic for setting LED PWM on arduino

    std::string hdr_topic_config;   // File location of topic config for HDR recorder
    std::string scan_topic_config;  // File location of topic config for scan recorder
    std::string bg_topic_config;    // File location of topic config for background recorder

    ros::Subscriber node_status_sub;        // Subscriber to node status topic
    ros::Subscriber command_sub;            // Subscriber to command topic
    ros::Subscriber rotator_sub;            // Subscriber to rotator status topic
    ros::Subscriber homing_sub;             // Subscriber to rotator homing topic
    ros::Subscriber img_trig_sub;           // Subscriber to image trigger topic
    ros::Subscriber img_sub;                // Subscriber to image topic

    ros::Publisher exec_resp_pub;           // Publisher for executive response
    ros::Publisher roboteq_mode_pub;        // Publisher for setting roboteq mode
    ros::Publisher roboteq_setpoint_pub;    // Publisher for setpoint
    ros::Publisher roboteq_script_restart;  // Publisher for restarting roboteq script
    ros::Publisher start_pub;               // Publiser for exec start topic
    ros::Publisher pub_rate;                // Publisher for camera frame rate
    ros::Publisher pwm_pub;                 // Publisher for LED PWM

    std_msgs::UInt8 start_msg;
    std_msgs::UInt8 pwm_msg;

    PRecorderOptions hdr_opts;       // HDR bag recorder options
    PRecorderOptions scan_opts;      // Scan bag recorder options
    PRecorderOptions background_opts;// Background bag recorder options

    PRecorder* hdr_recorder;         // HDR bag recorder
    PRecorder* scan_recorder;        // Scan bag recorder
    PRecorder* bg_recorder;          // Background bag recroder

    bool hdr_recording;                     // State of hdr recorder
    bool scan_recording;                    // State of scan recorder
    bool background_recording;              // State of background recorder

    State current_state;
    State prior_state;
    State entering_state;

    Mode current_mode;

    Motion current_motion;
};

#endif

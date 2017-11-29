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
#include <std_msgs/UInt8.h>
#include <std_msgs/String.h>
#include <roboteq_msgs/Command.h>
#include <roboteq_msgs/FerretRotator.h>

#include "executive/executive_node.hpp"

#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>

volatile uint8_t LIDAR_READY;	// State of LIDAR node
volatile uint8_t CAMERA_READY;	// State of camera node
volatile uint8_t IMU_READY;		// State of IMU node
volatile uint8_t HOMING_COMPLETE; // State of motor homing sequence
volatile uint8_t FORWARD_LIMIT; // State of forward limit switch
volatile uint8_t REVERSE_LIMIT; // State of reverse limit switch

bool LOGGING_ENABLED;           // Whether logging is enabled during run

int LIDAR_NODE_ID;			// LIDAR node id number
int CAMERA_NODE_ID;			// Camera node id number
int IMU_NODE_ID;			// IMU node id number

double SERVICE_SPEED;           // Speed (rad/s) during non-scan motion
double SCAN_SPEED;              // Speed (rad/s) during scan motion
double GEAR_RATIO;              // Gearbox ratio
double EXT_RATIO;               // External gear ratio

uint8_t current_command = 0;    // Currnet command from user
uint8_t last_command;           // Last user command

uint8_t logging = 0;            // State of rosbag logger

std::string root_launch_dir;    // Path to ferret launch directory

double twoPi;

#define CW  -1
#define CCW  1

ros::Publisher exec_resp_pub;     // Publisher for executive response
ros::Publisher roboteq_mode_pub;  // Publisher for setting roboteq mode
ros::Publisher roboteq_setpoint_pub; // Publisher for setpoint
ros::Publisher roboteq_script_restart;  // Publisher for restarting roboteq script

// Define states that the robot can be in
//  waiting  - waiting to be commanded
//  done     - finished commanded task, cleaning up for next command
//  scanning - performing 360 degree LIDAR scan
//  hdr      - taking HDR images at set angular locations
enum State {waiting, done, estop, start_scan, scanning, hdr, homing};
State current_state = waiting, prior_state = done, entering_state = done;

// Define motor modes
enum Mode {closed_loop_velocity, closed_loop_count_pos};
Mode current_mode = closed_loop_velocity;

// Define motion states of the rotator
enum Motion {clockwise, counter_clockwise, stopped};
Motion current_motion = stopped;

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
}

/* @brief Sets the rotational speed setpoint of the motor

*/
void setRotationSpeed(float speed, int8_t dir) {
    roboteq_msgs::Command cmd;

    // Ensure that the motor is in the proper mode
    if( current_mode == closed_loop_velocity ) {
        // Apply direction and convert RPM to rad/s
        cmd.setpoint = ((float)dir)*(speed*GEAR_RATIO*EXT_RATIO*(twoPi/60.0));
        cmd.mode = cmd.MODE_VELOCITY;
        roboteq_setpoint_pub.publish(cmd);
        // Spin a few times to get the command executed
        ros::spinOnce();
        ros::spinOnce();
        ros::spinOnce();

    } else {
        ROS_WARN("Attempted to set velocity while not in closed-loop velocity mode!");
    }
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
	return (LIDAR_READY && CAMERA_READY && IMU_READY);
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
	}

    // Republish message on executive response topic to ack
    exec_resp_pub.publish(msg);
}

/* @brief Processes incoming commands and sets the appropriate action in motion

*/
void processCommands() {
    switch( current_command ) {
        case STOP_COMMAND:
            publish_comment("Command: Recieved stop command.");
            break;
        case START_SCAN_COMMAND:
            publish_comment("Command: Recieved start scan command.");
            current_state = start_scan;
            break;
        case START_HDR_COMMAND:
            publish_comment("Command: Recieved start HDR command.");
            current_state = hdr;
            break;
        case STATUS_COMMAND:
            publish_comment("Command: Recieved status command.");
            break;
        default:
            publish_comment("Command: Recieved unknown command.");
            break;
    }
}

/* @brief Starts recording the rosbag with the appropriate topics

 */
void start_bag() {
    if( !logging && LOGGING_ENABLED ) {
        ROS_INFO("Starting logger...");
        // Change working directory to individual launch file directory
        chdir(root_launch_dir.c_str());

        // Construct system call string, running new instance in a detached screen terminal
        std::string sysCall = "tmux new -s record_bag -d 'roslaunch ~/ferret/launch/ferret_logger.launch'";

        // Make system call to run command
        system(sysCall.c_str());

        // Set logging flag
        logging = 1;
    }
}

/* @brief Stops recording rosbag
 */
void stop_bag() {
    // Check if logging is currently running and enabled
    if( logging && LOGGING_ENABLED ) {
        publish_comment("Stopping logger...");
        system("tmux kill-session -t record_bag ");
        logging = 0;
    }
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "executive_node");
	ros::NodeHandle n;
	ros::NodeHandle n_private("~");

	int rate;
	std::string exec_start_topic, exec_resp_topic, exec_command_topic;
    std::string status_topic;

	n.param<int>("lidar_node_id",  LIDAR_NODE_ID,  0);
	n.param<int>("camera_node_id", CAMERA_NODE_ID, 1);
	n.param<int>("imu_node_id",    IMU_NODE_ID,    2);

    // Rotator speeds (RPM)
    n.param<double>("scan_speed",    SCAN_SPEED,    0.5);
    n.param<double>("service_speed", SERVICE_SPEED, 2.0);
    n.param<double>("gear_ratio",    GEAR_RATIO,    0);
    n.param<double>("ext_ratio",     EXT_RATIO,     0);
    
    n.param<double>("twoPi", twoPi, 0);

    n_private.param<int>("exec_rate",        rate,            10);
    n_private.param<bool>("enable_logging",  LOGGING_ENABLED, true);
    n_private.param<std::string>("launch_dir",       root_launch_dir,    "~/ferret");
    n_private.param<std::string>("exec_start_topic", exec_start_topic,   "/executive/start");
    n_private.param<std::string>("exec_resp_topic",  exec_resp_topic,    "/executive/response");
    n_private.param<std::string>("exec_cmd_topic",   exec_command_topic, "/executive/command");
    n_private.param<std::string>("status_topic",     status_topic,       "/node/status");

	ros::Subscriber node_status_sub = n.subscribe(status_topic,       10,  nodeReadyCallback);
    ros::Subscriber command_sub     = n.subscribe(exec_command_topic, 1,   commandCallback);
    ros::Subscriber rotator_sub     = n.subscribe("/roboteq/rotator", 100, rotatorStatusCallback);
    ros::Subscriber homing_sub      = n.subscribe("/roboteq/home",    1,   homingCallback);

    ros::Publisher start_pub = n.advertise<std_msgs::UInt8>(exec_start_topic, 10);

    roboteq_mode_pub     = n.advertise<std_msgs::UInt8>("/roboteq/mode", 1);
    exec_resp_pub        = n.advertise<std_msgs::UInt8>(exec_resp_topic, 100);
    roboteq_setpoint_pub = n.advertise<roboteq_msgs::Command>("/roboteq/cmd",1);
    roboteq_script_restart = n.advertise<std_msgs::UInt8>("/roboteq/restartScript",1);

    // Set loop rate
	ros::Rate loop_rate(10);

	// Wait for all nodes to boot into ros::ok() loops and homing to complete
	// while( !nodesReady() && !HOMING_COMPLETE ) {
	// 	ros::spinOnce();
	// 	loop_rate.sleep();
	// }

	// Enter main loop
	while( ros::ok() ) {
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

                prior_state = estop;
				current_state = waiting;
				break;

            // Initializes system for scanning
            case start_scan:
                if( current_state != prior_state ) {
                    publish_comment("State: start_scan");
                }

                // Ensure the homing sequence completed successfully
                if( !HOMING_COMPLETE ) {
                    ROS_WARN("Homing not completed, initiating homing sequence");
                    current_state = homing;
                    prior_state = start_scan;
                    entering_state = start_scan;
                    break;
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
                    ros::Duration(2).sleep();
                    // Start logging
                    if( LOGGING_ENABLED ) {
                        start_bag();
                    }
                    // Wait before starting scan
                    ros::Duration(2).sleep();
                    // Start scan rotation
                    ROS_INFO("Setting scan speed, %f", CCW*SCAN_SPEED);
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
                    setRotationSpeed(0, CW);
                    current_state = done;
                }

				prior_state = scanning;
				break;

			// Initiate sequence of HDR images at different angular offsets
			case hdr:
				if( current_state != prior_state ) {
					publish_comment("State: hdr");
				}

                // Process any commands
                if( current_command != last_command ) {
                    last_command = current_command;
                    processCommands();
                }

				prior_state = hdr;
				break;

            case homing:
                if( current_state != prior_state ) {
                    publish_comment("State: homing");

                    // Send command to start the homing sequence
                    startHomingSequence();
                }

                if( HOMING_COMPLETE ) {
                    if( entering_state == start_scan ) {
                        publish_comment("Homing complete, resuming scan");
                        current_state = start_scan;
                    }
                }

                prior_state = homing;
                break;


		} // swtich( current_state )

		ros::spinOnce();
		loop_rate.sleep();
	} // while( ros::ok() )
}

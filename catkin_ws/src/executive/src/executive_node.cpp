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
#include <inttypes.h>

volatile uint8_t LIDAR_READY;	// State of LIDAR node
volatile uint8_t CAMERA_READY;	// State of camera node
volatile uint8_t IMU_READY;		// State of IMU node

uint8_t LIDAR_NODE_ID;			// LIDAR node id number
uint8_t CAMERA_NODE_ID;			// Camera node id number
uint8_t IMU_NODE_ID;			// IMU node id number

// Publisher for executive response
ros::Publisher exec_resp_pub;

// Define states that the robot can be in
enum State {waiting, done, scanning, hdr_collection};
State current_state = waiting, prior_state = done;

/* @brief Helper function to log comments

*/
void publish_comment(const std::string& comment)
{
    std_msgs::String msg;
    msg.data = comment;
    ROS_INFO("Executive Comment:%s",msg.data.c_str());

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
	if( msg->data == LIDAR_NODE_ID ) {
		LIDAR_READY = 1;
	} else if( msg->data == CAMERA_NODE_ID ) {
		CAMERA_READY = 1;
	} else if( msg->data == IMU_NODE_ID ) {
		IMU_READY = 1;
	}
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "executive_node");
	ros::NodeHandle n;
	ros::NodeHandle n_private("~");

	int rate;
	std::string exec_start_topic, exec_resp_topic,

	n_private.param<int>("lidar_node_id",  LIDAR_NODE_ID,  0);
	n_private.param<int>("camera_node_id", CAMERA_NODE_ID, 1);
	n_private.param<int>("imu_node_id",    IMU_NODE_ID,    2);
	n_private.param<int>("exec_rate",      rate,           10);

	ros::Subscriber node_status_sub = n.subscribe(exec_start_topic, 10, nodeReadyCallback);

	// Set loop rate
	ros::Rate loop_rate(10);
	
	// Wait for all nodes to boot into ros::ok() loops
	while( !nodesReady() ) {
		ros::spinOnce();
		loop_rate.sleep();
	}

	// Enter main loop
	while( ros::ok() ) {
		// check for new messages
		ros::spinOnce();

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
				}

				prior_state = done;
				current_state = waiting;
				break;

			// Initiate a 360 degree lidar scan
			case scanning:
				if( current_state != prior_state ) {
					publish_comment("State: scanning");
				}

				prior_state = scanning;
				break;

			// Initiate sequence of HDR images at different angular offsets
			case hdr_collection:
				if( current_state != prior_state ) {
					publish_comment("State: hdr_collection");
				}

				prior_state = hdr_collection;
				break;

		} // swtich( current_state )

		ros::spinOnce();
		loop_rate.sleep();
	} // while( ros::ok() )

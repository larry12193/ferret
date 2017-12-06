/*******************************************************************************
 * hdr.cpp - Implements HDR image collection
 *
 * Carnegie Mellon University
 * Author: Lawrence Papincak
 *
 *
 ******************************************************************************/

#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float32.h>

volatile bool start_hdr;
volatile bool exec_ack;
volatile bool dyn_config_ack;

int HDR_NODE_ID, HDR_START_ID, HDR_END_ID, CONFIG_SET_ID;

enum State {waiting, take_images};
State current_state = waiting;

/* @breif Callback for executive response

 */
void execRespCallback(const std_msgs::UInt8::ConstPtr& msg ) {
    if( msg->data == HDR_NODE_ID || msg->data == HDR_START_ID) {
        exec_ack = true;
    }
}

/* @brief Callback from other node status

 */
void nodeStatusCallback( const std_msgs::UInt8::ConstPtr& msg ) {
    if( msg->data == CONFIG_SET_ID ) {
        dyn_config_ack = true;
    }
}

/* @brief Callback for executive command that starts HDR

 */
void execStartCallback(const std_msgs::UInt8::ConstPtr& msg) {
    if( msg->data == HDR_START_ID ) {
        if( !start_hdr ) {
            start_hdr = true;
        }
    }
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "hdr_node");
	ros::NodeHandle n;
	ros::NodeHandle n_private("~");

    std::string exec_resp_topic, status_topic, img_trig_topic;
    double max_exp, min_exp, d_exp;
    // Import parameters
    n_private.param<std::string>("exec_resp_topic", exec_resp_topic, "/executive/response");
    n_private.param<std::string>("status_topic",    status_topic,    "/node/status");
    n_private.param<std::string>("img_trig_topic",  img_trig_topic,  "/executive/img_trigger");
    n_private.param<double>("max_exp", max_exp, 2);
    n_private.param<double>("min_exp", min_exp, -2);
    n_private.param<double>("d_exp",   d_exp, 1);
    n.param<int>("hdr_node_id", HDR_NODE_ID, 3);
    n.param<int>("hdr_start_id", HDR_START_ID, 10);
    n.param<int>("hdr_end_id", HDR_END_ID, 11);
    n.param<int>("config_set_id", CONFIG_SET_ID, 12);

    // Initialize executive response ack and hdr start flags as false
    exec_ack = false;
    start_hdr = false;

    // Setup subscribers and publishers
    ros::Subscriber sub_stat  = n.subscribe<std_msgs::UInt8>(status_topic,1,nodeStatusCallback);
    ros::Subscriber exec_start = n.subscribe<std_msgs::UInt8>("/executive/start",10,execStartCallback);
    ros::Subscriber exec_resp  = n.subscribe<std_msgs::UInt8>(exec_resp_topic,1,execRespCallback);
    ros::Publisher pub_exp    = n.advertise<std_msgs::Float32>("camera/exposure",1);
    ros::Publisher pub_status = n.advertise<std_msgs::UInt8>(status_topic, 10);
    ros::Publisher pub_trig   = n.advertise<std_msgs::UInt8>(img_trig_topic, 1);

    // Initialize exposure values
    uint8_t exp_index = 0;
    double current_exp = min_exp;
    bool firstExposure;
    // Define exposure message
    std_msgs::Float32 exp_msg;

    // Define node status message
    std_msgs::UInt8 status_msg;

    ros::Rate loop_rate(10);
    int loop_count = 0;

    while( ros::ok() ) {

        // Wait for executive ack to start HDR scan
        if( start_hdr ) {
            // Ack start HDR command
            status_msg.data = HDR_START_ID;
            pub_status.publish(status_msg);

            // Reset start flag for future runs
            start_hdr = false;
            firstExposure = true;
            // Run for all alloable exposures in range
            ROS_INFO("Min exp (%f) Max exp (%f) cur exp (%f)", min_exp, max_exp, current_exp);
            while( current_exp <= max_exp ) {
                ROS_INFO("Setting exp: %f", current_exp);
                // Send new exposure
                exp_msg.data = current_exp;
                pub_exp.publish(exp_msg);
                dyn_config_ack = false;

                // Wait for ack of expsure configuration
                while( !dyn_config_ack ) {
                    ros::spinOnce();
                    loop_rate.sleep();
                    if( loop_count >= 100 ) {
                        ROS_WARN("Failed to recieve ack from dynamic configure in time, resending setting...");
                        pub_exp.publish(exp_msg);
                        loop_count = 0;
                    }
                    loop_count++;
                }

                // Delay longer for first exposure to let the image settle
                if( firstExposure ) {
                    firstExposure = false;
                    for( int i = 0; i < 50; i++ ) {
                        ros::spinOnce();
                        loop_rate.sleep();
                    }
                }

                // Normal inter exposure delay
                ros::Rate lrate(10);
                for( int i = 0; i < 50; i++ ) {
                    ros::spinOnce();
                    lrate.sleep();
                }

                exec_ack = false;
                // Tell executive to take an image
                status_msg.data = 1;
                pub_trig.publish(status_msg);
                loop_count = 0;
                // Wait for executive to respond from image trigger
                while( !exec_ack ) {
                    ros::spinOnce();
                    loop_rate.sleep();

                    if( loop_count >= 100 ) {
                        ROS_WARN("Failed to recieve ack from executive in time, resending image trigger...");
                        pub_trig.publish(status_msg);
                        loop_count = 0;
                    }
                    loop_count++;
                }

                // Increment exposure
                current_exp += d_exp;
                ros::spinOnce();
                loop_rate.sleep();
            } // while( cur_exp)

            // Send status to executive that sequence is over
            status_msg.data = HDR_END_ID;
            pub_status.publish(status_msg);

            // Reset exposure setting
            current_exp = min_exp;

        } // if( start_hdr )
        ros::spinOnce();
        loop_rate.sleep();
    }
}

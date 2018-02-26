/*******************************************************************************
 * ferret_tf_broadcaster.cpp - Publishes rotator transform from encoder feedback
 *
 * Carnegie Mellon University
 * Author: Lawrence Papincak
 *
 *
 ******************************************************************************/

#include <ros/ros.h>
#include <tf/tf.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/Imu.h>
#include <roboteq_msgs/FerretRotator.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <iostream>

double cpr;
double twoPi;

/* @brief Callback for ferret rotator message with current count value

*/
void rotatorCallback(const roboteq_msgs::FerretRotator::ConstPtr& msg){

    // Define transform and its broadcaster
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;

    // Fill in data from message
    transformStamped.header.stamp = msg->header.stamp;
    transformStamped.header.frame_id = "rotator_link";
    transformStamped.child_frame_id = "rotator_static_link";

    // Set relative position to zero
    transformStamped.transform.translation.x = 0.0;
    transformStamped.transform.translation.y = 0.0;
    transformStamped.transform.translation.z = 0.0;

    // Calculate the yaw angle based on counts and find its associated quaternion
    double yaw = (-double(msg->count)/cpr) * twoPi;

    // Fill in transform quaternion
    transformStamped.transform.rotation = tf::createQuaternionMsgFromYaw(yaw);
    // transformStamped.transform.rotation.x = q.x;
    // transformStamped.transform.rotation.y = q.y;
    // transformStamped.transform.rotation.z = q.z;
    // transformStamped.transform.rotation.w = q.w;

    // Send off transform
    br.sendTransform(transformStamped);
}

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    if( msg->orientation_covariance[0] > 0.0 ) {
        // Define transform and its broadcaster
        static tf2_ros::TransformBroadcaster br;
        geometry_msgs::TransformStamped transformStamped;

        // Fill in data from message
        transformStamped.header.stamp = msg->header.stamp;
        transformStamped.header.frame_id = "world";
        transformStamped.child_frame_id = "rotator_link";

        // Set relative position to zero
        transformStamped.transform.translation.x = 0.0;
        transformStamped.transform.translation.y = 0.0;
        transformStamped.transform.translation.z = 0.0;

        geometry_msgs::Quaternion q;
        tf::Quaternion qmg(msg->orientation.x,msg->orientation.y,msg->orientation.z,msg->orientation.w);
        tf::Quaternion qtf(0,1,0,0);

        // Fill in transform quaternion
        tf::quaternionTFToMsg(qmg*qtf, transformStamped.transform.rotation);

        // Send off transform
        br.sendTransform(transformStamped);
    }
}

int main(int argc, char** argv){
    ros::init(argc, argv, "ferret_tf_broadcaster");
    ros::NodeHandle n;

    n.param<double>("twoPi", twoPi, 0);
    n.param<double>("rotator_cpr", cpr, 0);

    ros::Subscriber sub = n.subscribe("/roboteq/rotator", 10, &rotatorCallback);
    ros::Subscriber isub = n.subscribe("/imu/data", 1, &imuCallback);
    ros::spin();
    return 0;
};

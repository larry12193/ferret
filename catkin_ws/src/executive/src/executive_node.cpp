/*******************************************************************************
 * executive_node.cpp - Executive state machine that runs operational sequence
 *                      of the ferret
 *
 * Carnegie Mellon University
 * Author: Lawrence Papincak
 *
 *
 ******************************************************************************/

#include "executive/executive.hpp"

int main(int argc, char **argv) {

    ros::init(argc, argv, "executive_node");
    ros::NodeHandle n;
    ros::NodeHandle n_private("~");

    int rate;
    n_private.param("exec_rate", rate, 10);
    ros::Rate lrate(rate);

    // Define executive instance
    Executive exec(&n, &n_private);

    // Run while ros is running
    while( ros::ok() ) {
        // Cycle through state machine
        exec.run();
        // Do other ros stuff
        ros::spinOnce();
        lrate.sleep();
    }
}

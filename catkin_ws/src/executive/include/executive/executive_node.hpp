/*******************************************************************************
 * executive_node.hpp - Executive state machine header file
 *
 * Carnegie Mellon University
 * Author: Lawrence Papincak
 *
 *
 ******************************************************************************/

#ifndef EXECUTIVE_NODE_H
#define EXECUTIVE_NODE_H

#define CW  -1
#define CCW  1

#define BAG_TYPE_SCAN 0
#define BAG_TYPE_HDR  1

#define LED_OFF 0
#define LED_ON  255

#define STOP_COMMAND        0   // Command ID to stop all processes
#define START_SCAN_COMMAND  1   // Command ID to start LIDAR scan
#define START_HDR_COMMAND   2   // Command ID to start HDR image sequence
#define STATUS_COMMAND      3   // Command ID to return scan status

#endif

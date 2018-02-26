/*******************************************************************************
 * executive_node.cpp - Executive state machine that runs operational sequence
 *                      of the ferret
 *
 * Carnegie Mellon University
 * Author: Lawrence Papincak
 *
 *
 ******************************************************************************/

#include <executive/executive.hpp>

Executive::Executive(ros::NodeHandle* n, ros::NodeHandle* np) {
    _n  = (*n);
    _np = (*np);

    // Extract global parameter values
    _n.param<int>("lidar_node_id",    LIDAR_NODE_ID,    0);
    _n.param<int>("camera_node_id",   CAMERA_NODE_ID,   1);
    _n.param<int>("imu_node_id",      IMU_NODE_ID,      2);
    _n.param<int>("hdr_node_id",      HDR_NODE_ID,      3);
    _n.param<int>("dyn_config_id",    DYN_NODE_ID,      4);
    _n.param<int>("hdr_start_id",     HDR_START_ID,     10);
    _n.param<int>("hdr_end_id",       HDR_END_ID,       11);
    _n.param<int>("config_set_id",    CONFIG_SET_ID,    12);
    _n.param<double>("scan_speed",    SCAN_SPEED,       0.5);
    _n.param<double>("service_speed", SERVICE_SPEED,    2.0);
    _n.param<double>("gear_ratio",    GEAR_RATIO,       0);
    _n.param<double>("ext_ratio",     EXT_RATIO,        0);
    _n.param<double>("rotator_cpr",   CPR,              0);
    _n.param<double>("hdr_offset",    hdr_angle_offset, 0);
    _n.param<double>("twoPi",         twoPi,            0);

    // Extract private parameter values
    _np.param<int>("exec_rate",        rate,            10);
    _np.param<bool>("enable_logging",  LOGGING_ENABLED, true);
    _np.param<std::string>("log_dir",          base_directory,     "");
    _np.param<std::string>("launch_dir",       root_launch_dir,    "~/ferret/launch");
    _np.param<std::string>("exec_start_topic", exec_start_topic,   "/executive/start");
    _np.param<std::string>("exec_resp_topic",  exec_resp_topic,    "/executive/response");
    _np.param<std::string>("exec_cmd_topic",   exec_command_topic, "/executive/command");
    _np.param<std::string>("status_topic",     status_topic,       "/node/status");
    _np.param<std::string>("img_trig_topic",   img_trig_topic,     "/executive/img_trigger");
    _np.param<std::string>("pwm_set_topic",    pwm_set_topic,      "/led_pwm");
    _np.param<std::string>("hdr_topic_config", hdr_topic_config,   "");
    _np.param<std::string>("scan_topic_config",scan_topic_config,  "");
    _np.param<std::string>("bg_topic_config",  bg_topic_config,    "");

    current_state = waiting;
    prior_state = done;
    entering_state = done;

    current_mode = closed_loop_velocity;

    current_motion = stopped;

    // Initialize data directories for this instance of the executive
    initializeDirectories();

    // Setup logging
    intializeLoggers();

    // Start background bag logger
    start_background_bag();

    // Initialize all subscribers and publishers
    node_status_sub = _n.subscribe(status_topic,       10,  &Executive::nodeReadyCallback, this);
    rotator_sub     = _n.subscribe("/roboteq/rotator", 100, &Executive::rotatorStatusCallback, this);
    command_sub     = _n.subscribe(exec_command_topic, 1,   &Executive::commandCallback, this);
    homing_sub      = _n.subscribe("/roboteq/home",    1,   &Executive::homingCallback, this);
    img_trig_sub    = _n.subscribe(img_trig_topic,     1,   &Executive::imageTriggerCallback, this);
    img_sub         = _n.subscribe("/camera/image_raw",1,   &Executive::imageCallback, this);

    start_pub              = _n.advertise<std_msgs::UInt8>(exec_start_topic, 10);
    pub_rate               = _n.advertise<std_msgs::Float32>("camera/frame_rate",1);
    pwm_pub                = _n.advertise<std_msgs::UInt8>(pwm_set_topic,1);
    roboteq_mode_pub       = _n.advertise<std_msgs::UInt8>("/roboteq/mode", 1);
    exec_resp_pub          = _n.advertise<std_msgs::UInt8>(exec_resp_topic, 100);
    roboteq_setpoint_pub   = _n.advertise<roboteq_msgs::Command>("/roboteq/cmd",1);
    roboteq_script_restart = _n.advertise<std_msgs::UInt8>("/roboteq/restartScript",1);

    // Wait for nodes to come online
    // waitForSystemReady();
}

Executive::~Executive() {
    // Stop background logger
    stop_background_bag();
}

// void Executive::waitForSystemReady() {
//     std::string out;
//     ros::Rate lrate(1);
//     ros::Time start = ros::Time::now();
//
//     while( !nodesReady() ) {
//         if( !LIDAR_READY ) {
//             out += " LIDAR";
//         }
//         if( !CAMERA_READY ) {
//             out += " CAMERA";
//         }
//         if( !IMU_READY ) {
//             out += " IMU";
//         }
//
//         if( (ros::Time::now() - start) > ros::Duration(5) ) {
//             ROS_ERROR("System startup timeout due to nodes: %s", out)
//             ros::shutdown();
//         }
//
//         ROS_INFO("Waiting for nodes: %s",out.c_str());
//
//         out = "";
//         ros::spinOnce();
//         lrate.sleep();
//     }
// }

/* @brief Sets the loggers up for each type of data gathering event
 *
 */
void Executive::intializeLoggers() {
    // If config provided, extract topic names, else record all
    if( hdr_topic_config == "" ) {
        ROS_WARN("Recieved no topic names for HDR recorder, assuming all.");
        hdr_opts.record_all = true;
    } else {
        // Parse out all topic names from file for HDR
        extractTopicNames(&hdr_opts, &hdr_topic_config);
        // Sanity check on topic extraction
        if( hdr_opts.topics.empty() ) {
            hdr_opts.record_all = true;
        }
    }
    hdr_recording = false;

    // If config provided, extract topic names, else record all
    if( scan_topic_config == "" ) {
        ROS_WARN("Recieved no topic names for scan recorder, assuming all.");
        scan_opts.record_all = true;
    } else {
        // Parse out all topic names from file for HDR
        extractTopicNames(&scan_opts, &scan_topic_config);
        // Sanity check on topic extraction
        if( scan_opts.topics.empty() ) {
            scan_opts.record_all = true;
        }
    }
    scan_recording = false;

    // If config provided, extract topic names, else record all
    if( bg_topic_config == "" ) {
        ROS_WARN("Recieved no topic names for background recorder, assuming all.");
        background_opts.record_all = true;
    } else {
        // Parse out all topic names from file for HDR
        extractTopicNames(&background_opts, &bg_topic_config);
        // Sanity check on topic extraction
        if( background_opts.topics.empty() ) {
            background_opts.record_all = true;
        }
    }
    background_recording = false;
}

void Executive::extractTopicNames(PRecorderOptions* opts, std::string const* config) {

    // Define file stream for config file
    std::ifstream fd(config->c_str());
    std::string line;

    // Clear out any topics in the vector
    opts->topics.clear();

    // Make sure the file was opened properly before reading
    if( fd > 0 ) {
        while( std::getline(fd,line) ) {
            opts->topics.push_back(line);
        }
    } else {
        ROS_ERROR("Topic input file name invalid.");
        opts->record_all = true;
    }
}

 /* @breif Spins for a specific amount of seconds

  */
 void Executive::spinFor(int sec) {
     ros::Rate lrate(10);
     for( int i = 0; i < sec*10; i++ ) {
         ros::spinOnce();
         lrate.sleep();
     }
 }

 /* @brief Sends command to roboteq driver to restart the MicroBasic script that
           handles homing sequence and status reporting
  */
 void Executive::startHomingSequence() {
     std_msgs::UInt8 msg;
     msg.data = 1;
     roboteq_script_restart.publish(msg);
 }

 /* @brief Upates state of homing based on communication from roboteq

 */
 void Executive::homingCallback(const std_msgs::UInt8::ConstPtr& msg) {
     HOMING_COMPLETE = msg->data;
 }

 /* @brief Updates rotator limit switch information

 */
 void Executive::rotatorStatusCallback(const roboteq_msgs::FerretRotator::ConstPtr& msg) {
     FORWARD_LIMIT = msg->forward_limit;
     REVERSE_LIMIT = msg->reverse_limit;

     LAST_COUNTER = CUR_COUNTER;
     CUR_COUNTER = msg->count;
     DIFF_COUNTER = CUR_COUNTER - LAST_COUNTER;
 }

 /* @brief Changes the motor operational mode

  */
 void Executive::setOperationalMode(Mode mode) {
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
 void Executive::setRotationSpeed(float speed, int8_t dir) {
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
 void Executive::setAngularPosition(double position) {
     roboteq_msgs::Command cmd;

     // Ensure that the motor is in the proper mode
     if( current_mode != closed_loop_count_pos ) {
         setOperationalMode(closed_loop_count_pos);
         spinFor(5);
     }
     // Apply direction and convert RPM to rad/s
     cmd.setpoint = position*CPR/twoPi;
     ROS_INFO("Angular setpoint: %f",position*360.0/twoPi);
     cmd.mode = cmd.MODE_POSITION;
     roboteq_setpoint_pub.publish(cmd);
     // Spin a few times to get the command executed
     spinFor(1);
 }

 /* @brief Processes commands sent from control computer

 */
 void Executive::commandCallback(const std_msgs::UInt8::ConstPtr& msg) {
     last_command = current_command;
     current_command = msg->data;
 }

 /* @brief Helper function to log comments

 */
 void Executive::publish_comment(const std::string& comment)
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
 uint8_t Executive::nodesReady() {
     return (LIDAR_READY &&
             CAMERA_READY &&
             IMU_READY &&
             DYN_CONFIG_READY);
 }

 /* Processes messages from nodes when they are ready

 */
 void Executive::nodeReadyCallback(const std_msgs::UInt8::ConstPtr& msg) {

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
 void Executive::processCommands() {
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

 /* @brief Starts recording the rosbag with the appropriate topics

  */
void Executive::start_hdr_bag() {
    if( hdr_recording ) {
        ROS_WARN("HDR recorder already running, stopping and restarting new bag.");
        stop_hdr_bag();
    }
    // Set the record location
    hdr_opts.prefix = hdr_directory + "/hdr_bag";
    ROS_INFO("Saving HDR bag to %s", hdr_opts.prefix.c_str());
    // Initialize new instance of recorder
    hdr_recorder = new PRecorder(hdr_opts);

    // Start the recorder
    hdr_recorder->run();
    hdr_recording = true;
}

 /* @brief Stops recording rosbag
  *
  */
void Executive::stop_hdr_bag() {
    hdr_recorder->stop();
    hdr_recording = false;
}

 /* @brief Starts recording the rosbag with the appropriate topics
  *
  */
void Executive::start_scan_bag() {
    if( scan_recording ) {
        ROS_WARN("Scan recorder already running, stopping and restarting new bag.");
        stop_scan_bag();
    }
    // Set the record location
    scan_opts.prefix = scan_directory + "/scan_bag";
    ROS_INFO("Saving scan bag to %s", scan_opts.prefix.c_str());
    // Initialize new instance of recorder
    scan_recorder = new PRecorder(scan_opts);

    // Start the recorder
    scan_recorder->run();
    scan_recording = true;
}

 /* @brief Stops recording rosbag
  *
  */
void Executive::stop_scan_bag() {
    scan_recorder->stop();
    scan_recording = false;
}

/* @brief Starts recording the rosbag with the appropriate topics
 *
 */
void Executive::start_background_bag() {
   if( background_recording ) {
       ROS_WARN("Background recorder already running, stopping and restarting new bag.");
       stop_scan_bag();
   }
   // Set the record location
   background_opts.prefix = dataset_directory + "/background_bag";
   ROS_INFO("Saving background bag to %s", background_opts.prefix.c_str());
   // Initialize new instance of recorder
   bg_recorder = new PRecorder(background_opts);

   // Start the recorder
   bg_recorder->run();
   background_recording = true;
}

/* @brief Stops recording rosbag
 *
 */
void Executive::stop_background_bag() {
   bg_recorder->stop();
   background_recording = false;
}

 /* @brief Initializes the data set directory that all scan and HDR data will be recorded to

  */
 void Executive::initializeDirectories()
 {
     // Get the current date and time to append to folder
     time_t     now = time(0);
     struct tm  tstruct;
     char       buf[80];
     tstruct = *localtime(&now);
     strftime(buf, sizeof(buf), "%Y-%m-%d-%X", &tstruct);

     boost::filesystem::path base_dir(base_directory.c_str());
     if( !boost::filesystem::exists(base_dir) ) {
         ROS_INFO("Base directory does not exist, creating...");
         if(!boost::filesystem::create_directory(base_dir)) {
             ROS_ERROR("Failed to create base directory at %s", base_directory.c_str());
         }
     }

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
 void Executive::initializeNewScanDirectory()
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
 void Executive::initializeNewHDRDirectory()
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
 void Executive::initializeNewHDRPositionDirectory()
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
 void Executive::imageTriggerCallback(const std_msgs::UInt8::ConstPtr& msg) {
     IMAGE_TRIGGER = 1;
 }

 /* @brief Callback for camera images

  */
 void Executive::imageCallback(const sensor_msgs::Image::ConstPtr& msg) {
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

 /* @brief Main loop of state machine

  */
void Executive::run() {

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

                // Make sure LEDs are off
                pwm_msg.data = LED_ON;
                pwm_pub.publish(pwm_msg);
                // Wait for LED to come on
                spinFor(1);

                // Start logging
                if( LOGGING_ENABLED ) {
                    start_scan_bag();
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
                    stop_scan_bag();
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
                    start_hdr_bag();
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
                    stop_hdr_bag();
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
}

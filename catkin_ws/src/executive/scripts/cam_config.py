#!/usr/bin/env python

# -----------------------------------------------------------------------------
# cam_config.py - Dynamic reconfigure client definition for pointgrey camer
#
# Carnegie Mellon University
# Author: Lawrence Papincak
#
#------------------------------------------------------------------------------

import rospy
import dynamic_reconfigure.client

from std_msgs.msg import UInt8
from std_msgs.msg import Float32

exec_ack = False
dyn_node_id = 0
config_complete = 0;
resetCamera = False
sendAck = False

class CameraConfigure(object):

    global config_complete
    def __init__(self,frameRate):
        # Create client server
        self._client = dynamic_reconfigure.client.Client("/camera/camera_nodelet", timeout=5)

        # Set framerate and its configure topic
        self._client.update_configuration({'frame_rate' : frameRate})
        rospy.Subscriber("camera/frame_rate", Float32, self.frameRateCallback)
        self.status_pub = rospy.Publisher('/node/status', UInt8, queue_size=10)
        # Initialize subscriber to exposure message
        rospy.Subscriber("camera/exposure",   Float32, self.exposureCallback)
        rospy.Subscriber("camera/brightness", Float32, self.brightnessCallback)
        rospy.Subscriber("camera/gain",       Float32, self.gainCallback)
        rospy.Subscriber("camera/shutter",    Float32, self.shutterCallback)

        self._client.update_configuration({'auto_exposure': 'True'})
        self.aut_exp_state = True

    # Framerate message callback
    def frameRateCallback(self, msg):
        # Update framerate
        self._client.update_configuration({'frame_rate' : msg.data})

    # Exposure message callback
    def exposureCallback(self, msg):
        status_msg = UInt8()
        status_msg.data = config_complete

        self.status_pub.publish(status_msg)
        # Make sure auto_exposure is turned off
        if self.aut_exp_state == True:
            self._client.update_configuration({'auto_exposure': 'False'})
        # Update exposure
        self._client.update_configuration({'exposure' : msg.data})

    # Brighrness message callback
    def brightnessCallback(self, msg):
        # Update brightness
        self._client.update_configuration({'brightness' : config._brightness})

    # Gain message callback
    def gainCallback(self, msg):
        # Make sure auto_gain is turned off
        self._client.update_configuration({'auto_gain' : 'False'})
        # Update gain
        self._client.update_configuration({'gain' : msg.data})

    # Shutter message callback
    def shutterCallback(self, msg):
        # Make sure auto_shutter is turned off
        self._client.update_configuration({'auto_shutter' : 'False'})
        # Update exposure
        self._client.update_configuration({'shutter_speed' : msg.data})

# Callback for executive response topic
def execRespCallback(msg):
    global exec_ack
    global dyn_node_id

    if msg.data == dyn_node_id:
        exec_ack = True

# Callback for camera reset, resuming all auto settings
def cameraResetCallback(msg):
    resetCamera = True

if __name__ == '__main__':
    rospy.init_node("cam_config")

    # Fetch configuration parameters
    frameRate = rospy.get_param("~frame_rate",  7.0)
    loopRate  = rospy.get_param("~loop_rate",   20)

    dyn_node_id = rospy.get_param("dyn_config_id", 4)
    config_complete = rospy.get_param("config_complete_id", 12)

    # Initialize loop rate
    rate = rospy.Rate(loopRate)

    rospy.Subscriber("/executive/response", UInt8, execRespCallback)
    rospy.Subscriber("/camera/reset", UInt8, cameraResetCallback)

    status_msg = UInt8()
    status_msg.data = dyn_node_id

    # Define configuration object
    rospy.loginfo("Starting dynamic reconfigure node")
    config = CameraConfigure(frameRate)

    # # Wait for executive node to response with ack
    # while not exec_ack:
    #     status_pub.publish(status_msg)
    #     rate.sleep()


    # Run while ros is still running
    while not rospy.is_shutdown():
        # Handle camera resets
        if resetCamera == True:
            rospy.loginfo("reseting camera")
            config._client.update_configuration({'auto_exposure' : 'True'})
            config._client.update_configuration({'auto_gain' : 'True'})
            config._client.update_configuration({'auto_shutter' : 'True'})

        # Throttle loop speed
        rate.sleep()

    # Notify that the node is exiting
    rospy.loginfo("Ending dynamic reconfigure node")

# import rospy
# import dynamic_reconfigure.client
# from std_msgs.msg import Int8
#
# class CameraConfigure(object):
#
#     def __init__(self):
#         self._client = dynamic_reconfigure.client.Client("/camera/camera_nodelet", timeout=5)
#         self._setExposure = False
#         self._client.update_configuration({'auto_exposure': 'False'})
#         rospy.Subscriber("camera/exposure", Int8, self.exposureCallback)
#
#     def exposureCallback(self, msg):
#         self._setExposure = True
#         self._exposure = msg.data
#
# if __name__ == '__main__':
#     rospy.init_node("cam_config")
#
#     #rospy.loginfo("Waiting for pointgrey camera driver service")
#     #rospy.wait_for_service("pointgrey_camera_driver")
#     #rospy.loginfo("Found pointgrey camera driver service")
#
#     rate = rospy.Rate(20)
#
#     rospy.loginfo("Starting dynamic reconfigure node")
#     config = CameraConfigure()
#
#     while not rospy.is_shutdown():
#         # Check if exposure was set
#         if config._setExposure == True:
#             # Reset flag for next exposure message
#             config._setExposure = False
#             # Update exposure
#             config._client.update_configuration({'exposure' : config._exposure})
#
#         rate.sleep()
#
#     rospy.loginfo("Ending dynamic reconfigure node")

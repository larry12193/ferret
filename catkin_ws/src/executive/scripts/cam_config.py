#!/usr/bin/env python

import rospy
import dynamic_reconfigure.client
from std_msgs.msg import Int8

class CameraConfigure(object):

    def __init__(self):
        self._client = dynamic_reconfigure.client.Client("/camera/camera_nodelet", timeout=5)
        self._setExposure = False
        self._client.update_configuration({'auto_exposure': 'False'})
        rospy.Subscriber("camera/exposure", Int8, self.exposureCallback)

    def exposureCallback(self, msg):
        self._setExposure = True
        self._exposure = msg.data

if __name__ == '__main__':
    rospy.init_node("cam_config")

    #rospy.loginfo("Waiting for pointgrey camera driver service")
    #rospy.wait_for_service("pointgrey_camera_driver")
    #rospy.loginfo("Found pointgrey camera driver service")

    rate = rospy.Rate(20)

    rospy.loginfo("Starting dynamic reconfigure node")
    config = CameraConfigure()

    while not rospy.is_shutdown():
        # Check if exposure was set
        if config._setExposure == True:
            # Reset flag for next exposure message
            config._setExposure = False
            # Update exposure
            config._client.update_configuration({'exposure' : config._exposure})

        rate.sleep()

    rospy.loginfo("Ending dynamic reconfigure node")

#!/usr/bin/env python3

import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from dynamic_reconfigure.server import Server
from sensor_msgs.msg import Image

from ros_cv_tools import cv_tools
from ros_cv_tools.cfg import HSVConfig  # type: ignore

bridge = CvBridge()

hsv_low = np.array([0, 0, 0])
hsv_high = np.array([179, 255, 255])


# dynamic reconfigure callback
def dyn_rcfg_cb(config, level):
    global config_, hsv_low, hsv_high

    # Ensure low values don't exceed high values for HSV
    if config.h_low > config.h_high:
        config.h_low = config.h_high
    if config.s_low > config.s_high:
        config.s_low = config.s_high
    if config.v_low > config.v_high:
        config.v_low = config.v_high

    config_ = config

    hsv_low = np.array([config_.h_low, config_.s_low, config_.v_low])
    hsv_high = np.array([config_.h_high, config_.s_high, config_.v_high])

    return config


def image_callback(ros_image):
    """Image callback. ROS image --> CV image --> HSV --> binary --> BGR --> ROS image"""

    global config_
    try:
        cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")

        output_image = bridge.cv2_to_imgmsg(
            cv_tools.hsv_to_bgr(cv_tools.hsv_to_binary_mask(cv_tools.bgr_to_hsv(cv_image), hsv_low, hsv_high))
        )
        image_pub.publish(output_image)
    except CvBridgeError as e:
        print(e)
        return


if __name__ == "__main__":
    rospy.init_node("filter_hsv_binary", anonymous=True)

    image_pub = rospy.Publisher(rospy.get_param("~output_image_topic"), Image, queue_size=1)

    rospy.Subscriber(rospy.get_param("~input_image_topic"), Image, callback=image_callback, queue_size=1)
    Server(HSVConfig, dyn_rcfg_cb)

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

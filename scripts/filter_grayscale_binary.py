#!/usr/bin/env python3

import rospy
from cv_bridge import CvBridge, CvBridgeError
from dynamic_reconfigure.server import Server
from sensor_msgs.msg import Image
from ros_cv_tools import cv_tools

from ros_cv_tools.cfg import GrayscaleBinaryConfig

bridge = CvBridge()


# dynamic reconfigure callback
def dyn_rcfg_cb(config, level):
    global config_

    config_ = config

    return config


def image_callback(ros_image):
    """Image callback. ROS image --> CV image --> grayscale --> binary --> BGR --> ROS image"""
    global config_
    try:
        cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
    except CvBridgeError as e:
        print(e)
        return

    output_image = bridge.cv2_to_imgmsg(
        cv_tools.convert_gray_to_bgr(
            cv_tools.convert_gray_to_binary(cv_tools.convert_bgr_to_gray(cv_image), config_.threshold)
        ),
        encoding="rgb8",
    )
    image_pub.publish(output_image)


if __name__ == "__main__":
    rospy.init_node("filter_grayscale_binary", anonymous=True)

    image_pub = rospy.Publisher(rospy.get_param("~output_image_topic"), Image, queue_size=1)

    rospy.Subscriber(rospy.get_param("~input_image_topic"), Image, callback=image_callback, queue_size=1)
    Server(GrayscaleBinaryConfig, dyn_rcfg_cb)

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

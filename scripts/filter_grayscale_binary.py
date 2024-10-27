#!/usr/bin/env python3

import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from dynamic_reconfigure.server import Server
from sensor_msgs.msg import Image
from std_msgs.msg import Float32

from ros_cv_tools import cv_tools
from ros_cv_tools.cfg import GrayscaleBinaryConfig  # type: ignore

bridge = CvBridge()


# dynamic reconfigure callback
def dyn_rcfg_cb(config, level):
    global config_

    config_ = config

    return config


# Process images
def image_callback(ros_image):
    """Image callback. ROS image --> CV image --> grayscale --> binary --> BGR --> ROS image"""

    try:
        cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")

        ret, bw_image = cv2.threshold(
            cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY),
            config_.threshold,
            255,
            cv2.THRESH_BINARY,
        )

        if config_.white_percent:
            white_percent_pub.publish(cv_tools.white_percent(bw_image))

        image_pub.publish(bridge.cv2_to_imgmsg(cv2.cvtColor(bw_image, cv2.COLOR_GRAY2BGR), encoding="rgb8"))

    except CvBridgeError as e:
        print(e)
        return


if __name__ == "__main__":
    rospy.init_node("filter_grayscale_binary", anonymous=True)

    Server(GrayscaleBinaryConfig, dyn_rcfg_cb)

    image_pub = rospy.Publisher(rospy.get_param("~output_image_topic"), Image, queue_size=1)
    white_percent_pub = rospy.Publisher(rospy.get_param("~white_percent_topic"), Float32, queue_size=1)

    rospy.Subscriber(rospy.get_param("~input_image_topic"), Image, callback=image_callback, queue_size=1)

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

import cv2


def bgr_to_gray(cv_image):
    """Convert BGR image to grayscale"""

    return cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)


def gray_to_bgr(cv_image):
    """Convert grayscale image to BGR"""

    return cv2.cvtColor(cv_image, cv2.COLOR_GRAY2BGR)


def bgr_to_hsv(cv_image):
    """Convert BGR image to HSV"""

    return cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)


def hsv_to_bgr(cv_image):
    """Convert HSV image to BGR"""

    return cv2.cvtColor(cv_image, cv2.COLOR_HSV2BGR)


def gray_to_binary_mask(cv_image, threshold=127):
    """Convert grayscale image to binary. Still in grayscale range of 0-255"""

    return cv2.threshold(cv_image, threshold, 255, cv2.THRESH_BINARY)


def hsv_to_binary_mask(cv_image, min_array, max_array):
    """Convert HSV image to binary. Bounds are expected inside an numpy array in HSV format (H: 0-179, S: 0-255, V: 0-255)"""

    return cv2.inRange(cv_image, min_array, max_array)

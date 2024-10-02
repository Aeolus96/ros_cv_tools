import cv2


def convert_bgr_to_gray(cv_image):
    """Convert BGR image to grayscale"""

    return cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)


def convert_gray_to_binary(cv_image, threshold=127):
    """Convert grayscale image to binary. Still in grayscale range of 0-255"""

    return cv2.threshold(cv_image, threshold, 255, cv2.THRESH_BINARY)


def convert_gray_to_bgr(cv_image):
    """Convert grayscale image to BGR"""

    return cv2.cvtColor(cv_image, cv2.COLOR_GRAY2BGR)

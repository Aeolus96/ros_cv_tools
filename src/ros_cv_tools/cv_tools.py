import cv2
import numpy as np


def white_percent(cv_image):
    """Calculate the percentage of white pixels in a black-and-white image"""

    white_pixels = np.sum(cv_image == 255)
    total_pixels = cv_image.shape[0] * cv_image.shape[1]
    percentage = (white_pixels / total_pixels) * 100
    return round(percentage, 2)  # Return rounded percentage

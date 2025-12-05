import numpy as np
import cv2

def nothing(x):
    pass

def get_color_name(hsv_roi):
    """Returns a color name based on the mean HSV values of a region."""
    h, s, v = cv2.split(hsv_roi)
    h_mean = int(np.mean(h))
    s_mean = int(np.mean(s))
    v_mean = int(np.mean(v))

    if s_mean < 50 and v_mean > 200:
        return "White"
    elif v_mean < 50:
        return "Black"
    elif 0 <= h_mean < 10 or 160 <= h_mean <= 180:
        return "Red"
    elif 10 <= h_mean < 25:
        return "Orange"
    elif 25 <= h_mean < 35:
        return "Yellow"
    elif 35 <= h_mean < 85:
        return "Green"
    elif 85 <= h_mean < 130:
        return "Blue"
    elif 130 <= h_mean < 160:
        return "Purple"
    return "Unknown"

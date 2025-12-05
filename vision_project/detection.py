import cv2
import numpy as np
from utils import get_color_name

def detect_objects(frame, hsv, edges, min_area):
    """
    Detects shapes + colors and returns a list of detections:
    Each detection contains:
        (x, y, w, h, label)
    """

    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    detections = []

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < min_area:
            continue

        peri = cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, 0.04 * peri, True)
        x, y, w, h = cv2.boundingRect(approx)

        # ------ Shape detection ------
        sides = len(approx)
        if sides == 3:
            shape = "Triangle"
        elif sides == 4:
            aspect_ratio = w / float(h)
            shape = "Square" if 0.9 < aspect_ratio < 1.1 else "Rectangle"
        elif sides > 4:
            shape = "Circle"
        else:
            shape = "Unknown"

        # ------ Color detection ------
        hsv_roi = hsv[y:y + h, x:x + w]
        color = get_color_name(hsv_roi)

        label = f"{color} {shape}"
        detections.append((x, y, w, h, label))

    return detections
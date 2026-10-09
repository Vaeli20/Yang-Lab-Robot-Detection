# motion.py
# Simple motion detection using MOG2 background subtractor + contour filtering.
import cv2
import numpy as np

class MotionDetector:
    def __init__(self, history=5, area_threshold=500):
        self.bg = cv2.createBackgroundSubtractorMOG2(history=500, varThreshold=16, detectShadows=True)
        self.history = history
        self.area_threshold = area_threshold

    def detect(self, frame):
        """
        Input: BGR frame
        Output: motion_mask (uint8), motion_found (bool), list_of_bounding_boxes
        """
        fgmask = self.bg.apply(frame)
        # morphological cleaning
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3))
        fgmask = cv2.morphologyEx(fgmask, cv2.MORPH_OPEN, kernel, iterations=1)
        fgmask = cv2.morphologyEx(fgmask, cv2.MORPH_DILATE, kernel, iterations=2)
        contours, _ = cv2.findContours(fgmask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        boxes = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < self.area_threshold:
                continue
            x,y,w,h = cv2.boundingRect(cnt)
            boxes.append((x,y,x+w,y+h))
        motion_found = len(boxes) > 0
        return fgmask, motion_found, boxes

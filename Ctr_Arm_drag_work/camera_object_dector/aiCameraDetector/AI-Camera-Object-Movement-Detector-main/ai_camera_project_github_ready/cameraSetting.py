import cv2
import numpy as np
    
def adjust_saturation(frame, sat_scale=1.3):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)

    # scale saturation
    s = np.clip(s.astype(np.float32) * sat_scale, 0, 255).astype(np.uint8)

    hsv = cv2.merge([h, s, v])
    return cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)

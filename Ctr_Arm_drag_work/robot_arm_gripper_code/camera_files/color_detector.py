import cv2
import numpy as np

COLOR_RANGES = {
    "Red1":    [([0, 120, 70],   [10, 255, 255])],
    "Red2":    [([170, 120, 70], [180, 255, 255])],

    "Orange":  [([10, 100, 100], [20, 255, 255])],   # Between red and yellow
    "Yellow":  [([20, 100, 100], [30, 255, 255])],
    "Green":   [([40, 40, 40],   [80, 255, 255])],
    "Cyan":    [([80, 100, 100], [100, 255, 255])],
    "Blue":    [([90, 60, 60],   [130, 255, 255])],
    "Purple":  [([130, 100, 100], [150, 255, 255])], # Between blue and magenta
    "Magenta": [([140, 100, 100], [160, 255, 255])],
    "Pink":    [([160, 100, 150], [170, 255, 255])], # Light magenta tones
    "Brown":   [([10, 100, 20],  [20, 255, 150])],  # Dark orange

    "White":   [([0, 0, 200],    [180, 30, 255])],   # Low saturation, high value
    "Gray":    [([0, 0, 50],     [180, 30, 200])],   # Low saturation, mid value
    "Black":   [([0, 0, 0],      [180, 255, 50])]    # Low value}
}

def get_dominant_color(roi_bgr):
    """
    Analyzes a BGR region of interest (ROI) crop and returns 
    the color name with the highest pixel coverage.
    """
    if roi_bgr.size == 0:
        return "Unknown"

    # Smooth crop and convert to HSV
    blurred = cv2.GaussianBlur(roi_bgr, (5, 5), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    
    max_pixels = 0
    dominant_color = "Unknown"

    for color_name, ranges in COLOR_RANGES.items():
        total_mask = np.zeros(hsv.shape[:2], dtype="uint8")
        
        for lower, upper in ranges:
            lower = np.array(lower, dtype="uint8")
            upper = np.array(upper, dtype="uint8")
            mask = cv2.inRange(hsv, lower, upper)
            total_mask = cv2.bitwise_or(total_mask, mask)

        # Count how many pixels fall into this color range
        pixel_count = cv2.countNonZero(total_mask)

        if pixel_count > max_pixels:
            max_pixels = pixel_count
            dominant_color = color_name

    # Require at least 5% of the crop to match a color to avoid false positives
    roi_area = roi_bgr.shape[0] * roi_bgr.shape[1]
    if max_pixels < (roi_area * 0.05):
        return "Unknown"
    return dominant_color




   
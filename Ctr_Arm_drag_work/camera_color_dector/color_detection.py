import cv2
import numpy as np

cap = cv2.VideoCapture(1)

color_ranges = {
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
kernel = np.ones((5, 5), np.uint8)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Apply Gaussian Blur to smooth out pixel noise before HSV conversion
    blurred = cv2.GaussianBlur(frame, (5, 5), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    for group_name, ranges in color_ranges.items():
        # Initialize an empty mask for the group
        combined_mask = np.zeros(hsv.shape[:2], dtype="uint8")

        # Combine all sub-ranges belonging to this group (e.g., Red1 + Red2)
        for (lower, upper) in ranges:
            lower = np.array(lower, dtype="uint8")
            upper = np.array(upper, dtype="uint8")
            
            mask = cv2.inRange(hsv, lower, upper)
            combined_mask = cv2.bitwise_or(combined_mask, mask)

        # Morphological Closing: fills internal holes and merges adjacent blobs
        combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_CLOSE, kernel)

        # Find external contours for the merged mask
        contours, _ = cv2.findContours(combined_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 1000:  # Raised threshold to ignore small fragmented patches
                x, y, w, h = cv2.boundingRect(contour)
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.putText(frame, group_name, (x, y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    cv2.imshow("Grouped Color Detection", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
import cv2
import numpy as np

cap = cv2.VideoCapture(1)

color_ranges = {
    "Red1":    ([0, 120, 70],   [10, 255, 255]),
    "Red2":    ([170, 120, 70], [180, 255, 255]),

    "Orange":  ([10, 100, 100], [20, 255, 255]),   # Between red and yellow
    "Yellow":  ([20, 100, 100], [30, 255, 255]),
    "Green":   ([40, 40, 40],   [80, 255, 255]),
    "Cyan":    ([80, 100, 100], [100, 255, 255]),
    "Blue":    ([90, 60, 60],   [130, 255, 255]),
    "Purple":  ([130, 100, 100], [150, 255, 255]), # Between blue and magenta
    "Magenta": ([140, 100, 100], [160, 255, 255]),
    "Pink":    ([160, 100, 150], [170, 255, 255]), # Light magenta tones
    "Brown":   ([10, 100, 20],  [20, 255, 150]),  # Dark orange

    "White":   ([0, 0, 200],    [180, 30, 255]),   # Low saturation, high value
    "Gray":    ([0, 0, 50],     [180, 30, 200]),   # Low saturation, mid value
    "Black":   ([0, 0, 0],      [180, 255, 50])    # Low value}
}
kernel = np.ones((5, 5), np.uint8)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    for color_name, (lower, upper) in color_ranges.items():
        lower = np.array(lower, dtype="uint8")
        upper = np.array(upper, dtype="uint8")

        mask = cv2.inRange(hsv, lower, upper)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 800:
                x, y, w, h = cv2.boundingRect(contour)
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.putText(frame, color_name, (x, y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

    cv2.imshow("Color Detection", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
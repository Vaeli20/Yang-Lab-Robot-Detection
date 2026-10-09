import numpy as np
import cv2
from matplotlib import pyplot as plt
import cvlib as cv

# Initialize the webcam (0 is usually the default camera)
cap = cv2.VideoCapture(0)

# Set the resolution (optional)

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

program_status = 'c'

print("enter 'q' to exit the program.")
while True:
  # Read frame by frame
  ret, frame = cap.read()
  if not ret:
    break

  # Get the height and width of the frame to find the center pixel
  height, width, _ = frame.shape
  cx, cy = int(width / 2), int(height / 2)

  # Convert BGR frame to HSV color space for better color tracking
  hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

  # Get HSV and BGR values of the center pixel
  pixel_hsv = hsv_frame[cy, cx]
  pixel_bgr = frame[cy, cx]

  blue, green, red = int(pixel_bgr[0]), int(pixel_bgr[1]), int(pixel_bgr[2])
  hue, saturation, value = (
      int(pixel_hsv[0]),
      int(pixel_hsv[1]),
      int(pixel_hsv[2]),
  )

  # Basic color name matching based on Hue value (HSV scale in OpenCV is 0-179)
  color_name = "Unknown"
  if saturation <= 5 and value <= 15:
    if hue <= 30:
          color_name = "Dark Gray"
    elif hue <= 50:
              color_name = "Gray"
    elif hue <= 75:
            color_name = "Light Gray"
  elif saturation < 20 and value > 200:
    color_name = "White"
  elif value < 50:
    color_name = "Black"
  elif hue < 10 or hue > 165:
    color_name = "Red"
  elif 10 <= hue < 22:
    color_name = "Orange"
  elif 22 <= hue < 38:
    color_name = "Yellow"
  elif 38 <= hue < 75:
    color_name = "Green"
  elif 75 <= hue < 130:
    color_name = "Blue"
  elif 130 <= hue <= 165:
    color_name = "Purple"

  # Draw a crosshair target at the center of the screen
  cv2.circle(frame, (cx, cy), 5, (0, 255, 0), 2)
  cv2.rectangle(frame, (cx - 20, cy - 20), (cx + 20, cy + 20), (0, 255, 0), 1)

  # Display the detected color name and RGB values on the screen
  text = f"{color_name} R:{red} G:{green} B:{blue}"
  cv2.putText(
      frame, text, (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2
  )

  # Show the live feed window
  cv2.imshow("Color Detector Camera", frame)

  # Press 'q' key to stop the loop
  if cv2.waitKey(1) & 0xFF == ord("q"):
      break


# Release the camera and close all windows
cap.release()
cv2.destroyAllWindows()
    
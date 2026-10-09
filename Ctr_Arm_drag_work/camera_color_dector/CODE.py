import cv2
import numpy as np

frame = None
clicked = False
click_position = (0, 0)
color_text = ""

def get_basic_color_name(r, g, b):
    if r > 200 and g > 200 and b > 200:
        return "White"
    elif r < 50 and g < 50 and b < 50:
        return "Black"
    elif abs(r - g) < 15 and abs(g - b) < 15:
        return "Gray"
    elif r > g and r > b:
        if g > 100:
            return "Yellow"
        elif b > 100:
            return "Magenta"
        else:
            return "Red"
    elif g > r and g > b:
        if r > 100:
            return "Yellow"
        elif b > 100:
            return "Cyan"
        else:
            return "Green"
    elif b > r and b > g:
        if r > 100:
            return "Magenta"
        elif g > 100:
            return "Cyan"
        else:
            return "Blue"
    else:
        return "Unknown"


def pick_color(event, x, y, flags, param):
    global frame, clicked, click_position, color_text
    if event == cv2.EVENT_LBUTTONDOWN and frame is not None:
        clicked = True
        click_position = (x, y)
        b, g, r = frame[y, x]
        name = get_basic_color_name(r, g, b)
        color_text = f"{name} (R={r} G={g} B={b})"
        print(color_text)


cap = cv2.VideoCapture(0)
cv2.namedWindow("Color Recognition")
cv2.setMouseCallback("Color Recognition", pick_color)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    if clicked:
        x, y = click_position
        b, g, r = frame[y, x]
        cv2.rectangle(frame, (x, y - 20), (x + 250, y), (int(b), int(g), int(r)), -1)
        cv2.putText(frame, color_text, (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (255 - int(b), 255 - int(g), 255 - int(r)), 1, cv2.LINE_AA)

    cv2.imshow("Color Recognition", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()


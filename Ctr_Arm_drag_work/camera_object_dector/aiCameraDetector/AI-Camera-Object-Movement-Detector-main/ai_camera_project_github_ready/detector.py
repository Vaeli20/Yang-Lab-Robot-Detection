# detector.py
# Wraps ultralytics YOLO detection model and drawing helpers.
from ultralytics import YOLO
import cv2
import numpy as np

class Detector:
    def __init__(self, model_name, conf, iou, imgsz):
        self.model_name = model_name
        self.conf = conf
        self.iou = iou
        self.imgsz = imgsz
        self.model = YOLO(model_name)

    def predict(self, frame):
        """
        Run inference on a single BGR frame (numpy array).
        Returns a list of detections: dicts with keys: box (x1,y1,x2,y2), conf, class_id, label
        """
        if frame is None or not isinstance(frame, np.ndarray):
            return []

        # ultralytics expects RGB images
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        try:
            results = self.model.predict(source=rgb, imgsz=self.imgsz, conf=self.conf, iou=self.iou, verbose=False)
        except Exception:
            return []

        # results may be a single Results object or a list; handle both
        r = results[0] if isinstance(results, (list, tuple)) and len(results) > 0 else results

        out = []
        if not hasattr(r, 'boxes') or r.boxes is None:
            return out

        # extract arrays (handle torch tensors or plain lists)
        try:
            xyxy_arr = r.boxes.xyxy
            if hasattr(xyxy_arr, 'cpu'):
                xyxy_arr = xyxy_arr.cpu().numpy()
            else:
                xyxy_arr = np.array(xyxy_arr)
        except Exception:
            xyxy_arr = np.zeros((0,4))

        try:
            conf_arr = r.boxes.conf
            if hasattr(conf_arr, 'cpu'):
                conf_arr = conf_arr.cpu().numpy()
            else:
                conf_arr = np.array(conf_arr)
        except Exception:
            conf_arr = np.zeros((len(xyxy_arr),))

        try:
            cls_arr = r.boxes.cls
            if hasattr(cls_arr, 'cpu'):
                cls_arr = cls_arr.cpu().numpy()
            else:
                cls_arr = np.array(cls_arr)
        except Exception:
            cls_arr = np.zeros((len(xyxy_arr),), dtype=int)

        # label names: prefer result-level mapping, fallback to model names
        names = getattr(r, 'names', None) or getattr(self.model, 'names', {}) or {}

        h, w = frame.shape[:2]
        n = len(xyxy_arr)
        for i in range(n):
            xy = xyxy_arr[i]
            # ensure shape and numeric values
            if len(xy) < 4:
                continue
            x1, y1, x2, y2 = int(max(0, np.floor(xy[0]))), int(max(0, np.floor(xy[1]))), int(min(w - 1, np.ceil(xy[2]))), int(min(h - 1, np.ceil(xy[3])))
            conf = float(conf_arr[i]) if i < len(conf_arr) else 0.0
            cls_id = int(cls_arr[i]) if i < len(cls_arr) else -1
            label = str(names.get(cls_id, cls_id))
            out.append({
                "box": [x1, y1, x2, y2],
                "conf": conf,
                "class_id": cls_id,
                "label": label
            })

        return out

def draw_detections(frame, detections, motion_mask=None):
    """
    Draw bounding boxes and minimal info on frame (BGR).
    If motion_mask is provided, boxes overlapping motion regions will be emphasized.
    """
    if frame is None or not isinstance(frame, np.ndarray):
        return frame

    h, w = frame.shape[:2]
    overlay = frame.copy()
    for det in detections:
        x1, y1, x2, y2 = det['box']
        # clamp box coordinates
        x1, y1 = max(0, x1), max(0, y1)
        x2, y2 = min(w - 1, x2), min(h - 1, y2)
        label = det.get('label', '')
        conf = det.get('conf', 0.0)

        # Decide if this box overlaps motion mask
        is_moving = False
        if motion_mask is not None:
            xx1, yy1 = x1, y1
            xx2, yy2 = x2, y2
            if yy2 >= yy1 and xx2 >= xx1:
                roi = motion_mask[yy1:yy2+1, xx1:xx2+1]
                if roi.size > 0 and float(np.mean(roi)) > 10:
                    is_moving = True

        # color: green for normal, orange for person, blue if moving (overrides)
        color = (0, 255, 0)
        if str(label).lower() == 'person':
            color = (0, 128, 255)
        if is_moving:
            color = (255, 0, 0)

        thickness = 2 if not is_moving else 3
        cv2.rectangle(overlay, (x1, y1), (x2, y2), color, thickness)

        # minimal info text: label + confidence (rounded)
        info = f"{label} {conf:.2f}"
        (tw, th), _ = cv2.getTextSize(info, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        tx1 = x1
        ty1 = max(0, y1 - th - 6)
        tx2 = min(w - 1, x1 + tw + 6)
        ty2 = y1
        cv2.rectangle(overlay, (tx1, ty1), (tx2, ty2), color, -1)
        text_pos = (x1 + 3, max(ty1 + th, 4))
        cv2.putText(overlay, info, text_pos, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)

    # blend overlay onto frame
    cv2.addWeighted(overlay, 0.9, frame, 0.1, 0, frame)
    return frame

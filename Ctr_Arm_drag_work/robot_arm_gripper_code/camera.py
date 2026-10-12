import os
import cv2
import yaml
import torch

from PyQt5.QtCore import QThread, pyqtSignal, Qt
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QGroupBox, QLabel, QPushButton

from camera_files.detector import Detector, draw_detections
from camera_files.motion import MotionDetector
from camera_files.color_detector import get_dominant_color
from camera_files.cameraSetting import adjust_saturation

class CameraWork(QThread):
    frame_ready = pyqtSignal(QImage)
    log_msg = pyqtSignal(str)

    def __init__(self, config_path='config.yaml'):
        super().__init__()
        self.running = False
        self.cfg = self.load_config(config_path)

    def load_config(self, path='config.yaml'):
        # Find config.yaml inside the camera_files directory
        script_dir = os.path.dirname(os.path.abspath(__file__))
        config_path = os.path.join(script_dir, 'camera_files', path)

        if not os.path.exists(config_path):
            raise FileNotFoundError(
                f"Configuration file not found: {config_path}"
            )

        with open(config_path, 'r') as f:
            return yaml.safe_load(f)

    def run(self):
        self.running = True
        cap = None
        try:

            model_name = self.cfg.get('model', self.cfg.get('model'))
            conf_th = float(self.cfg['conf_threshold'])        
            iou_th = float(self.cfg['iou_threshold'])
            motion_history = int(self.cfg['motion_history'])
            motion_area_threshold = int(self.cfg['motion_area_threshold'])
            
            detector = Detector(model_name=model_name, conf=conf_th, iou=iou_th, imgsz=self.cfg['imgsz'])
            motion = MotionDetector(history=motion_history, area_threshold=motion_area_threshold)
            
            cap = cv2.VideoCapture(1)
            if not cap.isOpened():
                self.log_msg.emit("ERROR: Could not open camera.")
                return

            while self.running:
                ret, frame = cap.read()
                if not ret:
                    break

                # Adjust frame properties based on config
                frame = cv2.convertScaleAbs(frame, alpha=self.cfg['contrast_alpha'], beta=self.cfg['brightness_beta'])
                frame = adjust_saturation(frame, sat_scale=self.cfg['saturation_scale'])
                
                fgmask, motion_found, motion_boxes = motion.detect(frame)
                detections = detector.predict(frame)

                for det in detections:
                    # Assumes det provides bounding box coordinates: [x1, y1, x2, y2]
                    # Adjust indexing if your detector output structure is different (e.g. det['bbox'])
                    x1, y1, x2, y2 = map(int, det['box'])

                    # Ensure coordinates stay within frame bounds
                    x1, y1 = max(0, x1), max(0, y1)
                    x2, y2 = min(frame.shape[1], x2), min(frame.shape[0], y2)

                    # Crop object region and get dominant color
                    roi = frame[y1:y2, x1:x2]
                    color_name = get_dominant_color(roi)

                    # Append color name to detection label/class name if available
                    if hasattr(det, 'label'):
                        det.label = f"{det.label} ({color_name})"
                    elif isinstance(det, dict) and 'label' in det:
                        det['label'] = f"{det['label']} ({color_name})"

                out = draw_detections(frame, detections, motion_mask=fgmask)
                
                # annotate motion boxes
                if motion_found:
                    for (x1, y1, x2, y2) in motion_boxes:
                        cv2.rectangle(out, (x1, y1), (x2, y2), (0, 0, 255), 1)

                # Convert frame to QImage format for PyQt
                rgb_image = cv2.cvtColor(out, cv2.COLOR_BGR2RGB)
                h, w, ch = rgb_image.shape
                qt_image = QImage(rgb_image.data, w, h, ch * w, QImage.Format_RGB888)
                self.frame_ready.emit(qt_image)
        except Exception as e:
            self.log_msg.emit(f"Camera detector error: {e}")

        finally:
            self.running = False
            if cap is not None:
                cap.release()

    def stop(self):
        self.running = False
        if self.isRunning():
            self.wait(2000)

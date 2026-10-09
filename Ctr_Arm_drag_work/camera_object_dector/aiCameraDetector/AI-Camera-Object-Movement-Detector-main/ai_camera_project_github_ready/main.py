# main.py
# Entry point for the AI camera detector.
import cv2
import yaml
import os
from detector import Detector, draw_detections
from motion import MotionDetector
from utils import save_snapshot
from color_detector import get_dominant_color
from cameraSetting import adjust_saturation

def load_config(path='config.yaml'):
    # Absolute path check ensures config.yaml is found reliably
    script_dir = os.path.dirname(os.path.abspath(__file__))
    config_path = os.path.join(script_dir, path)
    
    if not os.path.exists(config_path):
        config_path = path  # Fallback to current working directory
        
    with open(config_path, 'r') as f:
        return yaml.safe_load(f)

def main():
    cfg = load_config()
    model_name = cfg.get('model', cfg['model'])
    conf_th = float(cfg.get('conf_threshold', cfg['conf_threshold']))
    iou_th = float(cfg.get('iou_threshold', cfg['iou_threshold']))
    motion_history = int(cfg.get('motion_history', cfg['motion_history']))
    motion_area_threshold = int(cfg.get('motion_area_threshold', cfg['motion_area_threshold']))

    detector = Detector(model_name=model_name, conf=conf_th, iou=iou_th, imgsz=cfg['imgsz'])
    motion = MotionDetector(history=motion_history, area_threshold=motion_area_threshold)

    cap = cv2.VideoCapture(1)
    if not cap.isOpened():
        print('ERROR: Could not open camera.')
        return

    print('Press q to quit, s to save a snapshot.')
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # Adjust brightness and contrast, then increase saturation for better color detection
        frame = cv2.convertScaleAbs(frame, alpha=cfg['contrast_alpha'], beta=cfg['brightness_beta'])
        frame = adjust_saturation(frame, sat_scale=cfg['saturation_scale'])

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
            for (x1,y1,x2,y2) in motion_boxes:
                cv2.rectangle(out, (x1,y1), (x2,y2), (0,0,255), 1)

        cv2.imshow('AI Camera Detector', out)
        key = cv2.waitKey(1) & 0xFF
        ch = chr(key).lower()
        if ch == "q":
            break
        elif ch == "s":
            path = save_snapshot(out)
            print(f'Snapshot saved to {path}')

    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

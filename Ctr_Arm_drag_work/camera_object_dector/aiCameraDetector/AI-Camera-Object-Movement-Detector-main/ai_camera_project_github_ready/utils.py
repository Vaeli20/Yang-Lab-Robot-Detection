# utils.py
import cv2
import os
from datetime import datetime

def ensure_dir(path):
    os.makedirs(path, exist_ok=True)

def save_snapshot(frame, out_dir='snapshots'):
    ensure_dir(out_dir)
    ts = datetime.now().strftime('%Y%m%d_%H%M%S')
    path = os.path.join(out_dir, f'snapshot_{ts}.png')
    cv2.imwrite(path, frame)
    return path

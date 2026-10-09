# AI Camera Object Movement Detector + Web API

A Python-based real-time object and motion detection system using YOLOv8 and OpenCV, enhanced with a FastAPI web server for remote control and live video streaming.

## 🚀 Features

- Real-time object detection with YOLOv8
- Classical motion detection (background subtraction)
- Combined motion + object detection
- Save snapshot with keypress
- 🔌 Web API to control detection remotely
- 📺 Live video feed over the web

---

## 🧰 Requirements

Install dependencies:

```bash
pip install -r requirements.txt
pip install fastapi uvicorn python-multipart

.
├── main.py           # Camera + detection logic
├── detector.py       # YOLOv8 object detection module
├── motion.py         # Motion detection logic
├── utils.py          # Helper functions
├── config.yaml       # Config file (thresholds etc.)
├── web_api.py        # FastAPI server (start/stop/video)
├── requirements.txt  # Dependencies
└── README.md         # Project overview (you are here)


uvicorn web_api:app --reload



http://localhost:8000/video



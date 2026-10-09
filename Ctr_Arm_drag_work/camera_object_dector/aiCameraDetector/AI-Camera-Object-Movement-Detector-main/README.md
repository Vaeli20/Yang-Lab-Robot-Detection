<img width="1035" height="278" alt="Screenshot 2025-10-08 at 10 49 16 AM" src="https://github.com/user-attachments/assets/30f65a02-0110-411f-991b-c6d9ae4cb71f" />

# 🎥 AI Camera Object & Movement Detector

![Static Badge](https://img.shields.io/badge/Project-AI_Camera_Object_%26_Movement_Detector-blueviolet?style=for-the-badge)
![Static Badge](https://img.shields.io/badge/Language-JavaScript-yellow?style=for-the-badge)
![Static Badge](https://img.shields.io/badge/AI-ML_Models-red?style=for-the-badge)
![Static Badge](https://img.shields.io/badge/Frontend-HTML-orange?style=for-the-badge)
![Static Badge](https://img.shields.io/badge/License-MIT-green?style=for-the-badge)

AI Camera Object & Movement Detector is a real-time browser-based detection system that uses AI models to identify objects and detect motion directly from your device’s camera.  
Fast, lightweight, and completely client-side — no server required.

---

## ✨ Features
- 🎯 Real-time object detection (TensorFlow.js / ML models)
- 📹 Live camera processing directly in the browser
- 🏃 Motion detection using pixel-difference analysis
- ⚡ Zero backend — fully client-side  
- 🖼 Bounding boxes and labels on detected objects
- 🔧 Modular and extendable architecture
- 📝 MIT License

---

## 🚀 Getting Started

### Clone the repository
```bash
git clone https://github.com/shloook/AI-Camera-Object-Movement-Detector.git
cd AI-Camera-Object-Movement-Detector
```

### Run the project
Simply open:
```
index.html
```
Works instantly in any modern browser.

Ensure you **allow camera permissions**.

---

## 📂 Project Structure
```
AI-Camera-Object-Movement-Detector/
├── index.html            # Main UI
├── style.css             # Styling
├── script.js             # Camera + motion detection logic
├── model.js              # AI model loading & object detection
├── assets/               # Images / icons
├── LICENSE               # MIT License
└── README.md             # Documentation
```

---

## 🖥 HTML Layout (index.html)
```html
<div class="container">
    <h1>AI Camera Object & Motion Detector</h1>

    <video id="cameraFeed" autoplay></video>
    <canvas id="overlay"></canvas>

    <div class="controls">
        <button onclick="startCamera()">Start Camera</button>
        <button onclick="stopCamera()">Stop Camera</button>
    </div>
</div>
```

---

## 🎨 Styling (style.css)
```css
body {
    background: #111;
    color: white;
    font-family: Arial, sans-serif;
    display: flex;
    justify-content: center;
    padding: 20px;
}

.container {
    width: 80%;
    text-align: center;
}

video, canvas {
    width: 100%;
    border-radius: 12px;
    margin-top: 15px;
    background: black;
}

button {
    padding: 12px 20px;
    margin: 10px;
    border: none;
    border-radius: 6px;
    background: #6200ea;
    color: white;
    font-size: 16px;
    cursor: pointer;
}
```

---

## 📡 Camera + Motion Detection Logic (script.js)
```javascript
let stream;
const video = document.getElementById("cameraFeed");
const canvas = document.getElementById("overlay");
const ctx = canvas.getContext("2d");
let previousFrame = null;

async function startCamera() {
    stream = await navigator.mediaDevices.getUserMedia({ video: true });
    video.srcObject = stream;

    video.onloadedmetadata = () => {
        canvas.width = video.videoWidth;
        canvas.height = video.videoHeight;
        detectLoop();
    };
}

function stopCamera() {
    if (stream) stream.getTracks().forEach(t => t.stop());
}

function detectMotion(frame) {
    if (!previousFrame) {
        previousFrame = frame;
        return false;
    }

    let movement = 0;
    for (let i = 0; i < frame.data.length; i += 4) {
        const diff = Math.abs(frame.data[i] - previousFrame.data[i]);
        if (diff > 30) movement++;
    }

    previousFrame = frame;
    return movement > 15000; // threshold
}

async function detectLoop() {
    ctx.drawImage(video, 0, 0);
    const frame = ctx.getImageData(0, 0, canvas.width, canvas.height);

    const movementDetected = detectMotion(frame);
    if (movementDetected) {
        ctx.strokeStyle = "red";
        ctx.lineWidth = 4;
        ctx.strokeRect(10, 10, canvas.width - 20, canvas.height - 20);
    }

    requestAnimationFrame(detectLoop);
}
```

---

## 🧠 Object Detection Model (model.js)
```javascript
let model;

async function loadModel() {
    model = await cocoSsd.load();
    console.log("Model Loaded!");
}

async function detectObjects() {
    if (!model) return;

    const predictions = await model.detect(video);

    ctx.clearRect(0, 0, canvas.width, canvas.height);

    predictions.forEach(pred => {
        ctx.strokeStyle = "#00FF00";
        ctx.lineWidth = 3;
        ctx.strokeRect(
            pred.bbox[0],
            pred.bbox[1],
            pred.bbox[2],
            pred.bbox[3]
        );

        ctx.fillStyle = "yellow";
        ctx.font = "18px Arial";
        ctx.fillText(pred.class, pred.bbox[0], pred.bbox[1] - 5);
    });

    requestAnimationFrame(detectObjects);
}

loadModel();
```

---

## 🧪 Optional Backend API (for image upload / processing)

### Upload Image
```
POST /api/upload
Content-Type: multipart/form-data
```

### Enhanced Detection API
```
POST /api/ai/detect
Content-Type: application/json

{
  "image": "<base64>"
}
```

### Example Response
```json
{
  "objects": [
    { "class": "person", "confidence": 0.87 }
  ]
}
```

---

## 🤝 Contributing
1. Fork the repository  
2. Create a new branch  
3. Commit your changes  
4. Push and submit a pull request  

---

## 📄 License
MIT License.

---

## ⭐ Acknowledgements
Built using TensorFlow.js & COCO-SSD  
Inspired by modern computer vision systems  
Designed for learning, experimentation, and real-time AI interaction

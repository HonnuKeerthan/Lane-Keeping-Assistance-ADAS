# Lane Keeping Assistance (LKA) Simulation

> ⚠️ **Software Simulation Only** — This is an engineering college project. It is NOT a real FSD or autonomous driving system. No machine learning or real vehicle hardware is used.

---

## Project Overview

A complete **Lane Keeping Assistance simulation** system built with classical computer vision and vehicle dynamics modeling. It features:
- **LKA ON/OFF Master Control**: Toggle between active PD lane keeping assist and natural vehicle drift with manual observation.
- **Dynamic Curved Road Simulator**: Real-time 3D perspective road engine with smooth curves, animated forward motion, consistent lane width, and vehicle lateral dynamics.
- **6 Road Geometries**: Straight, Gentle Left Curve, Gentle Right Curve, Sharp Left Curve, Sharp Right Curve, and S-Curve.
- **Lane Departure Warning (LDW)**: Visual flashing alert when vehicle departs lane boundary (≥0.75m).
- **Vehicle Re-centering**: Dedicated `Reset Vehicle` control.
- **FSD-Style Live Dashboard**: Real-time MJPEG feed, dynamic telemetry cards, steering visualizer, and Chart.js graphs.
- **Video Playback Mode**: Preserves custom upload and sample dashcam video analysis.

### Tech Stack
| Layer | Technologies |
|-------|-------------|
| Backend | Python 3, Flask, OpenCV, NumPy |
| Physics & Control | 3D Perspective Projection, Proportional-Derivative (PD) Controller |
| Frontend | HTML5, CSS3 (Vanilla), JavaScript, Chart.js |
| CV Pipeline | Grayscale → Gaussian Blur → Canny → ROI → Hough Lines / Curved Spline Model |

---

## Quick Start (macOS)

### Option A – One-line Start
```bash
cd LKA_Simulation
./run.sh
```

### Option B – Manual Setup
```bash
cd LKA_Simulation

# Start server
python3 backend/app.py
```

### Option C – Automated Verification Suite (12 Scenarios)
```bash
cd LKA_Simulation
python3 backend/test_scenarios.py
```

Open **http://127.0.0.1:5000** (or **http://127.0.0.1:5005** if port 5000 is occupied) in your browser.

---

## Project Structure

```
LKA_Simulation/
├── run.sh                        ← One-click start script
├── backend/
│   ├── app.py                    ← Flask server & API endpoints
│   ├── curved_road.py            ← 3D curved road simulator & vehicle dynamics
│   ├── lane_detection.py         ← CV pipeline (core algorithm with LKA toggle)
│   ├── video_processor.py        ← Background thread manager (curved + video)
│   ├── test_scenarios.py         ← Automated test suite for 12 scenarios
│   ├── generate_samples.py       ← Synthetic sample video generator
│   ├── requirements.txt
│   ├── uploads/                  ← User-uploaded videos
│   └── sample_videos/            ← Sample video clips
└── frontend/
    ├── templates/
    │   ├── index.html            ← Home page
    │   └── simulation.html       ← Upgraded FSD-style dashboard
    └── static/
        ├── css/
        │   ├── style.css         ← Global dark theme
        │   └── simulation.css    ← Dashboard layout & toggle styles
        └── js/
            └── simulation.js     ← Interactive frontend logic + Chart.js
```


---

## Detection Pipeline

```
READ FRAME
    ↓
GRAYSCALE CONVERSION
    ↓
GAUSSIAN BLUR (5×5 kernel)
    ↓
CANNY EDGE DETECTION (threshold 50-150)
    ↓
REGION OF INTEREST MASK (trapezoid, bottom 42%)
    ↓
HOUGH LINE TRANSFORM (probabilistic)
    ↓
LEFT / RIGHT LANE SEPARATION (by slope & x-position)
    ↓
AVERAGE LINES → PIXEL COORDINATES
    ↓
LANE CENTRE CALCULATION
    ↓
DEVIATION = (vehicle_cx - lane_cx) / frame_width × 3.5 m
    ↓
STATUS: CENTERED | DRIFTING LEFT | DRIFTING RIGHT
    ↓
STEERING: KEEP STRAIGHT | STEER LEFT | STEER RIGHT
```

---

## API Endpoints

| Method | Endpoint | Description |
|--------|----------|-------------|
| `GET`  | `/` | Home page |
| `GET`  | `/simulation` | Dashboard page |
| `POST` | `/upload` | Upload video file |
| `POST` | `/start` | Start processing |
| `POST` | `/stop` | Stop processing |
| `POST` | `/reset` | Reset system |
| `GET`  | `/video_feed` | MJPEG stream |
| `GET`  | `/status` | Latest LKA status JSON |
| `GET`  | `/sample_videos` | List sample videos |
| `GET`  | `/use_sample?name=<file>` | Select sample |

### Status Response Example
```json
{
  "lka_status":  "ACTIVE",
  "lane_status": "DRIFTING LEFT",
  "steering":    "STEER RIGHT",
  "deviation":   -0.18,
  "detection":   "LANES DETECTED",
  "fps":         22.5,
  "running":     true
}
```

---

## Supported Cases

| Scenario | LKA Status | Lane Status | Steering |
|----------|-----------|-------------|----------|
| Both lanes, centred | ACTIVE | CENTERED | KEEP STRAIGHT |
| Drift left (>0.05m) | ACTIVE | DRIFTING LEFT | STEER RIGHT |
| Drift right (>0.05m) | ACTIVE | DRIFTING RIGHT | STEER LEFT |
| Only left lane | ACTIVE | CENTERED/DRIFTING | based on estimate |
| Only right lane | ACTIVE | CENTERED/DRIFTING | based on estimate |
| No lanes detected | SEARCHING | LANE NOT DETECTED | --- |
| Invalid/ended video | ERROR / loops | — | — |

---

## Dashboard Features

- 🎥 Live MJPEG video feed with CV overlays
- 🗺️ Annotated lanes (amber lines), lane fill (green), centre markers
- 📊 Real-time Lane Deviation chart
- 📊 Real-time Steering Correction chart
- 🚗 Steering visualiser with car position & arrow
- 📋 Status cards: LKA, Lane, Steering, Deviation, Detection
- ⚙️ Upload your own MP4/AVI/MOV or use sample clips
- 🔁 Auto-loops video for continuous demonstration

---

## License

College Engineering Project — Educational Use Only.

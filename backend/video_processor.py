"""
Video Processor Module for LKA Simulation
=========================================
Manages background simulation threads (Curved Road Simulator & Video Playback),
JPEG encoding for the MJPEG stream, and thread-safe telemetry status for /status.
"""

import os
import time
import threading
import cv2
import numpy as np

from lane_detection import detect_lanes
from curved_road import simulator, LANE_WIDTH_M, LDW_THRESHOLD

# ─────────────────────────────────────────────
#  Shared state (thread-safe via Lock)
# ─────────────────────────────────────────────

_lock          = threading.Lock()
_latest_frame  = None   # JPEG bytes
_lka_enabled   = True
_road_type     = "straight"
_sim_mode      = "curved_road"  # "curved_road" or "video"
_video_path    = None

_latest_status = {
    "lka_status":      "ACTIVE",
    "lka_enabled":     True,
    "mode":            "LKA ASSISTED",
    "lane_status":     "CENTERED",
    "steering":        "KEEP STRAIGHT",
    "steering_angle":  0.0,
    "correction_val":  0.0,
    "deviation":       0.0,
    "lateral_pos":     0.0,
    "detection":       "LANES DETECTED",
    "road_type":       "straight",
    "road_name":       "Straight Road",
    "curvature":       0.0,
    "ldw_active":      False,
    "ldw_warning":     False,
    "frame_count":     0,
    "fps":             0.0,
    "running":         False,
    "simulation_mode": "curved_road",
}

_stop_event    = threading.Event()
_proc_thread   = None


# ─────────────────────────────────────────────
#  Curved Road Background Worker
# ─────────────────────────────────────────────

def _process_curved_road():
    """
    Background thread: generates synthetic curved road frames using
    the vehicle dynamics model and PD lane keeping controller.
    """
    global _latest_frame, _latest_status

    target_fps  = 30.0
    frame_delay = 1.0 / target_fps
    t_fps       = time.time()
    frames_fps  = 0
    fps_est     = target_fps

    while not _stop_event.is_set():
        t_start = time.time()

        # Generate physics update + rendered frame
        res = simulator.generate_frame()
        frame = res["annotated_frame"]

        # Encode to JPEG
        _, jpeg = cv2.imencode(
            ".jpg", frame,
            [cv2.IMWRITE_JPEG_QUALITY, 85]
        )

        frames_fps += 1
        elapsed_fps = time.time() - t_fps
        if elapsed_fps >= 1.0:
            fps_est = frames_fps / elapsed_fps
            frames_fps = 0
            t_fps = time.time()

        with _lock:
            _latest_frame = jpeg.tobytes()
            _latest_status = {
                "lka_status":      res["lka_status"],
                "lka_enabled":     res["lka_enabled"],
                "mode":            res["mode"],
                "lane_status":     res["lane_status"],
                "steering":        res["steering"],
                "steering_angle":  res["steering_angle"],
                "correction_val":  res["correction_val"],
                "deviation":       res["deviation"],
                "lateral_pos":     res["lateral_pos"],
                "detection":       res["detection"],
                "road_type":       res["road_type"],
                "road_name":       res["road_name"],
                "curvature":       res["curvature"],
                "ldw_active":      res["ldw_active"],
                "ldw_warning":     res["ldw_warning"],
                "frame_count":     res["frame_count"],
                "fps":             round(fps_est, 1),
                "running":         True,
                "simulation_mode": "curved_road",
            }

        elapsed = time.time() - t_start
        sleep_t = max(0.001, frame_delay - elapsed)
        time.sleep(sleep_t)

    with _lock:
        _latest_status["running"] = False


# ─────────────────────────────────────────────
#  Video File Background Worker
# ─────────────────────────────────────────────

def _process_video(video_path: str):
    """
    Background thread: reads frames from video file, runs CV lane detection,
    applies LKA ON/OFF logic, and updates shared state.
    """
    global _latest_frame, _latest_status

    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        with _lock:
            _latest_status.update({
                "lka_status":  "ERROR",
                "lane_status": "INVALID VIDEO",
                "steering":    "---",
                "deviation":   0.0,
                "detection":   "VIDEO OPEN FAILED",
                "running":     False,
            })
        return

    source_fps = cap.get(cv2.CAP_PROP_FPS) or 25.0
    frame_delay = 1.0 / source_fps

    frame_count  = 0
    fps_estimate = source_fps
    t_fps        = time.time()

    while not _stop_event.is_set():
        t_start = time.time()

        ret, frame = cap.read()
        if not ret:
            cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            ret, frame = cap.read()
            if not ret:
                break

        frame = cv2.resize(frame, (960, 540))

        with _lock:
            curr_lka_enabled = _lka_enabled

        # Run CV detection with current LKA toggle state
        result = detect_lanes(frame, lka_enabled=curr_lka_enabled)

        _, jpeg = cv2.imencode(
            ".jpg", result["annotated_frame"],
            [cv2.IMWRITE_JPEG_QUALITY, 85]
        )

        frame_count += 1
        elapsed_fps = time.time() - t_fps
        if elapsed_fps >= 1.0:
            fps_estimate = frame_count / elapsed_fps
            frame_count  = 0
            t_fps        = time.time()

        with _lock:
            _latest_frame = jpeg.tobytes()
            _latest_status = {
                "lka_status":      result["lka_status"],
                "lka_enabled":     result["lka_enabled"],
                "mode":            result["mode"],
                "lane_status":     result["lane_status"],
                "steering":        result["steering"],
                "steering_angle":  result.get("steering_angle", 0.0),
                "correction_val":  result.get("correction_val", 0.0),
                "deviation":       result["deviation"],
                "lateral_pos":     result["lateral_pos"],
                "detection":       result["detection"],
                "road_type":       "recorded_video",
                "road_name":       os.path.basename(video_path),
                "curvature":       0.0,
                "ldw_active":      result.get("ldw_active", False),
                "ldw_warning":     result.get("ldw_warning", False),
                "frame_count":     frame_count,
                "fps":             round(fps_estimate, 1),
                "running":         True,
                "simulation_mode": "video",
            }

        elapsed = time.time() - t_start
        sleep_t = max(0.001, frame_delay - elapsed)
        time.sleep(sleep_t)

    cap.release()
    with _lock:
        _latest_status["running"] = False


# ─────────────────────────────────────────────
#  Public Control API
# ─────────────────────────────────────────────

def set_lka_enabled(enabled: bool):
    """Enable or disable LKA control."""
    global _lka_enabled
    with _lock:
        _lka_enabled = bool(enabled)
        simulator.set_lka_enabled(_lka_enabled)
        _latest_status["lka_enabled"] = _lka_enabled
        _latest_status["lka_status"]  = "ACTIVE" if _lka_enabled else "DISABLED"
        _latest_status["mode"]        = "LKA ASSISTED" if _lka_enabled else "MANUAL"
        if not _lka_enabled:
            _latest_status["correction_val"] = 0.0
            _latest_status["steering_angle"] = 0.0
            _latest_status["steering"]       = "MANUAL CONTROL"


def set_road_type(road_type: str):
    """Set road curvature geometry (straight, gentle_left, sharp_right, etc.)."""
    global _road_type
    with _lock:
        _road_type = road_type
        simulator.set_road_type(road_type)
        road_names = {
            "straight":     "Straight Road",
            "gentle_left":  "Gentle Left Curve",
            "gentle_right": "Gentle Right Curve",
            "sharp_left":   "Sharp Left Curve",
            "sharp_right":  "Sharp Right Curve",
            "s_curve":      "S-Curve",
        }
        _latest_status["road_type"] = road_type
        _latest_status["road_name"] = road_names.get(road_type, road_type)
        _latest_status["curvature"] = round(simulator.get_curvature(0.0), 5)


def reset_vehicle():
    """Reset simulated vehicle to the lane center."""
    with _lock:
        simulator.reset_vehicle()
        _latest_status.update({
            "deviation":      0.0,
            "lateral_pos":    0.0,
            "lane_status":    "CENTERED",
            "ldw_active":     False,
            "ldw_warning":    False,
            "steering_angle": 0.0,
            "correction_val": 0.0,
            "steering":       "KEEP STRAIGHT" if _lka_enabled else "MANUAL CONTROL",
        })


def start_processing(video_path: str = None, road_type: str = None):
    """
    Start background processing loop.
    If video_path is provided and valid, runs video processor.
    Otherwise, runs curved road software simulator.
    """
    global _proc_thread, _sim_mode, _video_path

    stop_processing()

    _stop_event.clear()

    if video_path and os.path.isfile(video_path):
        _sim_mode   = "video"
        _video_path = video_path
        _proc_thread = threading.Thread(
            target=_process_video,
            args=(video_path,),
            daemon=True,
            name="lka-video-proc",
        )
    else:
        _sim_mode   = "curved_road"
        _video_path = None
        if road_type:
            set_road_type(road_type)
        _proc_thread = threading.Thread(
            target=_process_curved_road,
            daemon=True,
            name="lka-curved-proc",
        )

    _proc_thread.start()
    return True, f"Processing started in {_sim_mode} mode."


def stop_processing():
    """Signal background thread to stop and wait for completion."""
    global _proc_thread

    _stop_event.set()
    if _proc_thread and _proc_thread.is_alive():
        _proc_thread.join(timeout=2.0)
    _proc_thread = None

    with _lock:
        _latest_status["running"] = False


def reset_system():
    """Stop simulation and reset all states."""
    stop_processing()
    reset_vehicle()
    with _lock:
        global _latest_frame, _video_path
        _latest_frame = None
        _video_path   = None
        _latest_status.update({
            "running":         False,
            "frame_count":     0,
            "fps":             0.0,
            "deviation":       0.0,
            "lateral_pos":     0.0,
            "lane_status":     "---",
            "steering":        "---",
            "detection":       "---",
            "ldw_active":      False,
            "ldw_warning":     False,
            "correction_val":  0.0,
            "steering_angle":  0.0,
        })


def get_status() -> dict:
    """Return a copy of the latest status dictionary."""
    with _lock:
        return dict(_latest_status)


def generate_frames():
    """MJPEG stream generator. Yields JPEG frames."""
    while True:
        with _lock:
            frame = _latest_frame

        if frame is None:
            placeholder = _black_placeholder()
            yield (
                b"--frame\r\n"
                b"Content-Type: image/jpeg\r\n\r\n" + placeholder + b"\r\n"
            )
            time.sleep(0.08)
            continue

        yield (
            b"--frame\r\n"
            b"Content-Type: image/jpeg\r\n\r\n" + frame + b"\r\n"
        )
        time.sleep(0.03)


def is_running() -> bool:
    with _lock:
        return _latest_status.get("running", False)


# ─────────────────────────────────────────────
#  Placeholder helper
# ─────────────────────────────────────────────

_placeholder_cache = None

def _black_placeholder():
    """Return cached 960x540 standby JPEG."""
    global _placeholder_cache
    if _placeholder_cache is None:
        black = np.zeros((540, 960, 3), dtype=np.uint8)
        cv2.putText(black, "LKA Simulation Ready -- Select Road & Press Start",
                    (110, 270), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (120, 140, 160), 2)
        _, buf = cv2.imencode(".jpg", black)
        _placeholder_cache = buf.tobytes()
    return _placeholder_cache

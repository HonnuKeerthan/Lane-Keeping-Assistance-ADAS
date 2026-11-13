"""
LKA Simulation – Flask Backend
Endpoints:
  POST /upload        – upload a video file
  POST /start         – start processing (with optional video_path body)
  POST /stop          – stop processing
  POST /reset         – stop + clear state
  GET  /video_feed    – MJPEG stream
  GET  /status        – latest LKA JSON
  GET  /sample_videos – list available sample videos
  GET  /use_sample    – select a sample video (query param: name)
"""

import os
import json
from flask import Flask, Response, jsonify, request, send_from_directory, render_template
from flask_cors import CORS
from werkzeug.utils import secure_filename

import video_processor as vp

# ─────────────────────────────────────────────
#  App setup
# ─────────────────────────────────────────────

BASE_DIR       = os.path.dirname(os.path.abspath(__file__))
UPLOAD_FOLDER  = os.path.join(BASE_DIR, "uploads")
SAMPLE_FOLDER  = os.path.join(BASE_DIR, "sample_videos")
TEMPLATE_DIR   = os.path.join(BASE_DIR, "..", "frontend", "templates")
STATIC_DIR     = os.path.join(BASE_DIR, "..", "frontend", "static")

os.makedirs(UPLOAD_FOLDER, exist_ok=True)
os.makedirs(SAMPLE_FOLDER, exist_ok=True)

ALLOWED_EXTENSIONS = {"mp4", "avi", "mov", "mkv"}

app = Flask(
    __name__,
    template_folder=TEMPLATE_DIR,
    static_folder=STATIC_DIR,
)
CORS(app)

app.config["UPLOAD_FOLDER"]    = UPLOAD_FOLDER
app.config["MAX_CONTENT_LENGTH"] = 500 * 1024 * 1024  # 500 MB


# ─────────────────────────────────────────────
#  Helpers
# ─────────────────────────────────────────────

def allowed_file(filename: str) -> bool:
    return "." in filename and filename.rsplit(".", 1)[1].lower() in ALLOWED_EXTENSIONS


# ─────────────────────────────────────────────
#  Page routes
# ─────────────────────────────────────────────

@app.route("/")
def home():
    return render_template("index.html")


@app.route("/simulation")
def simulation():
    return render_template("simulation.html")


# ─────────────────────────────────────────────
#  API routes
# ─────────────────────────────────────────────

@app.route("/upload", methods=["POST"])
def upload():
    if "video" not in request.files:
        return jsonify({"success": False, "message": "No file part in request."}), 400

    file = request.files["video"]
    if file.filename == "":
        return jsonify({"success": False, "message": "No file selected."}), 400

    if not allowed_file(file.filename):
        return jsonify({
            "success": False,
            "message": f"Unsupported format. Allowed: {', '.join(ALLOWED_EXTENSIONS)}",
        }), 400

    filename = secure_filename(file.filename)
    save_path = os.path.join(app.config["UPLOAD_FOLDER"], filename)
    file.save(save_path)

    return jsonify({
        "success":    True,
        "message":    "File uploaded successfully.",
        "video_path": save_path,
        "filename":   filename,
    })


@app.route("/start", methods=["POST"])
def start():
    data       = request.get_json(silent=True) or {}
    video_path = data.get("video_path", "")
    road_type  = data.get("road_type", "straight")

    # If video_path is provided and exists, run video mode; otherwise run curved road mode
    if video_path and not os.path.isfile(video_path):
        video_path = None

    ok, msg = vp.start_processing(video_path=video_path, road_type=road_type)
    return jsonify({"success": ok, "message": msg, "status": vp.get_status()})


@app.route("/stop", methods=["POST"])
def stop():
    vp.stop_processing()
    return jsonify({"success": True, "message": "Processing stopped.", "status": vp.get_status()})


@app.route("/reset", methods=["POST"])
def reset():
    vp.reset_system()
    return jsonify({"success": True, "message": "System reset.", "status": vp.get_status()})


@app.route("/reset_vehicle", methods=["POST"])
def reset_vehicle():
    vp.reset_vehicle()
    return jsonify({"success": True, "message": "Vehicle re-centered.", "status": vp.get_status()})


@app.route("/set_lka", methods=["POST"])
def set_lka():
    data    = request.get_json(silent=True) or {}
    enabled = data.get("enabled", True)
    vp.set_lka_enabled(enabled)
    return jsonify({
        "success":     True,
        "lka_enabled": enabled,
        "status":      vp.get_status(),
    })


@app.route("/set_road", methods=["POST"])
def set_road():
    data      = request.get_json(silent=True) or {}
    road_type = data.get("road_type", "straight")
    vp.set_road_type(road_type)
    return jsonify({
        "success":   True,
        "road_type": road_type,
        "status":    vp.get_status(),
    })



@app.route("/video_feed")
def video_feed():
    return Response(
        vp.generate_frames(),
        mimetype="multipart/x-mixed-replace; boundary=frame",
    )


@app.route("/status")
def status():
    return jsonify(vp.get_status())


@app.route("/sample_videos")
def list_sample_videos():
    videos = []
    for f in os.listdir(SAMPLE_FOLDER):
        if allowed_file(f):
            videos.append({
                "name": f,
                "path": os.path.join(SAMPLE_FOLDER, f),
            })
    return jsonify({"videos": videos})


@app.route("/use_sample")
def use_sample():
    name = request.args.get("name", "")
    if not name:
        return jsonify({"success": False, "message": "No sample name provided."}), 400

    path = os.path.join(SAMPLE_FOLDER, secure_filename(name))
    if not os.path.isfile(path):
        return jsonify({"success": False, "message": "Sample video not found."}), 404

    return jsonify({"success": True, "video_path": path, "filename": name})


# ─────────────────────────────────────────────
#  Run
# ─────────────────────────────────────────────

def get_available_port(preferred=5000):
    env_p = os.environ.get("PORT")
    if env_p:
        return int(env_p)
    import socket
    for p in [preferred, 5005, 5050, 8080]:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            try:
                s.bind(("127.0.0.1", p))
                return p
            except OSError:
                continue
    return preferred

if __name__ == "__main__":
    port = get_available_port(5000)
    print("=" * 60)
    print("  LKA Simulation – Flask Backend")
    print(f"  Open http://127.0.0.1:{port} in your browser")
    print("  Press Ctrl+C to stop")
    print("=" * 60)
    app.run(host="0.0.0.0", port=port, debug=False, threaded=True)


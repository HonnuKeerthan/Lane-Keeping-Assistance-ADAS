#!/bin/bash
# ─────────────────────────────────────────────────────────
#  LKA Simulation – Quick Start Script
#  Run from the LKA_Simulation/ project root
# ─────────────────────────────────────────────────────────

set -e
cd "$(dirname "$0")"

echo "╔══════════════════════════════════════════════╗"
echo "║   Lane Keeping Assistance – LKA Simulation  ║"
echo "║   FSD-Style Simulation | Software Only       ║"
echo "╚══════════════════════════════════════════════╝"
echo ""

# ── Step 1: Create venv if missing ──────────────────────
if [ ! -d "venv" ]; then
    echo "→ Creating virtual environment…"
    python3 -m venv venv
fi

# ── Step 2: Install / verify dependencies ───────────────
echo "→ Checking dependencies…"
venv/bin/pip install -q -r backend/requirements.txt || true

# Fallback: use system python3 if pip install failed
PYTHON="venv/bin/python3"
if ! $PYTHON -c "import flask, cv2" 2>/dev/null; then
    echo "  venv packages not available, using system python3"
    PYTHON="python3"
fi

# ── Step 3: Generate sample videos if missing ───────────
if [ ! -f "backend/sample_videos/lane_demo_centered.mp4" ]; then
    echo "→ Generating sample videos (one-time, ~5 seconds)…"
    $PYTHON backend/generate_samples.py
fi

# ── Step 4: Start server ─────────────────────────────────
PORT=${PORT:-5000}
if ! $PYTHON -c "import socket; s = socket.socket(); s.bind(('127.0.0.1', 5000)); s.close()" 2>/dev/null; then
    PORT=5005
fi
export PORT

echo ""
echo "→ Starting Flask server…"
echo "  Open http://127.0.0.1:$PORT in your browser"
echo "  Press Ctrl+C to stop"
echo ""

$PYTHON backend/app.py


"""
Generate a synthetic dashcam-style lane video for the LKA sample library.
Creates two video variants:
  1. lane_demo_centered.mp4   – vehicle stays roughly centred
  2. lane_demo_drift.mp4      – vehicle drifts left then right repeatedly

Run:
    python3 generate_samples.py
"""

import cv2
import numpy as np
import math
import os

OUT_DIR  = os.path.join(os.path.dirname(__file__), "sample_videos")
WIDTH    = 960
HEIGHT   = 540
FPS      = 25
DURATION = 30   # seconds per clip


def draw_road(img, vanishing_y, left_bottom, right_bottom, left_top, right_top):
    """Fill road polygon (dark grey) between lane boundaries."""
    road_pts = np.array([
        [left_bottom,  HEIGHT],
        [left_top,     vanishing_y],
        [right_top,    vanishing_y],
        [right_bottom, HEIGHT],
    ], dtype=np.int32)
    cv2.fillPoly(img, [road_pts], (50, 50, 55))


def draw_lane_lines(img, left_bottom, right_bottom, left_top, right_top, vanishing_y):
    """Draw solid left/right white lane markings."""
    cv2.line(img, (left_bottom,  HEIGHT), (left_top,  vanishing_y), (230, 230, 230), 3)
    cv2.line(img, (right_bottom, HEIGHT), (right_top, vanishing_y), (230, 230, 230), 3)


def draw_dashed_centre(img, cx_bottom, cx_top, vanishing_y, frame_idx):
    """Draw a dashed yellow centre line."""
    steps = 12
    for i in range(steps):
        t0  = i       / steps
        t1  = (i + 0.5) / steps
        y0  = int(HEIGHT - (HEIGHT - vanishing_y) * t0)
        y1  = int(HEIGHT - (HEIGHT - vanishing_y) * t1)
        x0  = int(cx_bottom + (cx_top  - cx_bottom) * t0)
        x1  = int(cx_bottom + (cx_top  - cx_bottom) * t1)
        # Animate scroll by frame offset
        offset = (frame_idx * 3) % (HEIGHT // steps)
        y0 = min(HEIGHT, y0 + offset)
        y1 = min(HEIGHT, y1 + offset)
        if y1 > vanishing_y:
            cv2.line(img, (x0, y0), (x1, y1), (0, 220, 220), 2)


def draw_car(img, cx):
    """Draw a simple top-down car icon centred at (cx, HEIGHT-70)."""
    cy = HEIGHT - 70
    # Body
    cv2.rectangle(img, (cx - 28, cy - 55), (cx + 28, cy + 10), (30, 30, 180), -1)
    # Windshield
    cv2.rectangle(img, (cx - 20, cy - 50), (cx + 20, cy - 20), (150, 210, 255), -1)
    # Wheels
    for wx, wy in [(cx - 32, cy - 40), (cx + 32, cy - 40),
                   (cx - 32, cy),       (cx + 32, cy)]:
        cv2.rectangle(img, (wx - 7, wy - 12), (wx + 7, wy + 12), (20, 20, 20), -1)


def add_sky_and_horizon(img, vanishing_y):
    """Fill sky area with gradient and add horizon glow."""
    sky = img[:vanishing_y, :]
    for row in range(vanishing_y):
        t = row / vanishing_y
        sky[row, :] = (int(20 * (1 - t) + 10 * t),
                       int(20 * (1 - t) + 15 * t),
                       int(40 * (1 - t) + 25 * t))
    img[:vanishing_y, :] = sky
    cv2.line(img, (0, vanishing_y), (WIDTH, vanishing_y), (80, 100, 130), 2)


def make_frame(frame_idx, offset_x, total_frames):
    """
    Render one synthetic dashcam frame.
    offset_x  – horizontal pixel shift of the camera (simulates drift).
    """
    img = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)

    vanishing_y = int(HEIGHT * 0.42)

    # Lane geometry in world space
    lane_half    = 170   # pixels at bottom (half of lane width)
    centre_world = WIDTH // 2 - offset_x  # camera is offset

    left_bottom  = centre_world - lane_half
    right_bottom = centre_world + lane_half
    left_top     = WIDTH // 2 - 30
    right_top    = WIDTH // 2 + 30

    add_sky_and_horizon(img, vanishing_y)
    draw_road(img, vanishing_y, left_bottom, right_bottom, left_top, right_top)
    draw_lane_lines(img, left_bottom, right_bottom, left_top, right_top, vanishing_y)
    draw_dashed_centre(img, (left_bottom + right_bottom) // 2,
                       (left_top + right_top) // 2, vanishing_y, frame_idx)

    # Car is always drawn at frame centre (camera-relative)
    car_cx = WIDTH // 2
    draw_car(img, car_cx)

    # HUD overlay
    t_sec = frame_idx / FPS
    cv2.putText(img, f"DASHCAM | t={t_sec:.1f}s", (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 1)
    cv2.putText(img, "FSD-STYLE SIMULATION | Software Simulation Only",
                (10, HEIGHT - 12), cv2.FONT_HERSHEY_SIMPLEX,
                0.42, (140, 140, 140), 1)

    return img


def write_video(path, offsets):
    """Write a video from a list of per-frame offset_x values."""
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    out    = cv2.VideoWriter(path, fourcc, FPS, (WIDTH, HEIGHT))
    total  = len(offsets)
    for i, ox in enumerate(offsets):
        frame = make_frame(i, ox, total)
        out.write(frame)
    out.release()
    print(f"  Saved: {path}  ({total} frames)")


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    total = FPS * DURATION

    # ── Video 1: centred (small oscillation) ─────────────────────────
    offsets_centre = [
        int(8 * math.sin(2 * math.pi * i / (FPS * 4)))  # ±8 px, 4-s period
        for i in range(total)
    ]
    write_video(os.path.join(OUT_DIR, "lane_demo_centered.mp4"), offsets_centre)

    # ── Video 2: drift left then right ───────────────────────────────
    offsets_drift = []
    for i in range(total):
        # Slow sinusoid: ±80 px, 10-s period
        ox = int(80 * math.sin(2 * math.pi * i / (FPS * 10)))
        offsets_drift.append(ox)
    write_video(os.path.join(OUT_DIR, "lane_demo_drift.mp4"), offsets_drift)

    # ── Video 3: gradual left drift ──────────────────────────────────
    offsets_left = []
    for i in range(total):
        ox = int(-60 * (i / total))   # drift from 0 to -60 px
        offsets_left.append(ox)
    write_video(os.path.join(OUT_DIR, "lane_demo_drift_left.mp4"), offsets_left)

    print("\nAll sample videos generated successfully!")


if __name__ == "__main__":
    main()

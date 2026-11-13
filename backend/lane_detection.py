"""
Lane Detection Module for LKA Simulation
Uses classical computer vision: Grayscale -> Gaussian Blur -> Canny -> ROI -> Hough Lines
"""

import cv2
import numpy as np


# ─────────────────────────────────────────────
#  Constants
# ─────────────────────────────────────────────
DEVIATION_THRESHOLD = 0.05   # metres – centre±threshold = CENTERED
LANE_WIDTH_ESTIMATE = 3.5    # metres (standard lane width)


# ─────────────────────────────────────────────
#  Pre-processing helpers
# ─────────────────────────────────────────────

def preprocess_frame(frame):
    """Convert frame to grayscale, blur, and run Canny edge detection."""
    gray   = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur   = cv2.GaussianBlur(gray, (5, 5), 0)
    edges  = cv2.Canny(blur, 50, 150)
    return edges


def region_of_interest(edges, height, width):
    """
    Mask the frame so only the lower trapezoidal road area is analysed.
    The trapezoid covers the bottom ~55 % of the image.
    """
    mask = np.zeros_like(edges)

    # Trapezoid vertices (bottom-left, top-left, top-right, bottom-right)
    vertices = np.array([[
        (0,            height),
        (int(width * 0.45), int(height * 0.58)),
        (int(width * 0.55), int(height * 0.58)),
        (width,        height),
    ]], dtype=np.int32)

    cv2.fillPoly(mask, vertices, 255)
    return cv2.bitwise_and(edges, mask)


# ─────────────────────────────────────────────
#  Hough-line helpers
# ─────────────────────────────────────────────

def detect_hough_lines(masked_edges):
    """Run probabilistic Hough transform and return raw line segments."""
    lines = cv2.HoughLinesP(
        masked_edges,
        rho=1,
        theta=np.pi / 180,
        threshold=40,
        minLineLength=40,
        maxLineGap=200,
    )
    return lines if lines is not None else []


def slope_intercept(x1, y1, x2, y2):
    """Return (slope, intercept) for a line through (x1,y1)-(x2,y2)."""
    if x2 - x1 == 0:
        return None, None
    slope = (y2 - y1) / (x2 - x1)
    intercept = y1 - slope * x1
    return slope, intercept


def separate_lanes(lines, width):
    """
    Classify each line segment as left or right lane based on:
      - slope sign  (negative → left, positive → right in image coords)
      - x-position  (left half / right half)
    Returns two lists of (slope, intercept) tuples.
    """
    left_lines  = []
    right_lines = []

    for line in lines:
        x1, y1, x2, y2 = line[0]
        slope, intercept = slope_intercept(x1, y1, x2, y2)

        if slope is None:
            continue
        # Ignore nearly horizontal lines (noise)
        if abs(slope) < 0.3:
            continue

        # Left lane: negative slope and mostly in left half
        mid_x = (x1 + x2) / 2
        if slope < 0 and mid_x < width * 0.55:
            left_lines.append((slope, intercept))
        # Right lane: positive slope and mostly in right half
        elif slope > 0 and mid_x > width * 0.45:
            right_lines.append((slope, intercept))

    return left_lines, right_lines


def average_line(lines):
    """Average a list of (slope, intercept) pairs into one representative line."""
    if not lines:
        return None
    slopes     = [l[0] for l in lines]
    intercepts = [l[1] for l in lines]
    return float(np.mean(slopes)), float(np.mean(intercepts))


def make_coordinates(slope, intercept, y_start, y_end):
    """
    Given a slope-intercept line and y-range, return pixel endpoints
    (x1, y1, x2, y2) clamped to valid values.
    """
    if slope == 0:
        return None
    x1 = int((y_start - intercept) / slope)
    x2 = int((y_end   - intercept) / slope)
    return x1, int(y_start), x2, int(y_end)


# ─────────────────────────────────────────────
#  Deviation & status
# ─────────────────────────────────────────────

def compute_deviation(lane_center_x, vehicle_center_x, frame_width):
    """
    Convert pixel deviation to metres using lane-width scaling.
    Negative  → vehicle is to the LEFT  of lane centre
    Positive  → vehicle is to the RIGHT of lane centre
    """
    pixel_deviation = vehicle_center_x - lane_center_x
    # Normalise by frame width and scale to real-world lane width
    deviation_m = (pixel_deviation / frame_width) * LANE_WIDTH_ESTIMATE
    return round(deviation_m, 3)


def get_lane_status(deviation_m):
    """Return (lane_status, steering_recommendation) strings."""
    if abs(deviation_m) <= DEVIATION_THRESHOLD:
        return "CENTERED", "KEEP STRAIGHT"
    elif deviation_m < 0:
        return "DRIFTING LEFT", "STEER RIGHT"
    else:
        return "DRIFTING RIGHT", "STEER LEFT"


LDW_THRESHOLD = 0.75   # metres – vehicle crossing defined safety threshold


# ─────────────────────────────────────────────
#  Main detection entry point
# ─────────────────────────────────────────────

def detect_lanes(frame, lka_enabled: bool = True):
    """
    Full lane detection pipeline.

    Returns a dict with:
      annotated_frame  – BGR frame with overlays
      lka_status       – "ACTIVE" | "DISABLED" | "SEARCHING"
      lane_status      – "CENTERED" | "DRIFTING LEFT" | "DRIFTING RIGHT" | "LANE DEPARTURE WARNING" | "LANE NOT DETECTED"
      steering         – "KEEP STRAIGHT" | "STEER LEFT" | "STEER RIGHT" | "MANUAL CONTROL" | "---"
      steering_angle   – float (degrees)
      correction_val   – float (degrees)
      deviation        – float (metres)
      detection        – human-readable detection note
      left_detected    – bool
      right_detected   – bool
      lane_center_x    – int (pixels)
      vehicle_center_x – int (pixels)
      ldw_active       – bool
      lka_enabled      – bool
      mode             – "LKA ASSISTED" | "MANUAL"
    """
    height, width = frame.shape[:2]

    # ── Step 1-4: pre-process ────────────────
    edges        = preprocess_frame(frame)
    masked_edges = region_of_interest(edges, height, width)

    # ── Step 5: Hough lines ──────────────────
    raw_lines = detect_hough_lines(masked_edges)

    # ── Step 6: separate left / right ────────
    left_raw, right_raw = separate_lanes(raw_lines, width)

    left_avg  = average_line(left_raw)
    right_avg = average_line(right_raw)

    left_detected  = left_avg  is not None
    right_detected = right_avg is not None

    # ── Step 7: compute pixel coordinates ────
    y_start = height           # bottom of frame
    y_end   = int(height * 0.58)  # horizon line

    left_coords  = make_coordinates(*left_avg,  y_start, y_end) if left_detected  else None
    right_coords = make_coordinates(*right_avg, y_start, y_end) if right_detected else None

    # ── Step 8-9: lane & vehicle centres ─────
    vehicle_center_x = width // 2

    if left_coords and right_coords:
        # Both lanes: use average of bottom x-points
        lane_center_x = (left_coords[0] + right_coords[0]) // 2
        detection_note = "LANES DETECTED"
    elif left_coords:
        # Only left lane detected – estimate right boundary
        lane_center_x  = left_coords[0] + int(width * 0.35)
        detection_note = "LEFT LANE ONLY"
    elif right_coords:
        # Only right lane detected – estimate left boundary
        lane_center_x  = right_coords[0] - int(width * 0.35)
        detection_note = "RIGHT LANE ONLY"
    else:
        lane_center_x  = vehicle_center_x  # no detection, assume centered
        detection_note = "NO LANES DETECTED"

    # ── Step 10: deviation ───────────────────
    deviation_m = compute_deviation(lane_center_x, vehicle_center_x, width)
    is_ldw = abs(deviation_m) >= LDW_THRESHOLD

    # ── Step 11-12: status & steering ────────
    if not left_detected and not right_detected:
        lka_status     = "DISABLED" if not lka_enabled else "SEARCHING"
        lane_status    = "LANE NOT DETECTED"
        steering       = "MANUAL CONTROL" if not lka_enabled else "---"
        deviation_m    = 0.0
        correction_val = 0.0
        steer_angle    = 0.0
    else:
        if is_ldw:
            lane_status = "LANE DEPARTURE WARNING"
        else:
            lane_status, _ = get_lane_status(deviation_m)

        if lka_enabled:
            lka_status = "ACTIVE"
            _, steering = get_lane_status(deviation_m)
            # Automatic steering correction angle (proportional to deviation)
            steer_angle = round(-deviation_m * 10.0, 1)
            correction_val = steer_angle
        else:
            lka_status = "DISABLED"
            steering = "MANUAL CONTROL"
            steer_angle = 0.0
            correction_val = 0.0

    # ── Draw overlays ────────────────────────
    annotated = draw_overlays(
        frame.copy(),
        left_coords, right_coords,
        lane_center_x, vehicle_center_x,
        lane_status, steering,
        y_start, y_end,
        height, width,
        lka_enabled=lka_enabled,
        is_ldw=is_ldw,
        deviation_m=deviation_m,
    )

    return {
        "annotated_frame":  annotated,
        "lka_status":       lka_status,
        "lka_enabled":      lka_enabled,
        "mode":             "LKA ASSISTED" if lka_enabled else "MANUAL",
        "lane_status":      lane_status,
        "steering":         steering,
        "steering_angle":   steer_angle,
        "correction_val":   correction_val,
        "deviation":        deviation_m,
        "lateral_pos":      deviation_m,
        "detection":        detection_note,
        "left_detected":    left_detected,
        "right_detected":   right_detected,
        "lane_center_x":    int(lane_center_x),
        "vehicle_center_x": int(vehicle_center_x),
        "ldw_active":       is_ldw,
        "ldw_warning":      is_ldw,
    }


# ─────────────────────────────────────────────
#  Overlay drawing
# ─────────────────────────────────────────────

def draw_overlays(frame, left_coords, right_coords,
                  lane_center_x, vehicle_center_x,
                  lane_status, steering,
                  y_start, y_end, height, width,
                  lka_enabled: bool = True,
                  is_ldw: bool = False,
                  deviation_m: float = 0.0):
    """Draw lane lines, fill, centre markers, and steering arrow on the frame."""

    overlay = frame.copy()

    # ── Lane fill polygon ─────────────────────
    if left_coords and right_coords:
        poly_pts = np.array([[
            (left_coords[0],  left_coords[1]),   # bottom-left
            (left_coords[2],  left_coords[3]),   # top-left
            (right_coords[2], right_coords[3]),  # top-right
            (right_coords[0], right_coords[1]),  # bottom-right
        ]], dtype=np.int32)

        if is_ldw:
            fill_color = (0, 0, 160)       # Red fill on departure
            alpha = 0.35
        elif abs(deviation_m) > 0.2:
            fill_color = (0, 140, 220)     # Amber fill on drift
            alpha = 0.28
        else:
            fill_color = (0, 100, 0) if lka_enabled else (70, 70, 85)
            alpha = 0.25

        cv2.fillPoly(overlay, poly_pts, fill_color)
        frame = cv2.addWeighted(overlay, alpha, frame, 1.0 - alpha, 0)

    # ── Lane lines ───────────────────────────
    line_col = (0, 0, 255) if is_ldw else (0, 200, 255)
    if left_coords:
        cv2.line(frame,
                 (left_coords[0],  left_coords[1]),
                 (left_coords[2],  left_coords[3]),
                 line_col, 4)

    if right_coords:
        cv2.line(frame,
                 (right_coords[0], right_coords[1]),
                 (right_coords[2], right_coords[3]),
                 line_col, 4)

    # ── Lane centre line ─────────────────────
    cv2.line(frame,
             (lane_center_x, y_start),
             (lane_center_x, y_end),
             (0, 255, 0), 2)

    # ── Vehicle centre line ──────────────────
    cv2.line(frame,
             (vehicle_center_x, y_start),
             (vehicle_center_x, y_end),
             (255, 255, 255), 2)

    # ── Top-Left Mode / Status Badge ─────────
    if lka_enabled:
        cv2.rectangle(frame, (18, 16), (220, 52), (15, 45, 20), -1)
        cv2.rectangle(frame, (18, 16), (220, 52), (0, 230, 118), 1)
        cv2.circle(frame, (34, 34), 6, (0, 230, 118), -1)
        cv2.putText(frame, "LKA: ACTIVE", (48, 39), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 245, 140), 2)
    else:
        cv2.rectangle(frame, (18, 16), (235, 52), (25, 25, 30), -1)
        cv2.rectangle(frame, (18, 16), (235, 52), (160, 60, 60), 1)
        cv2.circle(frame, (34, 34), 6, (80, 80, 240), -1)
        cv2.putText(frame, "LKA: DISABLED", (48, 39), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (140, 140, 255), 2)

    # ── Status text ──────────────────────────
    status_color = {
        "CENTERED":              (0, 255, 0),
        "DRIFTING LEFT":         (0, 165, 255),
        "DRIFTING RIGHT":        (0, 165, 255),
        "LANE DEPARTURE WARNING":(0, 0, 255),
        "LANE NOT DETECTED":     (0, 0, 255),
    }.get(lane_status, (255, 255, 255))

    cv2.putText(frame, lane_status, (20, 80),
                cv2.FONT_HERSHEY_SIMPLEX, 0.75, status_color, 2)

    if lka_enabled:
        cv2.putText(frame, steering, (20, 110),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 0), 2)
    else:
        cv2.putText(frame, "MANUAL CONTROL -- SIMULATION ONLY", (20, 110),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (160, 170, 240), 1)

    # ── Steering arrow (ONLY when LKA is enabled!) ──
    arrow_y  = height - 60
    arrow_cx = vehicle_center_x

    if lka_enabled:
        if steering == "STEER LEFT":
            cv2.arrowedLine(frame,
                            (arrow_cx + 60, arrow_y),
                            (arrow_cx - 60, arrow_y),
                            (0, 200, 255), 4, tipLength=0.4)
        elif steering == "STEER RIGHT":
            cv2.arrowedLine(frame,
                            (arrow_cx - 60, arrow_y),
                            (arrow_cx + 60, arrow_y),
                            (0, 200, 255), 4, tipLength=0.4)
        else:
            # Straight arrow (up)
            cv2.arrowedLine(frame,
                            (arrow_cx, arrow_y + 40),
                            (arrow_cx, arrow_y - 10),
                            (0, 255, 0), 3, tipLength=0.5)

    # ── Lane Departure Warning Alert Banner ──
    if is_ldw:
        bx1, by1 = width // 2 - 270, 70
        bx2, by2 = width // 2 + 270, 118
        cv2.rectangle(frame, (bx1, by1), (bx2, by2), (20, 20, 180), -1)
        cv2.rectangle(frame, (bx1, by1), (bx2, by2), (50, 60, 255), 2)
        cv2.putText(frame, "!! LANE DEPARTURE WARNING !!",
                    (bx1 + 25, by1 + 32),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

    # ── "FSD-STYLE SIMULATION" watermark ────
    cv2.putText(frame, "FSD-STYLE SIMULATION | Software Simulation Only",
                (10, height - 15),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (180, 180, 180), 1)

    return frame


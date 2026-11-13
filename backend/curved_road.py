"""
Curved Road Simulator & Vehicle Dynamics Model for LKA Simulation
==================================================================
Provides:
  - 3D perspective projection of realistic curved roads (Straight, Gentle L/R, Sharp L/R, S-Curve)
  - Animated forward motion with continuous lane dashes & road verges
  - Vehicle lateral dynamics model (LKA ON vs LKA OFF)
  - Proportional-Derivative (PD) lane keeping controller
  - Real-time OpenCV rendering with HUD, trajectory, deviation & warnings
"""

import math
import time
import cv2
import numpy as np

# ─────────────────────────────────────────────
#  Constants & Physical Parameters
# ─────────────────────────────────────────────
FRAME_WIDTH  = 960
FRAME_HEIGHT = 540

# Camera & Perspective
HORIZON_Y    = 220      # Vanishing horizon in pixels
CAM_HEIGHT   = 1.2      # Camera height above ground in metres
FOCAL_PX     = 800.0    # Virtual focal length in pixels
LANE_WIDTH_M = 3.5      # Standard lane width in metres
Z_NEAR       = 2.8      # Distance in metres at bottom of frame (y=540)
Z_FAR        = 75.0     # Maximum render distance in metres

# Vehicle Dynamics
SPEED_MPS    = 16.67    # Forward speed: ~60 km/h (16.67 m/s)
WHEELBASE_M  = 2.7      # Typical sedan wheelbase
DEVIATION_THRESHOLD = 0.05  # Metres within which vehicle is considered CENTERED
LDW_THRESHOLD       = 0.75  # Metres at which Lane Departure Warning triggers
MAX_DRIFT_M         = 2.2   # Clamp lateral drift on shoulder


class CurvedRoadSimulator:
    def __init__(self):
        # Simulation Configuration
        self.road_type   = "straight"
        self.lka_enabled = True

        # Vehicle State
        self.lat_pos     = 0.0     # Metres from lane centre (positive = right, negative = left)
        self.lat_vel     = 0.0     # Lateral velocity in m/s
        self.heading     = 0.0     # Heading angle relative to road in radians
        self.steer_angle = 0.0     # Steering angle in degrees
        self.corr_val    = 0.0     # Active LKA correction value in degrees

        # Animation state
        self.distance_traveled = 0.0
        self.last_update_time  = time.time()
        self.fps               = 30.0
        self.frame_count       = 0

        # Controller parameters (PD + Feedforward)
        self.kp = 13.0     # Steering deg per metre of lateral error
        self.kd = 7.5      # Steering deg per (m/s) of lateral velocity
        self.kf = 1.0      # Feedforward gain for road curvature

    def set_road_type(self, road_type: str):
        """Update road geometry type."""
        valid_types = {
            "straight",
            "gentle_left",
            "gentle_right",
            "sharp_left",
            "sharp_right",
            "s_curve",
        }
        if road_type in valid_types:
            self.road_type = road_type

    def set_lka_enabled(self, enabled: bool):
        """Toggle LKA assistance."""
        self.lka_enabled = bool(enabled)
        if not self.lka_enabled:
            self.corr_val = 0.0

    def reset_vehicle(self):
        """Reset simulated vehicle to the lane centre."""
        self.lat_pos     = 0.0
        self.lat_vel     = 0.0
        self.heading     = 0.0
        self.steer_angle = 0.0
        self.corr_val    = 0.0
        self.last_update_time = time.time()


    # ─────────────────────────────────────────────
    #  Road Curvature Mathematics
    # ─────────────────────────────────────────────
    def get_curvature(self, z: float = 0.0) -> float:
        """
        Return road curvature kappa (1/m) at distance z ahead of vehicle.
        Negative = curves left, Positive = curves right.
        """
        if self.road_type == "straight":
            return 0.0
        elif self.road_type == "gentle_left":
            return -0.0016
        elif self.road_type == "gentle_right":
            return +0.0016
        elif self.road_type == "sharp_left":
            return -0.0036
        elif self.road_type == "sharp_right":
            return +0.0036
        elif self.road_type == "s_curve":
            # S-Curve: oscillates with road distance s
            s = self.distance_traveled + z
            # First left, then right with period 80m
            return -0.0028 * math.cos(2.0 * math.pi * (s % 90.0) / 90.0)
        return 0.0

    def get_road_center_x(self, z: float) -> float:
        """
        Return lateral coordinate X (metres) of the road centerline
        at distance z ahead of vehicle.
        """
        if self.road_type == "straight":
            return 0.0
        elif self.road_type == "gentle_left":
            return -0.5 * 0.0016 * (z ** 2)
        elif self.road_type == "gentle_right":
            return +0.5 * 0.0016 * (z ** 2)
        elif self.road_type == "sharp_left":
            return -0.5 * 0.0036 * (z ** 2)
        elif self.road_type == "sharp_right":
            return +0.5 * 0.0036 * (z ** 2)
        elif self.road_type == "s_curve":
            # Continuous S-curve profile
            s = self.distance_traveled + z
            # S-shape: amplitude ~3.2m
            return -3.2 * math.sin(2.0 * math.pi * (s % 90.0) / 90.0)
        return 0.0

    # ─────────────────────────────────────────────
    #  Vehicle Physics Update
    # ─────────────────────────────────────────────
    def update_physics(self, dt: float):
        """
        Step vehicle lateral dynamics forward by dt seconds.
        """
        # Clamp dt to avoid simulation jumps during lag
        dt = max(0.005, min(dt, 0.08))

        kappa = self.get_curvature(z=0.0)
        self.distance_traveled += SPEED_MPS * dt

        if self.lka_enabled:
            # ── LKA ON: Active PD Controller ─────────────────
            # Lateral error e = lat_pos (positive = right of center)
            # Desired correction steers left when lat_pos > 0
            curv_deg = math.degrees(kappa * WHEELBASE_M)
            target_steer = -(self.kp * self.lat_pos + self.kd * self.lat_vel) - (self.kf * curv_deg)

            # Clamp steering angle to realistic physical limit
            target_steer = max(-22.0, min(22.0, target_steer))
            self.corr_val = round(target_steer, 2)

            # Smooth actuator response (rate limit ~45 deg/s)
            steer_diff = target_steer - self.steer_angle
            max_rate = 45.0 * dt
            if abs(steer_diff) <= max_rate:
                self.steer_angle = target_steer
            else:
                self.steer_angle += math.copysign(max_rate, steer_diff)

            # Yaw rate response
            steer_rad = math.radians(self.steer_angle)
            yaw_rate  = (SPEED_MPS / WHEELBASE_M) * steer_rad + (SPEED_MPS * kappa)
            self.heading += yaw_rate * dt
            self.heading *= 0.94   # Natural heading alignment damping

            # Lateral velocity & position integration
            self.lat_vel = SPEED_MPS * math.sin(self.heading)
            self.lat_pos += self.lat_vel * dt

        else:
            # ── LKA OFF: Manual Control (No automatic correction) ──
            self.corr_val    = 0.0
            self.steer_angle = 0.0   # Steering neutral / unassisted

            # Centrifugal road drift: when road curves, unsteered vehicle drifts outwards
            # Left curve (kappa < 0) -> vehicle drifts right (lat_pos increases)
            # Right curve (kappa > 0) -> vehicle drifts left (lat_pos decreases)
            yaw_rate = -SPEED_MPS * kappa * 0.85
            self.heading += yaw_rate * dt
            # Small natural drift noise/momentum
            self.heading = max(-0.15, min(0.15, self.heading))

            self.lat_vel = SPEED_MPS * math.sin(self.heading)
            # If on straight road with initial perturbation, drift slowly continues
            if abs(self.lat_pos) > 0.1 and abs(kappa) < 0.0001:
                self.lat_pos += math.copysign(0.15 * dt, self.lat_pos)
            else:
                self.lat_pos += self.lat_vel * dt

        # Clamp lateral position at road boundary / shoulder
        self.lat_pos = max(-MAX_DRIFT_M, min(MAX_DRIFT_M, self.lat_pos))

    # ─────────────────────────────────────────────
    #  3D Projection Helper
    # ─────────────────────────────────────────────
    def project_point(self, x_world: float, z_world: float):
        """
        Project 3D world coordinate (X, Z) in front of vehicle onto 2D screen (x, y).
        Camera is centered at X_veh = lat_pos, height = CAM_HEIGHT.
        """
        if z_world <= 0.1:
            z_world = 0.1

        # X relative to vehicle camera
        rel_x = x_world - self.lat_pos

        # Perspective equations
        screen_x = int(FRAME_WIDTH / 2.0 + (FOCAL_PX * rel_x) / z_world)
        screen_y = int(HORIZON_Y + (FOCAL_PX * CAM_HEIGHT) / z_world)
        return screen_x, screen_y

    # ─────────────────────────────────────────────
    #  Frame Generation & Rendering
    # ─────────────────────────────────────────────
    def generate_frame(self) -> dict:
        """
        Compute physics update and render full annotated OpenCV frame.
        Returns dict containing annotated_frame and telemetry status.
        """
        now = time.time()
        dt  = now - self.last_update_time
        self.last_update_time = now
        self.frame_count += 1

        # Update physics
        self.update_physics(dt)

        # ── Step 1: Base Canvas & Environment ───────────────
        frame = np.zeros((FRAME_HEIGHT, FRAME_WIDTH, 3), dtype=np.uint8)

        # Sky Gradient (deep navy/slate to dark teal horizon)
        for y in range(HORIZON_Y):
            t = y / float(HORIZON_Y)
            r = int(12 * (1 - t) + 20 * t)
            g = int(16 * (1 - t) + 32 * t)
            b = int(28 * (1 - t) + 48 * t)
            frame[y, :] = (b, g, r)

        # Horizon Line Glow
        cv2.line(frame, (0, HORIZON_Y), (FRAME_WIDTH, HORIZON_Y), (70, 95, 120), 1)

        # Ground / Grass Verges (dark subtle terrain)
        frame[HORIZON_Y:, :] = (26, 32, 28)

        # ── Step 2: Compute 3D Road Profile Points ──────────
        z_steps = np.linspace(Z_NEAR, Z_FAR, 60)
        left_pts  = []
        right_pts = []
        center_pts= []

        half_w = LANE_WIDTH_M / 2.0

        for z in z_steps:
            rc_x = self.get_road_center_x(z)
            left_x  = rc_x - half_w
            right_x = rc_x + half_w

            sx_l, sy = self.project_point(left_x, z)
            sx_r, _  = self.project_point(right_x, z)
            sx_c, _  = self.project_point(rc_x, z)

            if sy >= HORIZON_Y and sy <= FRAME_HEIGHT + 30:
                left_pts.append((sx_l, sy))
                right_pts.append((sx_r, sy))
                center_pts.append((sx_c, sy))

        # ── Step 3: Draw Road Asphalt ───────────────────────
        if len(left_pts) >= 2 and len(right_pts) >= 2:
            road_poly = np.array(left_pts + right_pts[::-1], dtype=np.int32)
            # Dark asphalt surface
            cv2.fillPoly(frame, [road_poly], (36, 40, 44))

            # Shoulder curbs (textured outer borders)
            curb_l = np.array([left_pts[i] for i in range(0, len(left_pts), 2)], dtype=np.int32)
            curb_r = np.array([right_pts[i] for i in range(0, len(right_pts), 2)], dtype=np.int32)
            if len(curb_l) >= 2:
                cv2.polylines(frame, [curb_l], isClosed=False, color=(60, 68, 76), thickness=6)
            if len(curb_r) >= 2:
                cv2.polylines(frame, [curb_r], isClosed=False, color=(60, 68, 76), thickness=6)

        # ── Step 4: Translucent Lane Fill & Warning Colors ──
        overlay = frame.copy()
        dev_abs = abs(self.lat_pos)
        is_ldw  = dev_abs >= LDW_THRESHOLD

        if is_ldw:
            # Flashing or prominent red alert fill
            fill_color = (25, 30, 180)   # Red / Danger
            alpha = 0.38
        elif dev_abs > 0.20:
            fill_color = (20, 140, 220)  # Amber / Warning
            alpha = 0.28
        else:
            if self.lka_enabled:
                fill_color = (30, 150, 45)   # Green / Safe LKA
                alpha = 0.26
            else:
                fill_color = (80, 80, 95)    # Neutral / Manual
                alpha = 0.18

        if len(left_pts) >= 2 and len(right_pts) >= 2:
            cv2.fillPoly(overlay, [road_poly], fill_color)
            frame = cv2.addWeighted(overlay, alpha, frame, 1.0 - alpha, 0)

        # ── Step 5: Draw Solid Curved Left & Right Lane Lines
        if len(left_pts) >= 2:
            pts_l = np.array(left_pts, dtype=np.int32)
            # Anti-aliased solid lane marking (bright amber / white)
            line_color = (0, 215, 255) if is_ldw and self.lat_pos < 0 else (240, 240, 245)
            cv2.polylines(frame, [pts_l], isClosed=False, color=line_color, thickness=4, lineType=cv2.LINE_AA)

        if len(right_pts) >= 2:
            pts_r = np.array(right_pts, dtype=np.int32)
            line_color = (0, 215, 255) if is_ldw and self.lat_pos > 0 else (240, 240, 245)
            cv2.polylines(frame, [pts_r], isClosed=False, color=line_color, thickness=4, lineType=cv2.LINE_AA)

        # ── Step 6: Animated Dashed Center Line ─────────────
        # In world coordinates, dashes repeat every 9m (3m dash, 6m gap)
        for i in range(len(z_steps) - 1):
            z0 = z_steps[i]
            z1 = z_steps[i + 1]
            world_dist = self.distance_traveled + z0
            dash_phase = world_dist % 9.0

            if dash_phase < 3.2:
                rc_x0 = self.get_road_center_x(z0)
                rc_x1 = self.get_road_center_x(z1)
                p0 = self.project_point(rc_x0, z0)
                p1 = self.project_point(rc_x1, z1)
                if p0[1] >= HORIZON_Y and p1[1] <= FRAME_HEIGHT + 20:
                    cv2.line(frame, p0, p1, (0, 220, 240), 2, lineType=cv2.LINE_AA)

        # ── Step 7: Centerlines & Lateral Deviation Vector ──
        # Vehicle Center Line (White, from screen bottom center)
        veh_cx = FRAME_WIDTH // 2
        cv2.line(frame, (veh_cx, FRAME_HEIGHT), (veh_cx, FRAME_HEIGHT - 95), (255, 255, 255), 2, lineType=cv2.LINE_AA)

        # Lane Center at car bumper (Z_NEAR)
        lane_bottom_x, lane_bottom_y = self.project_point(self.get_road_center_x(Z_NEAR), Z_NEAR)
        lane_bottom_y = min(FRAME_HEIGHT - 10, max(HORIZON_Y + 10, lane_bottom_y))

        # Lane Center marker line (Green dashed appearance)
        cv2.line(frame, (lane_bottom_x, FRAME_HEIGHT), (lane_bottom_x, FRAME_HEIGHT - 95), (0, 240, 120), 2, lineType=cv2.LINE_AA)

        # Deviation connection vector between vehicle center and lane center
        cv2.line(frame, (veh_cx, FRAME_HEIGHT - 45), (lane_bottom_x, FRAME_HEIGHT - 45), (0, 220, 255), 2, lineType=cv2.LINE_AA)
        cv2.circle(frame, (veh_cx, FRAME_HEIGHT - 45), 4, (255, 255, 255), -1)
        cv2.circle(frame, (lane_bottom_x, FRAME_HEIGHT - 45), 4, (0, 240, 120), -1)

        # ── Step 8: Ego Vehicle Rendering ───────────────────
        self._draw_ego_vehicle(frame, veh_cx, FRAME_HEIGHT - 40)

        # ── Step 9: Predictive Trajectory / Steering Arrow ──
        if self.lka_enabled:
            self._draw_trajectory(frame, veh_cx, lane_bottom_x)
        else:
            self._draw_manual_drift_indicator(frame, veh_cx)

        # ── Step 10: HUD Badges & Alerts ────────────────────
        self._draw_hud(frame, is_ldw)

        # ── Step 11: Determine Status Strings ───────────────
        if is_ldw:
            lane_status = "LANE DEPARTURE WARNING"
        elif self.lat_pos < -DEVIATION_THRESHOLD:
            lane_status = "DRIFTING LEFT"
        elif self.lat_pos > DEVIATION_THRESHOLD:
            lane_status = "DRIFTING RIGHT"
        else:
            lane_status = "CENTERED"

        if self.lka_enabled:
            lka_status = "ACTIVE"
            if self.corr_val > 1.2:
                steering_str = "STEER RIGHT"
            elif self.corr_val < -1.2:
                steering_str = "STEER LEFT"
            else:
                steering_str = "KEEP STRAIGHT"
        else:
            lka_status   = "DISABLED"
            steering_str = "MANUAL CONTROL"

        road_names = {
            "straight":     "Straight Road",
            "gentle_left":  "Gentle Left Curve",
            "gentle_right": "Gentle Right Curve",
            "sharp_left":   "Sharp Left Curve",
            "sharp_right":  "Sharp Right Curve",
            "s_curve":      "S-Curve",
        }

        return {
            "annotated_frame": frame,
            "lka_status":      lka_status,
            "lka_enabled":     self.lka_enabled,
            "mode":            "LKA ASSISTED" if self.lka_enabled else "MANUAL",
            "lane_status":     lane_status,
            "steering":        steering_str,
            "steering_angle":  round(self.steer_angle, 1),
            "correction_val":  round(self.corr_val, 1) if self.lka_enabled else 0.0,
            "deviation":       round(self.lat_pos, 3),
            "lateral_pos":     round(self.lat_pos, 3),
            "road_type":       self.road_type,
            "road_name":       road_names.get(self.road_type, self.road_type),
            "curvature":       round(self.get_curvature(0.0), 5),
            "ldw_active":      is_ldw,
            "ldw_warning":     is_ldw,
            "detection":       "LANES DETECTED",
            "frame_count":     self.frame_count,
            "fps":             round(1.0 / dt if dt > 0 else 30.0, 1),
            "running":         True,
            "simulation_mode": "curved_road",
        }

    # ─────────────────────────────────────────────
    #  HUD & Visual Sub-Routines
    # ─────────────────────────────────────────────
    def _draw_ego_vehicle(self, img, cx: int, cy: int):
        """Draw sleek modern ego vehicle at bottom center."""
        # Vehicle Shadow
        cv2.ellipse(img, (cx, cy + 18), (34, 12), 0, 0, 360, (15, 15, 18), -1)

        # Vehicle Body (dark obsidian metallic)
        body_pts = np.array([
            (cx - 24, cy + 16),
            (cx + 24, cy + 16),
            (cx + 21, cy - 26),
            (cx - 21, cy - 26),
        ], dtype=np.int32)
        cv2.fillPoly(img, [body_pts], (42, 45, 52))
        cv2.polylines(img, [body_pts], True, (90, 100, 115), 1, lineType=cv2.LINE_AA)

        # Windshield / Roof Glass (tinted cyan glass)
        glass_pts = np.array([
            (cx - 16, cy + 6),
            (cx + 16, cy + 6),
            (cx + 13, cy - 18),
            (cx - 13, cy - 18),
        ], dtype=np.int32)
        cv2.fillPoly(img, [glass_pts], (140, 180, 205))

        # Taillights (vibrant LED red strip)
        cv2.line(img, (cx - 21, cy + 15), (cx - 8, cy + 15), (25, 30, 240), 3, lineType=cv2.LINE_AA)
        cv2.line(img, (cx + 8, cy + 15), (cx + 21, cy + 15), (25, 30, 240), 3, lineType=cv2.LINE_AA)

    def _draw_trajectory(self, img, start_x: int, target_x: int):
        """Draw dynamic curved corrective trajectory line when LKA is active."""
        pts = []
        for t in np.linspace(0.0, 1.0, 15):
            y = int((FRAME_HEIGHT - 50) * (1 - t) + (HORIZON_Y + 70) * t)
            # Cubic bezier ease from vehicle center toward lane center
            interp_x = start_x + (target_x - start_x) * (3 * t**2 - 2 * t**3)
            # Add road curve influence
            interp_x += math.sin(t * math.pi) * (self.corr_val * 2.2)
            pts.append((int(interp_x), y))

        if len(pts) >= 2:
            traj_poly = np.array(pts, dtype=np.int32)
            cv2.polylines(img, [traj_poly], isClosed=False, color=(0, 240, 180), thickness=2, lineType=cv2.LINE_AA)

        # Steering arrow
        arrow_y = FRAME_HEIGHT - 65
        if self.corr_val > 1.2:
            cv2.arrowedLine(img, (start_x - 35, arrow_y), (start_x + 35, arrow_y), (0, 230, 140), 3, tipLength=0.35)
        elif self.corr_val < -1.2:
            cv2.arrowedLine(img, (start_x + 35, arrow_y), (start_x - 35, arrow_y), (0, 230, 140), 3, tipLength=0.35)
        else:
            cv2.arrowedLine(img, (start_x, arrow_y + 20), (start_x, arrow_y - 20), (0, 230, 140), 2, tipLength=0.35)

    def _draw_manual_drift_indicator(self, img, cx: int):
        """Draw subtle manual drift arrow when LKA is disabled."""
        arrow_y = FRAME_HEIGHT - 65
        if abs(self.heading) > 0.015:
            drift_dir = 1 if self.heading > 0 else -1
            cv2.arrowedLine(img, (cx, arrow_y), (cx + drift_dir * 30, arrow_y), (140, 140, 160), 2, tipLength=0.4)

    def _draw_hud(self, img, is_ldw: bool):
        """Draw modern FSD-style HUD badges and alerts."""
        # Top-Left: LKA Mode Badge
        if self.lka_enabled:
            # Green ACTIVE badge
            cv2.rectangle(img, (18, 18), (220, 56), (15, 45, 20), -1)
            cv2.rectangle(img, (18, 18), (220, 56), (0, 230, 118), 1)
            cv2.circle(img, (36, 37), 6, (0, 230, 118), -1)
            cv2.putText(img, "LKA: ACTIVE", (52, 42), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 245, 140), 2, cv2.LINE_AA)
        else:
            # Red/Grey DISABLED badge
            cv2.rectangle(img, (18, 18), (235, 56), (25, 25, 30), -1)
            cv2.rectangle(img, (18, 18), (235, 56), (160, 60, 60), 1)
            cv2.circle(img, (36, 37), 6, (80, 80, 240), -1)
            cv2.putText(img, "LKA: DISABLED", (52, 42), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (140, 140, 255), 2, cv2.LINE_AA)

        # Top-Right: Road Type & Curvature
        road_labels = {
            "straight":     "STRAIGHT ROAD",
            "gentle_left":  "GENTLE LEFT CURVE",
            "gentle_right": "GENTLE RIGHT CURVE",
            "sharp_left":   "SHARP LEFT CURVE",
            "sharp_right":  "SHARP RIGHT CURVE",
            "s_curve":      "S-CURVE ROAD",
        }
        r_text = road_labels.get(self.road_type, "ROAD")
        cv2.putText(img, r_text, (FRAME_WIDTH - 280, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 220, 240), 2, cv2.LINE_AA)

        # Bottom-Center: Mode Notice
        if not self.lka_enabled:
            cv2.putText(img, "MANUAL CONTROL -- SIMULATION ONLY",
                        (FRAME_WIDTH // 2 - 195, FRAME_HEIGHT - 90),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (160, 170, 240), 1, cv2.LINE_AA)

        # Center Alert Banner if Lane Departure Warning
        if is_ldw:
            # Prominent Red Alert Banner
            bx1, by1 = FRAME_WIDTH // 2 - 270, 80
            bx2, by2 = FRAME_WIDTH // 2 + 270, 130
            cv2.rectangle(img, (bx1, by1), (bx2, by2), (20, 20, 180), -1)
            cv2.rectangle(img, (bx1, by1), (bx2, by2), (50, 60, 255), 2)
            cv2.putText(img, "!! LANE DEPARTURE WARNING !!",
                        (bx1 + 25, by1 + 34),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2, cv2.LINE_AA)

        # Bottom watermark
        cv2.putText(img, "FSD-STYLE SIMULATION | Software Simulation Only",
                    (14, FRAME_HEIGHT - 14),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (150, 160, 175), 1, cv2.LINE_AA)


# Global simulator singleton
simulator = CurvedRoadSimulator()

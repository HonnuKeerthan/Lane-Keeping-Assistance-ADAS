"""
Verification Suite for Upgraded LKA Simulation (12 Scenarios)
=============================================================
Tests:
  1. Straight road with LKA ON
  2. Straight road with LKA OFF
  3. Left curve (gentle) with LKA ON
  4. Left curve (gentle) with LKA OFF
  5. Right curve (gentle) with LKA ON
  6. Right curve (gentle) with LKA OFF
  7. S-curve with LKA ON
  8. S-curve with LKA OFF
  9. Vehicle approaching a lane boundary
  10. Vehicle crossing a lane boundary (LDW Alert)
  11. Switching LKA ON and OFF during running simulation
  12. Resetting vehicle after lane departure
"""

import time
import requests
import sys

def get_base_url():
    import socket
    for port in [5005, 5000, 5050]:
        try:
            r = requests.get(f"http://127.0.0.1:{port}/status", timeout=1)
            if r.status_code == 200:
                data = r.json()
                if "lka_status" in data:
                    return f"http://127.0.0.1:{port}"
        except Exception:
            continue
    return "http://127.0.0.1:5005"

BASE_URL = get_base_url()


def log(msg, status="INFO"):
    symbol = {"INFO": "ℹ", "PASS": "✓", "FAIL": "✗", "WARN": "⚠"}.get(status, "·")
    print(f"[{symbol}] {msg}")

def check(condition, desc):
    if condition:
        log(f"PASS: {desc}", "PASS")
        return True
    else:
        log(f"FAIL: {desc}", "FAIL")
        return False

def run_tests():
    all_passed = True
    print("\n" + "=" * 60)
    print("  RUNNING LKA SIMULATION AUTOMATED VERIFICATION SUITE")
    print("=" * 60 + "\n")

    # ── Test 1: Straight road with LKA ON ─────────────────────
    log("Scenario 1: Straight road with LKA ON")
    requests.post(f"{BASE_URL}/reset")
    requests.post(f"{BASE_URL}/set_lka", json={"enabled": True})
    requests.post(f"{BASE_URL}/set_road", json={"road_type": "straight"})
    requests.post(f"{BASE_URL}/start", json={"road_type": "straight"})
    time.sleep(1.5)
    s = requests.get(f"{BASE_URL}/status").json()
    all_passed &= check(s["lka_enabled"] is True and s["lka_status"] == "ACTIVE", "LKA is ACTIVE and enabled")
    all_passed &= check(s["mode"] == "LKA ASSISTED", "Mode is LKA ASSISTED")
    all_passed &= check(abs(s["deviation"]) <= 0.15, f"Vehicle stays centered on straight road (dev={s['deviation']}m)")
    all_passed &= check(s["steering"] in ["KEEP STRAIGHT", "STEER LEFT", "STEER RIGHT"], f"Steering valid ({s['steering']})")

    # ── Test 2: Straight road with LKA OFF ────────────────────
    log("Scenario 2: Straight road with LKA OFF")
    requests.post(f"{BASE_URL}/set_lka", json={"enabled": False})
    time.sleep(1.0)
    s = requests.get(f"{BASE_URL}/status").json()
    all_passed &= check(s["lka_enabled"] is False and s["lka_status"] == "DISABLED", "LKA is DISABLED")
    all_passed &= check(s["mode"] == "MANUAL", "Mode is MANUAL")
    all_passed &= check(s["correction_val"] == 0.0, f"Steering correction is 0.0 when LKA is OFF (val={s['correction_val']})")
    all_passed &= check(s["steering"] == "MANUAL CONTROL", "Steering shows MANUAL CONTROL")

    # ── Test 3: Left curve with LKA ON ────────────────────────
    log("Scenario 3: Left curve with LKA ON")
    requests.post(f"{BASE_URL}/reset_vehicle")
    requests.post(f"{BASE_URL}/set_lka", json={"enabled": True})
    requests.post(f"{BASE_URL}/set_road", json={"road_type": "gentle_left"})
    time.sleep(2.0)
    s = requests.get(f"{BASE_URL}/status").json()
    all_passed &= check(s["road_type"] == "gentle_left", "Road type is gentle_left")
    all_passed &= check(s["lka_enabled"] is True, "LKA is enabled")
    all_passed &= check(abs(s["deviation"]) <= 0.35, f"LKA keeps vehicle within lane on left curve (dev={s['deviation']}m)")
    all_passed &= check(s["correction_val"] != 0.0 or s["steering"] == "KEEP STRAIGHT", f"Active steering correction applied ({s['correction_val']} deg)")

    # ── Test 4: Left curve with LKA OFF ───────────────────────
    log("Scenario 4: Left curve with LKA OFF (Natural Drift)")
    requests.post(f"{BASE_URL}/reset_vehicle")
    requests.post(f"{BASE_URL}/set_lka", json={"enabled": False})
    # Allow 2.5 seconds to drift outward due to centrifugal curve effect
    time.sleep(2.5)
    s = requests.get(f"{BASE_URL}/status").json()
    all_passed &= check(s["lka_enabled"] is False, "LKA is disabled")
    all_passed &= check(s["correction_val"] == 0.0, "Correction remains 0.0")
    all_passed &= check(s["deviation"] > 0.15, f"Vehicle naturally drifted outward/right on left curve (dev={s['deviation']}m)")

    # ── Test 5: Right curve with LKA ON ───────────────────────
    log("Scenario 5: Right curve with LKA ON")
    requests.post(f"{BASE_URL}/reset_vehicle")
    requests.post(f"{BASE_URL}/set_lka", json={"enabled": True})
    requests.post(f"{BASE_URL}/set_road", json={"road_type": "gentle_right"})
    time.sleep(2.0)
    s = requests.get(f"{BASE_URL}/status").json()
    all_passed &= check(s["road_type"] == "gentle_right", "Road type is gentle_right")
    all_passed &= check(abs(s["deviation"]) <= 0.35, f"LKA stabilizes vehicle on right curve (dev={s['deviation']}m)")

    # ── Test 6: Right curve with LKA OFF ──────────────────────
    log("Scenario 6: Right curve with LKA OFF (Natural Drift)")
    requests.post(f"{BASE_URL}/reset_vehicle")
    requests.post(f"{BASE_URL}/set_lka", json={"enabled": False})
    time.sleep(2.5)
    s = requests.get(f"{BASE_URL}/status").json()
    all_passed &= check(s["lka_enabled"] is False, "LKA is disabled")
    all_passed &= check(s["correction_val"] == 0.0, "Correction is 0.0")
    all_passed &= check(s["deviation"] < -0.15, f"Vehicle drifted outward/left on right curve (dev={s['deviation']}m)")

    # ── Test 7: S-curve with LKA ON ───────────────────────────
    log("Scenario 7: S-curve with LKA ON")
    requests.post(f"{BASE_URL}/reset_vehicle")
    requests.post(f"{BASE_URL}/set_lka", json={"enabled": True})
    requests.post(f"{BASE_URL}/set_road", json={"road_type": "s_curve"})
    time.sleep(2.5)
    s = requests.get(f"{BASE_URL}/status").json()
    all_passed &= check(s["road_type"] == "s_curve", "Road type is s_curve")
    all_passed &= check(abs(s["deviation"]) <= 0.65, f"LKA controls vehicle along S-curve (dev={s['deviation']}m)")

    # ── Test 8: S-curve with LKA OFF ──────────────────────────
    log("Scenario 8: S-curve with LKA OFF")
    requests.post(f"{BASE_URL}/reset_vehicle")
    requests.post(f"{BASE_URL}/set_lka", json={"enabled": False})
    time.sleep(2.5)
    s = requests.get(f"{BASE_URL}/status").json()
    all_passed &= check(s["lka_enabled"] is False, "LKA is disabled")
    all_passed &= check(s["correction_val"] == 0.0, "No automatic correction applied")

    # ── Test 9 & 10: Approaching boundary & Lane Departure Warning
    log("Scenario 9 & 10: Vehicle crossing lane boundary -> LDW Active")
    requests.post(f"{BASE_URL}/set_road", json={"road_type": "sharp_left"})
    requests.post(f"{BASE_URL}/set_lka", json={"enabled": False})
    # Wait for vehicle to drift beyond LDW threshold (>= 0.75m)
    time.sleep(2.5)
    s = requests.get(f"{BASE_URL}/status").json()
    all_passed &= check(abs(s["deviation"]) >= 0.75, f"Vehicle crossed LDW threshold (dev={s['deviation']}m >= 0.75m)")
    all_passed &= check(s["ldw_active"] is True, "Lane Departure Warning (LDW) is ACTIVE")
    all_passed &= check(s["lane_status"] == "LANE DEPARTURE WARNING", "Lane status is LANE DEPARTURE WARNING")

    # ── Test 11: Switch LKA ON during running simulation ──────
    log("Scenario 11: Switching LKA ON while off-center during running simulation")
    requests.post(f"{BASE_URL}/set_lka", json={"enabled": True})
    # Give controller 2.5 seconds to correct vehicle back toward center
    time.sleep(2.5)
    s = requests.get(f"{BASE_URL}/status").json()
    all_passed &= check(s["lka_enabled"] is True, "LKA switched back to ON")
    all_passed &= check(abs(s["deviation"]) < 0.45, f"LKA successfully re-centered vehicle (dev reduced to {s['deviation']}m)")

    # ── Test 12: Reset vehicle after lane departure ───────────
    log("Scenario 12: Resetting vehicle after lane departure")
    requests.post(f"{BASE_URL}/set_lka", json={"enabled": False})
    requests.post(f"{BASE_URL}/set_road", json={"road_type": "straight"})
    time.sleep(1.0)
    # Now reset vehicle
    resp = requests.post(f"{BASE_URL}/reset_vehicle").json()
    s = resp.get("status") or requests.get(f"{BASE_URL}/status").json()
    all_passed &= check(abs(s["deviation"]) <= 0.05, f"Vehicle reset directly to lane center (dev={s['deviation']}m)")
    all_passed &= check(s["ldw_active"] is False, "LDW cleared upon reset")


    # Stop simulation
    requests.post(f"{BASE_URL}/stop")

    print("\n" + "=" * 60)
    if all_passed:
        print("  ALL 12 SCENARIO TESTS PASSED SUCCESSFULLY! ✓")
    else:
        print("  SOME TESTS FAILED! ✗")
    print("=" * 60 + "\n")
    return all_passed

if __name__ == "__main__":
    success = run_tests()
    sys.exit(0 if success else 1)

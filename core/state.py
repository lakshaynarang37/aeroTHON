"""
core/state.py
─────────────────────────────────────────────────────────────
Central shared state for SkyScan.

ADDITIONS vs original:
  • current_alt_m          — live AGL altitude (written by telemetry)
  • target_alt_m           — desired altitude for active hold
  • orange_observations_A  — trig-projection observations (method A)
  • orange_observations_B  — bearing-offset observations (method B)
  Both lists cleared at phase 3 entry; read by projection fusion.
  Method C (weighted centroid) accumulator lives in projection.py.
─────────────────────────────────────────────────────────────
"""

import threading
from mavsdk.offboard import PositionNedYaw, VelocityBodyYawspeed

# ── Mission FSM ──────────────────────────────────────────────
mission_state: str = "BOOTING"

# ── QR / Target Memory ───────────────────────────────────────
target_id: str | None = None

# ── Navigation ───────────────────────────────────────────────
orange_exit_location: tuple[float, float] | None = None

# Dual-method orange location accumulators
# Method A: trigonometric pixel→ground projection
# Method B: bearing-offset dead-reckoning
# Method C: weighted centroid — lives in projection.py, reset via reset_weighted_centroid()
orange_observations_A: list[tuple[float, float]] = []
orange_observations_B: list[tuple[float, float]] = []

current_pos: tuple[float, float] = (0.0, 0.0)
current_yaw: float = 0.0

# Altitude tracking for active hold
current_alt_m: float = 0.0         # live AGL altitude (positive up), written by telemetry
target_alt_m:  float = 5.0         # desired altitude — updated by each phase on entry

# ── Offboard Setpoints ───────────────────────────────────────
offboard_setpoint: PositionNedYaw = PositionNedYaw(0, 0, 0, 0)
body_vel: VelocityBodyYawspeed    = VelocityBodyYawspeed(0, 0, 0, 0)
use_velocity_mode: bool = False

# ── Vision (thread-safe) ─────────────────────────────────────
_vision_lock = threading.Lock()
_vision: dict = {
    "qr_detected":     False, "qr_id": None, "qr_rect": None,
    "qr_ex":           0,     "qr_ey": 0,
    "green_detected":  False, "green_ex": 0, "green_sector": 0,
    "orange_detected": False, "orange_ex": 0, "orange_sector": 0,
    "red_detected":    False, "red_ex": 0,  "red_ey": 0,
    "green_line": None,  "green_angle": 0.0,
    "orange_line": None, "orange_angle": 0.0,
}


def read_vision() -> dict:
    with _vision_lock:
        return dict(_vision)


def write_vision(updates: dict) -> None:
    with _vision_lock:
        _vision.update(updates)


# ── Camera health ─────────────────────────────────────────────
camera_connected: bool = False

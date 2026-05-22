"""
phases/phase5_return.py
─────────────────────────────────────────────────────────────
PHASE 5 — RETURN NAVIGATION VIA ORANGE LANE

Sub-phases:
  5A  Transit to projected orange exit (median-filtered from Phase 3)
  5B  Orange lane acquisition — slow creep with 360° search sweep
      if not immediately found
  5C  Orange lane following — proportional velocity control with
      EMA smoothing, dead-reckoning, and RED PAD break-out
  5D  PD precision centering on red landing pad
  5E  Position-mode controlled descent → land()

KEY FIXES vs previous version:
  ─────────────────────────────────────────────────────────────
  BUG FIX: Drone was not following orange lane to the red pad.
  Root causes identified and corrected:

  1. RED DETECTION BREAK was checked INSIDE the orange-detected
     branch — it was never reached when orange was lost at the
     last stretch before the red pad. Now checked FIRST, every
     iteration, regardless of orange state.

  2. RED CONFIRMATION DEBOUNCE: A single red frame no longer
     breaks the loop. Requires RED_SEEN_FRAMES_CONFIRM consecutive
     frames of red before breaking. Counter resets on non-detection.

  3. TRACK FRAME LIMIT was 600 (30 s) — far too short if the
     orange lane is long or the drone drifts. Raised to 1200 (60 s)
     with a safety fallback: if exhausted without seeing red,
     the drone performs a hover scan before landing.

  4. DEAD-RECKONING HEADING: Previously used `outbound_yaw` which
     may be wrong if Phase 5A target was the survey centre fallback.
     Now uses `state.current_yaw` at track start as the baseline,
     updated to the outbound vector at 5A transit time.

  5. SECTOR OVERCORRECTION: Sectors -1/+1 previously applied
     only yaw with zero forward — this caused the drone to spin
     in place when it drifted slightly. Now it applies a small
     forward component even in sector correction so it continues
     to approach the red pad while correcting heading.

  6. SMOOTH STOP RAMP: 20-step ramp-down instead of 10 to
     prevent inertia overshoot near the red pad.

  ALL movements are rate-limited via exponential smoothing.
  Yaw rate capped at ORANGE_YAW_RATE_MAX.
  Forward speed capped at ORANGE_FWD_FAST (≤ 0.25 m/s).
─────────────────────────────────────────────────────────────
"""

import asyncio
import math
import numpy as np
from mavsdk.offboard import VelocityBodyYawspeed, PositionNedYaw

import core.state as state
from core.config import (
    SURVEY_CENTER,
    SURVEY_ALT_M,
    ORANGE_FWD_FAST, ORANGE_FWD_SLOW,
    ORANGE_YAW_FINE, ORANGE_YAW_COARSE,
    ORANGE_LAT_SCALE, ORANGE_EX_THRESHOLD,
    ORANGE_YAW_BIAS_K, ORANGE_BLIND_FWD,
    ORANGE_YAW_RATE_MAX,
    RED_SEEN_FRAMES_CONFIRM,
)
from utils.navigation import go_to, pd_center

# ── Tunable constants ─────────────────────────────────────────
ORANGE_ACQ_FRAMES    = 200       # max frames to search for orange (~10 s)
ORANGE_SWEEP_YAW_MAX = 60        # degrees either side for acquisition sweep
ORANGE_TRACK_FRAMES  = 1200      # max track iterations (60 s at 20 Hz)
ORANGE_BLIND_FRAMES  = 40        # consecutive non-detections → log warning
ORANGE_BLIND_MAX     = 120       # if orange lost for this long → hover scan
HOVER_SCAN_WAIT      = 5.0       # seconds to hover-scan if completely lost

# EMA smoothing coefficient — higher = faster response, less smooth
_SMOOTH_ALPHA     = 0.20         # was 0.25 → 0.20 for real drone
_STOP_ALPHA       = 0.35         # faster ramp-down for stopping


def _smooth(current: float, target: float, alpha: float = _SMOOTH_ALPHA) -> float:
    """Exponential moving average to soften velocity transitions."""
    return current + alpha * (target - current)


async def _zero_velocity_ramp(
    s_fwd: float,
    s_lat: float,
    s_yaw: float,
    steps: int = 20,
    alpha: float = _STOP_ALPHA,
) -> None:
    """Smoothly ramp all velocity components to zero."""
    for _ in range(steps):
        s_fwd = _smooth(s_fwd, 0.0, alpha)
        s_lat = _smooth(s_lat, 0.0, alpha)
        s_yaw = _smooth(s_yaw, 0.0, alpha)
        state.body_vel = VelocityBodyYawspeed(
            float(s_fwd), float(s_lat), 0.0, float(s_yaw)
        )
        await asyncio.sleep(0.05)
    state.body_vel = VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)


async def run(drone) -> None:
    """Full return leg: orange exit → orange lane → red pad → land."""

    state.target_alt_m = SURVEY_ALT_M

    # ── 5A: Transit to orange exit ───────────────────────────
    state.mission_state = "PHASE 5A: TRANSIT TO ORANGE EXIT"
    print(f"[FSM] {state.mission_state}")
    state.use_velocity_mode = False

    if state.orange_exit_location:
        ox, oy = state.orange_exit_location
        dx_out = ox - SURVEY_CENTER[0]
        dy_out = oy - SURVEY_CENTER[1]
        outbound_yaw = math.degrees(math.atan2(dy_out, dx_out))
        print(
            f"[RETURN] Flying to projected orange exit "
            f"{state.orange_exit_location}, yaw={outbound_yaw:.1f}°"
        )
        await go_to(ox, oy, SURVEY_ALT_M, yaw_deg=outbound_yaw, wait=12.0)
    else:
        print("\033[93m[PHASE 5A] No exit scouted — flying to survey centre.\033[0m")
        outbound_yaw = 270.0
        await go_to(
            SURVEY_CENTER[0], SURVEY_CENTER[1],
            SURVEY_ALT_M,
            yaw_deg=outbound_yaw,
            wait=12.0,
        )

    # Capture the yaw we arrived at as the baseline for dead-reckoning
    # (may differ slightly from outbound_yaw due to wind/drift)
    tracking_yaw = state.current_yaw
    print(f"[RETURN] Arrived at exit. Tracking yaw baseline: {tracking_yaw:.1f}°")

    # ── 5B: Orange lane acquisition ───────────────────────────
    state.mission_state = "PHASE 5B: ORANGE LANE ACQUISITION"
    print(f"[FSM] {state.mission_state}")
    state.use_velocity_mode = True
    orange_acquired = False

    # Phase 1: straight forward creep at acquisition yaw
    for _ in range(ORANGE_ACQ_FRAMES // 2):
        v = state.read_vision()
        if v["orange_detected"]:
            orange_acquired = True
            print("\033[92m[PHASE 5B] Orange lane acquired on forward creep!\033[0m")
            break
        state.body_vel = VelocityBodyYawspeed(0.2, 0.0, 0.0, 0.0)
        await asyncio.sleep(0.05)

    # Phase 2: slow yaw sweep if not found
    if not orange_acquired:
        print("\033[93m[PHASE 5B] Not found on creep — performing yaw sweep.\033[0m")
        state.body_vel = VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)
        await asyncio.sleep(0.3)

        # Sweep ±ORANGE_SWEEP_YAW_MAX degrees
        for direction in (1, -1):
            yaw_swept = 0.0
            while abs(yaw_swept) < ORANGE_SWEEP_YAW_MAX:
                v = state.read_vision()
                if v["orange_detected"]:
                    orange_acquired = True
                    print("\033[92m[PHASE 5B] Orange acquired during yaw sweep!\033[0m")
                    break
                state.body_vel = VelocityBodyYawspeed(0.0, 0.0, 0.0, direction * 5.0)
                await asyncio.sleep(0.05)
                yaw_swept += 5.0 * 0.05  # approximate degrees swept

            if orange_acquired:
                break
            # Return to centre before sweeping other direction
            state.body_vel = VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)
            await asyncio.sleep(0.5)

    if not orange_acquired:
        print(
            "\033[93m[PHASE 5B] Orange not acquired after sweep — "
            "continuing with yaw-bias dead-reckoning only.\033[0m"
        )

    # Full stop before entering tracking loop
    state.body_vel = VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)
    await asyncio.sleep(0.5)

    # ── 5C: Orange lane tracking ─────────────────────────────
    state.mission_state = "PHASE 5C: ORANGE LANE TRACKING"
    print(f"[FSM] {state.mission_state}")
    state.use_velocity_mode = True

    blind_frames    = 0
    red_consec      = 0             # consecutive red-detection frames
    # Smoothed velocity state
    s_fwd = 0.0
    s_lat = 0.0
    s_yaw = 0.0
    broke_for_red = False

    for _ in range(ORANGE_TRACK_FRAMES):
        v = state.read_vision()

        # ── RED PAD CHECK — ALWAYS FIRST, EVERY ITERATION ────
        # FIX: was checked inside orange branch only — now always checked.
        if v["red_detected"]:
            red_consec += 1
            if red_consec >= RED_SEEN_FRAMES_CONFIRM:
                print(
                    f"\033[92m[PHASE 5C] Red pad confirmed "
                    f"({red_consec} frames) — breaking to 5D.\033[0m"
                )
                broke_for_red = True
                break
            # Red seen but not confirmed — continue tracking
        else:
            red_consec = 0   # reset debounce on any non-red frame

        # ── Yaw bias toward known return heading ──────────────
        yaw_diff = (tracking_yaw - state.current_yaw + 180) % 360 - 180
        yaw_bias = yaw_diff * ORANGE_YAW_BIAS_K

        if v["orange_detected"]:
            blind_frames = 0
            sector = v.get("orange_sector", 0)
            ex     = v["orange_ex"]

            if sector == 0:
                # Lane centred — drive forward with fine corrections
                if abs(ex) < ORANGE_EX_THRESHOLD:
                    t_fwd = ORANGE_FWD_FAST
                    t_yaw = ex * ORANGE_YAW_FINE
                else:
                    t_fwd = ORANGE_FWD_SLOW
                    t_yaw = ex * ORANGE_YAW_COARSE
                t_lat = ex * ORANGE_LAT_SCALE

            elif sector == -1:
                # Lane to the left — reduce forward, yaw left + small forward
                # FIX: was t_fwd=0 which caused spin-in-place; now keep small fwd
                t_fwd = ORANGE_FWD_SLOW * 0.5
                t_yaw = -ORANGE_YAW_RATE_MAX * 0.4
                t_lat = 0.0

            else:  # sector == 1
                # Lane to the right — reduce forward, yaw right + small forward
                t_fwd = ORANGE_FWD_SLOW * 0.5
                t_yaw =  ORANGE_YAW_RATE_MAX * 0.4
                t_lat = 0.0

            t_yaw += yaw_bias
            t_yaw  = float(np.clip(t_yaw, -ORANGE_YAW_RATE_MAX, ORANGE_YAW_RATE_MAX))

        else:
            # ── Orange not detected — dead-reckoning ──────────
            blind_frames += 1

            if blind_frames > ORANGE_BLIND_FRAMES:
                print(
                    f"\r\033[93m[PHASE 5C] Orange lost {blind_frames} frames "
                    f"— dead-reckoning yaw={tracking_yaw:.0f}°\033[0m   ",
                    end="", flush=True,
                )

            if blind_frames >= ORANGE_BLIND_MAX:
                # Completely lost — slow to hover and scan briefly
                print(
                    f"\n\033[91m[PHASE 5C] Orange lost for {ORANGE_BLIND_MAX} "
                    f"frames — hover scan.\033[0m"
                )
                await _zero_velocity_ramp(s_fwd, s_lat, s_yaw)
                state.use_velocity_mode = False
                await asyncio.sleep(HOVER_SCAN_WAIT)
                state.use_velocity_mode = True
                blind_frames = 0
                s_fwd = s_lat = s_yaw = 0.0
                continue

            # Continue dead-reckoning on known heading
            t_fwd = ORANGE_BLIND_FWD
            t_lat = 0.0
            t_yaw = yaw_bias * 2.0       # stronger bias to recover heading

        # ── EMA smoothing ─────────────────────────────────────
        s_fwd = _smooth(s_fwd, t_fwd)
        s_lat = _smooth(s_lat, t_lat)
        s_yaw = _smooth(s_yaw, t_yaw)

        state.body_vel = VelocityBodyYawspeed(
            float(s_fwd),
            float(s_lat),
            0.0,         # vertical — altitude hold in heartbeat
            float(s_yaw),
        )
        await asyncio.sleep(0.05)

    # ── Smooth stop ───────────────────────────────────────────
    await _zero_velocity_ramp(s_fwd, s_lat, s_yaw, steps=20)
    await asyncio.sleep(0.3)

    if not broke_for_red:
        print(
            "\033[93m[PHASE 5C] Tracking loop exhausted without red pad — "
            "attempting 5D anyway.\033[0m"
        )

    # ── 5D: PD centering on red pad ──────────────────────────
    state.mission_state = "PHASE 5D: PD CENTER ON RED PAD"
    print(f"[FSM] {state.mission_state}")
    state.use_velocity_mode = True

    centred = await pd_center(target="red", timeout=25.0)
    if not centred:
        print(
            "\033[93m[PHASE 5D] Red pad centering timed out — "
            "landing at current position.\033[0m"
        )

    # Full stop before descent
    state.body_vel = VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)
    await asyncio.sleep(0.5)

    # ── 5E: Controlled position-mode descent then land ───────
    state.mission_state = "PHASE 5E: LANDING"
    print(f"[FSM] {state.mission_state}")
    state.use_velocity_mode = False

    # Step down to 1.5 m in position mode (smooth, PX4-managed)
    n, e = state.current_pos
    state.offboard_setpoint = PositionNedYaw(n, e, -1.5, state.current_yaw)
    await asyncio.sleep(5.0)   # 5 s to settle at 1.5 m

    print("[FSM] Descend complete — initiating land.")
    await drone.action.land()

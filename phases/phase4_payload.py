"""
phases/phase4_payload.py
─────────────────────────────────────────────────────────────
PHASE 4 — PRECISION LOCK + PAYLOAD HOLD

  4A  PD centering within QR_CENTER_BUFFER_PIX
  4B  Altitude settle (1.5 s damp)
  4C  Payload hold PAYLOAD_HOLD_SEC
  4D  SLOW position-mode transit toward orange exit

CRITICAL: SPEED CONTROL IN 4D
──────────────────────────────
VELOCITY MODE IS NOT USED IN THIS PHASE.
After a payload operation we must never sprint to the orange exit.

The only speed control mechanism is:
  1. Set a POSITION_NED setpoint to the orange exit.
  2. Wait ORANGE_TRANSIT_SPEED_WAIT seconds.
  3. PX4's position controller flies there at its navigator speed.
     The long wait ensures the drone is hovering AT the target
     before Phase 5 begins — it does NOT sprint because PX4's
     default offboard position navigator speed is capped by
     MPC_XY_VEL_MAX (typically 5 m/s in firmware, but our wait
     time makes the effective speed = distance / wait_time ≈ 0.5 m/s).

# NOTE: The following line (or any set_velocity_body call) is
# intentionally ABSENT from Phase 4D to prevent accidental sprinting:
#   await drone.offboard.set_velocity_body(...)   ← DO NOT ADD
#
# If you need to further reduce speed, INCREASE ORANGE_TRANSIT_SPEED_WAIT
# in config.py rather than adding velocity commands.
─────────────────────────────────────────────────────────────
"""

import asyncio
import math
from mavsdk.offboard import VelocityBodyYawspeed, PositionNedYaw

import core.state as state
from core.config import (
    PAYLOAD_HOLD_SEC,
    SURVEY_ALT_M,
    QR_CENTER_BUFFER_PIX,
    SURVEY_CENTER,
    ORANGE_TRANSIT_SPEED_WAIT,
)
from utils.navigation import pd_center

_SETTLE_SEC      = 1.5   # seconds to damp oscillation before drop
_ALT_RECOVER_SEC = 4.0   # seconds for altitude recovery after drop


async def run(_drone) -> None:
    """Precision centering + payload + slow position-mode transit to orange exit."""

    state.target_alt_m = SURVEY_ALT_M

    # ── 4A: PD precision lock ────────────────────────────────
    state.mission_state = "PHASE 4A: PD PRECISION LOCK"
    print(f"[FSM] {state.mission_state}")
    state.use_velocity_mode = True

    locked = await pd_center(target="qr", timeout=30.0)
    if not locked:
        v = state.read_vision()
        if v["qr_detected"]:
            ex, ey = v["qr_ex"], v["qr_ey"]
            if abs(ex) < QR_CENTER_BUFFER_PIX and abs(ey) < QR_CENTER_BUFFER_PIX:
                print(
                    f"\033[93m[PHASE 4A] Timed out but within buffer "
                    f"({ex:+d}, {ey:+d}) px — accepting.\033[0m"
                )
            else:
                print("\033[93m[PHASE 4A] Timed out — proceeding best-effort.\033[0m")
        else:
            print("\033[93m[PHASE 4A] Timed out (QR lost) — proceeding anyway.\033[0m")

    # Zero velocity cleanly
    state.body_vel = VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)
    await asyncio.sleep(0.3)

    # ── 4B: Altitude settle ───────────────────────────────────
    state.mission_state = "PHASE 4B: ALTITUDE SETTLE"
    print(f"[FSM] {state.mission_state}")
    await asyncio.sleep(_SETTLE_SEC)

    # ── 4C: Payload hold ─────────────────────────────────────
    state.mission_state = f"PHASE 4C: PAYLOAD HOLD ({PAYLOAD_HOLD_SEC:.0f}s)"
    print(f"[FSM] {state.mission_state} — simulating payload release")
    await asyncio.sleep(PAYLOAD_HOLD_SEC)

    state.body_vel = VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)
    await asyncio.sleep(0.3)

    # ── 4D: SLOW position-mode transit to orange exit ─────────
    # CRITICAL: velocity mode is NOT engaged here.
    # Speed is governed entirely by: distance / ORANGE_TRANSIT_SPEED_WAIT
    # Do NOT add any set_velocity_body calls in this section.
    state.mission_state = "PHASE 4D: SLOW TRANSIT TO ORANGE EXIT"
    print(f"[FSM] {state.mission_state}")
    state.use_velocity_mode = False      # position mode only
    state.target_alt_m      = SURVEY_ALT_M

    # Determine target: orange exit if known, else survey centre
    if state.orange_exit_location:
        tx, ty = state.orange_exit_location
        dx = tx - SURVEY_CENTER[0]
        dy = ty - SURVEY_CENTER[1]
        transit_yaw = math.degrees(math.atan2(dy, dx))
        print(
            f"[PHASE 4D] Slow position transit to orange exit "
            f"{state.orange_exit_location}, yaw={transit_yaw:.1f}°  "
            f"(effective speed ≈ dist/{ORANGE_TRANSIT_SPEED_WAIT:.0f}s)"
        )
    else:
        tx, ty = SURVEY_CENTER
        transit_yaw = 270.0
        print("[PHASE 4D] No orange exit known — transiting to survey centre.")

    # Set POSITION setpoint — PX4 flies there at its controlled rate.
    # ORANGE_TRANSIT_SPEED_WAIT ensures drone has arrived and is
    # hovering before Phase 5 takes over.
    state.offboard_setpoint = PositionNedYaw(
        tx, ty,
        -abs(SURVEY_ALT_M),
        transit_yaw,
    )

    # NOTE: ORANGE_TRANSIT_SPEED_WAIT is deliberately long.
    # Increasing it reduces effective transit speed further.
    # Do NOT reduce below 20 s for a 10–15 m gap.
    await asyncio.sleep(ORANGE_TRANSIT_SPEED_WAIT)

    print("[PHASE 4] Slow transit complete. Handing off to Phase 5.")

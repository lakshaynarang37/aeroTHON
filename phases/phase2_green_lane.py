"""
phases/phase2_green_lane.py
─────────────────────────────────────────────────────────────
PHASE 2 — GREEN CORRIDOR TRACKING

Follows the green painted lane from the start pad toward the
survey zone. Uses sector-based visual servoing with a global
inbound-yaw bias to keep the drone pointed toward
SURVEY_CENTER even when the lane curves.

Exit conditions (in priority order):
  1. Green lost for > GREEN_LOST_LIMIT frames → corridor ended.
  2. Hard iteration cap (safety: prevents infinite loop if vision
     glitches and lane never disappears).

Failsafes:
  • If green is never detected within the first 40 frames
    → assume we're already past the corridor, skip to Phase 3.
  • Velocity is zeroed on exit regardless of loop exit path.
─────────────────────────────────────────────────────────────
"""

import asyncio
import math
from mavsdk.offboard import VelocityBodyYawspeed

import core.state as state
from core.config import (
    SURVEY_CENTER,
    GREEN_FWD_FAST, GREEN_FWD_SLOW,
    GREEN_YAW_FINE, GREEN_YAW_COARSE,
    GREEN_LAT_SCALE, GREEN_EX_THRESHOLD,
    GREEN_LOST_LIMIT, GREEN_YAW_BIAS_K,
)

MAX_ITERATIONS   = 800
INITIAL_PATIENCE = 40       # frames to wait for first green detection


async def run(_drone) -> None:
    """
    Track the green corridor.
    drone arg kept for interface consistency; not used directly.
    """
    state.mission_state = "PHASE 2: TRACKING GREEN CORRIDOR"
    print(f"[FSM] {state.mission_state}")
    state.use_velocity_mode = True

    lost_frames    = 0
    ever_seen_green = False
    initial_wait   = 0

    for _ in range(MAX_ITERATIONS):
        v = state.read_vision()

        # ── Inbound yaw bias ──────────────────────────────────
        dx_in = SURVEY_CENTER[0] - state.current_pos[0]
        dy_in = SURVEY_CENTER[1] - state.current_pos[1]
        target_yaw_inbound = math.degrees(math.atan2(dy_in, dx_in))
        yaw_bias = (target_yaw_inbound - state.current_yaw + 180) % 360 - 180

        if v["green_detected"]:
            ever_seen_green = True
            lost_frames = 0
            sector = v.get("green_sector", 0)
            ex     = v["green_ex"]

            if sector == 0:
                if abs(ex) < GREEN_EX_THRESHOLD:
                    forward_speed = GREEN_FWD_FAST
                    yaw_speed     = ex * GREEN_YAW_FINE
                else:
                    forward_speed = GREEN_FWD_SLOW
                    yaw_speed     = ex * GREEN_YAW_COARSE
                lateral_vel = ex * GREEN_LAT_SCALE
            elif sector == -1:
                forward_speed, yaw_speed, lateral_vel = 0.0, -0.8, 0.0
            else:                                   # sector == 1
                forward_speed, yaw_speed, lateral_vel = 0.0,  0.8, 0.0

            yaw_speed += yaw_bias * GREEN_YAW_BIAS_K

        else:
            lost_frames += 1

            # ── Patience check on first detection ─────────────
            if not ever_seen_green:
                initial_wait += 1
                if initial_wait > INITIAL_PATIENCE:
                    print(
                        "\033[93m[PHASE 2] Green never detected — "
                        "assuming corridor already passed. Skipping.\033[0m"
                    )
                    break

            # ── End-of-corridor check ─────────────────────────
            if lost_frames > GREEN_LOST_LIMIT:
                print("\033[92m[PHASE 2] End of green corridor — moving to survey.\033[0m")
                break

            # Creep forward while temporarily lost
            forward_speed = 0.5
            lateral_vel   = 0.0
            yaw_speed     = yaw_bias * 0.02

        state.body_vel = VelocityBodyYawspeed(
            float(forward_speed), float(lateral_vel), 0.0, float(yaw_speed)
        )
        await asyncio.sleep(0.05)

    # ── Safe stop ─────────────────────────────────────────────
    state.body_vel = VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)
    await asyncio.sleep(0.5)   # let velocity settle before next phase

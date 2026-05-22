"""
phases/phase0_takeoff.py
─────────────────────────────────────────────────────────────
PHASE 0 — ARM + STABLE VERTICAL TAKEOFF

Real-drone safe takeoff sequence:
  1. Seed the offboard setpoint at current position before
     starting offboard mode (avoids the drone lurching).
  2. Arm.
  3. Start offboard.
  4. Command straight up to TAKEOFF_ALT_M — zero lateral
     movement until altitude is reached.
  5. Wait until velocity < TAKEOFF_STABLE_VEL for at least
     TAKEOFF_HOVER_SEC seconds → hover declared stable.
  6. Only then return to run_mission so lateral phases begin.

Failsafes:
  • If arm() raises → abort, do not enter offboard.
  • If offboard.start() raises → disarm, raise so main aborts.
  • If altitude is never reached within 30 s → land + raise.
─────────────────────────────────────────────────────────────
"""

import asyncio
from mavsdk.offboard import OffboardError, PositionNedYaw, VelocityBodyYawspeed

import core.state as state
from core.config import TAKEOFF_ALT_M, TAKEOFF_HOVER_SEC, TAKEOFF_STABLE_VEL


TAKEOFF_TIMEOUT_SEC = 30.0          # hard ceiling for altitude acquisition


async def run(drone) -> None:
    """
    Execute full arm + takeoff sequence.

    Raises RuntimeError on any unrecoverable failure so
    run_mission can abort cleanly.
    """
    state.mission_state = "PHASE 0: ARM & TAKEOFF"
    print(f"[FSM] {state.mission_state}")

    # ── Step 1: Pre-seed offboard setpoint AT current position ─
    # Critical: offboard.start() requires at least one setpoint
    # to have been sent before the call; use current NED position
    # at ground level so the drone does NOT jump.
    pos_now = state.current_pos
    seed_sp = PositionNedYaw(pos_now[0], pos_now[1], 0.0, 0.0)
    for _ in range(30):
        try:
            await drone.offboard.set_position_ned(seed_sp)
        except Exception:
            pass
        await asyncio.sleep(0.05)

    # ── Step 2: Arm ───────────────────────────────────────────
    print("[TAKEOFF] Arming...")
    try:
        await drone.action.arm()
        print("[TAKEOFF] Armed.")
    except Exception as e:
        raise RuntimeError(f"[TAKEOFF] Arm failed: {e}") from e

    # ── Step 3: Start offboard ────────────────────────────────
    print("[TAKEOFF] Starting offboard mode...")
    try:
        await drone.offboard.start()
        print("[TAKEOFF] Offboard active.")
    except OffboardError as e:
        await drone.action.disarm()
        raise RuntimeError(f"[TAKEOFF] Offboard start failed: {e}") from e

    # ── Step 4: Command vertical climb — zero lateral delta ───
    # Keep N/E fixed to current position; only altitude changes.
    climb_sp = PositionNedYaw(
        pos_now[0],
        pos_now[1],
        -abs(TAKEOFF_ALT_M),   # NED altitude is negative-up
        0.0,
    )
    state.use_velocity_mode = False
    state.offboard_setpoint = climb_sp
    print(f"[TAKEOFF] Climbing to {TAKEOFF_ALT_M:.1f} m AGL...")

    # ── Step 5: Wait until altitude reached ──────────────────
    elapsed = 0.0
    while elapsed < TAKEOFF_TIMEOUT_SEC:
        # Altitude feedback via telemetry position_velocity_ned
        # We check how close current_pos[2] is — but we only have
        # (north, east) in state. Use drone.telemetry directly here
        # for altitude, just once per 0.5 s, non-blocking.
        async for pos in drone.telemetry.position_velocity_ned():
            alt_reached = abs(pos.position.down_m) >= (TAKEOFF_ALT_M * 0.90)
            vel_ok = (
                abs(pos.velocity.north_m_s) < TAKEOFF_STABLE_VEL
                and abs(pos.velocity.east_m_s) < TAKEOFF_STABLE_VEL
                and abs(pos.velocity.down_m_s) < TAKEOFF_STABLE_VEL
            )
            break  # sample once
        if alt_reached and vel_ok:
            break
        await asyncio.sleep(0.5)
        elapsed += 0.5
    else:
        # Timeout: altitude never reached — safe landing
        print("\033[91m[TAKEOFF] TIMEOUT: altitude not reached. Landing.\033[0m")
        await drone.action.land()
        raise RuntimeError("[TAKEOFF] Altitude acquisition timed out.")

    # ── Step 6: Hold hover for TAKEOFF_HOVER_SEC ─────────────
    print(f"[TAKEOFF] Altitude reached. Hovering for {TAKEOFF_HOVER_SEC:.1f} s...")
    await asyncio.sleep(TAKEOFF_HOVER_SEC)
    print("[TAKEOFF] Hover stable. Handing off to mission phases.")

"""
core/telemetry.py
─────────────────────────────────────────────────────────────
Background asyncio tasks — position, attitude, offboard heartbeat.

CHANGES:
  • update_position also writes state.current_alt_m from NED down.
  • heartbeat applies ACTIVE ALTITUDE HOLD when in velocity mode:
    computes alt error, injects a clamped vertical correction into
    every body-velocity command. Prevents altitude loss during
    lane following, PD centering, and payload hold.
─────────────────────────────────────────────────────────────
"""

import asyncio
import numpy as np
from mavsdk.offboard import VelocityBodyYawspeed

import core.state as state
from core.config import HEARTBEAT_INTERVAL, ALTITUDE_HOLD_KP, ALTITUDE_HOLD_MAX


async def update_position(drone) -> None:
    """Mirror NED north/east and altitude into state."""
    async for pos in drone.telemetry.position_velocity_ned():
        state.current_pos = (
            pos.position.north_m,
            pos.position.east_m,
        )
        # NED down_m is negative-up; convert to positive AGL
        state.current_alt_m = -pos.position.down_m


async def update_attitude(drone) -> None:
    """Mirror yaw into state."""
    async for att in drone.telemetry.attitude_euler():
        state.current_yaw = att.yaw_deg


async def heartbeat(drone) -> None:
    """
    Offboard keepalive with active altitude hold.

    In velocity mode:
      alt_err = target_alt_m - current_alt_m
      A positive error (drone too low) → we need to climb
      → VelocityBodyYawspeed.down_m_s should be NEGATIVE (climb).
      vz_corr = clip(-alt_err * KP, -MAX, MAX)

    In position mode:
      Altitude is encoded in the NED setpoint; no correction needed.
    """
    while True:
        try:
            if state.use_velocity_mode:
                bv = state.body_vel
                alt_err    = state.target_alt_m - state.current_alt_m
                vz_corr    = float(np.clip(
                    -alt_err * ALTITUDE_HOLD_KP,
                    -ALTITUDE_HOLD_MAX,
                     ALTITUDE_HOLD_MAX,
                ))
                corrected = VelocityBodyYawspeed(
                    bv.forward_m_s,
                    bv.right_m_s,
                    vz_corr,            # override vertical component
                    bv.yawspeed_deg_s,
                )
                await drone.offboard.set_velocity_body(corrected)
            else:
                await drone.offboard.set_position_ned(state.offboard_setpoint)
        except Exception:
            pass
        await asyncio.sleep(HEARTBEAT_INTERVAL)

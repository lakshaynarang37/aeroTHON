"""
utils/navigation.py
─────────────────────────────────────────────────────────────
Low-level navigation primitives shared by every phase.

CHANGES:
  • go_to sets state.target_alt_m so altitude hold is correct.
  • execute_smooth_scan records orange via ALL THREE projection
    methods (A: trig, B: lateral bearing, C: weighted centroid)
    and calls projection.fuse_orange_location() for best estimate.
  • pd_center vertical always 0.0 — heartbeat handles altitude.
─────────────────────────────────────────────────────────────
"""

import asyncio
import math
import time
import numpy as np
from shapely.geometry import Polygon, LineString
from mavsdk.offboard import PositionNedYaw, VelocityBodyYawspeed

import core.state as state
from core.config import PD_KP, PD_KD, PD_MAX_VEL_MPS, PD_STABLE_PIX, PD_STABLE_FRAMES
from utils.projection import (
    ground_point_from_vision,
    lateral_ground_point,
    accumulate_weighted_centroid,
    fuse_orange_location,
)


# ─────────────────────────────────────────────────────────────
# PATH PLANNING
# ─────────────────────────────────────────────────────────────

def generate_lawnmower(
    boundary_corners: list[tuple[float, float]],
    obstacles_list: list | None = None,
    step: float = 6.0,
    start_right: bool = True,
) -> list[tuple[float, float]]:
    """Boustrophedon path inside boundary minus obstacle polygons."""
    if obstacles_list is None:
        obstacles_list = []

    navigable = Polygon(boundary_corners)
    for obs in obstacles_list:
        navigable = navigable.difference(Polygon(obs))

    minx, miny, maxx, maxy = navigable.bounds
    path: list[tuple[float, float]] = []
    flip = not start_right
    curr_y = miny + step / 2.0

    while curr_y <= maxy:
        scan_line    = LineString([(minx - 10, curr_y), (maxx + 10, curr_y)])
        intersection = navigable.intersection(scan_line)
        if intersection.is_empty:
            curr_y += step
            continue

        segments = (
            [intersection]
            if intersection.geom_type == "LineString"
            else list(intersection.geoms)
        )
        segments.sort(key=lambda l: list(l.coords)[0][0], reverse=flip)

        for seg in segments:
            coords  = list(seg.coords)
            p_start = coords[0]  if not flip else coords[-1]
            p_end   = coords[-1] if not flip else coords[0]
            path.append((p_start[0], p_start[1]))
            path.append((p_end[0],   p_end[1]))

        curr_y += step
        flip = not flip

    return path


# ─────────────────────────────────────────────────────────────
# WAYPOINT MOVE
# ─────────────────────────────────────────────────────────────

async def go_to(
    x: float,
    y: float,
    alt: float,
    yaw_deg: float,
    wait: float = 8.0,
) -> None:
    """
    Command a NED waypoint and wait `wait` seconds.
    Updates state.target_alt_m so altitude hold targets correctly.
    """
    state.use_velocity_mode = False
    state.target_alt_m      = float(alt)
    state.offboard_setpoint = PositionNedYaw(x, y, -abs(alt), yaw_deg)
    await asyncio.sleep(wait)


# ─────────────────────────────────────────────────────────────
# PD VISUAL CENTERING
# ─────────────────────────────────────────────────────────────

async def pd_center(
    target: str = "qr",
    timeout: float = 30.0,
) -> bool:
    """
    PD centering on visual target centroid.
    Vertical always 0.0 — altitude hold in heartbeat handles it.

    Returns True → stable within timeout, False → timed out / lost.
    """
    state.use_velocity_mode = True

    last_ex = last_ey = 0
    stable  = 0
    elapsed = 0.0
    first_seen = True
    last_time  = time.time()

    while stable < PD_STABLE_FRAMES and elapsed < timeout:
        v           = state.read_vision()
        is_detected = v.get(f"{target}_detected", False)

        if is_detected:
            ex = v[f"{target}_ex"]
            ey = v.get(f"{target}_ey", 0)

            curr_time = time.time()
            dt = max(curr_time - last_time, 0.01)
            last_time = curr_time

            if first_seen:
                last_ex, last_ey = ex, ey
                first_seen = False

            dx_dt = (ex - last_ex) / dt
            dy_dt = (ey - last_ey) / dt
            last_ex, last_ey = ex, ey

            vx = -1.0 * (ey * PD_KP + dy_dt * PD_KD)
            vy =  1.0 * (ex * PD_KP + dx_dt * PD_KD)

            state.body_vel = VelocityBodyYawspeed(
                float(np.clip(vx, -PD_MAX_VEL_MPS, PD_MAX_VEL_MPS)),
                float(np.clip(vy, -PD_MAX_VEL_MPS, PD_MAX_VEL_MPS)),
                0.0,    # altitude hold handles vertical
                0.0,
            )
            stable = stable + 1 if (abs(ex) < PD_STABLE_PIX and
                                     abs(ey) < PD_STABLE_PIX) else 0
        else:
            state.body_vel = VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)
            stable     = 0
            first_seen = True
            last_time  = time.time()

        await asyncio.sleep(0.05)
        elapsed += 0.05

    state.body_vel = VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)
    success = stable >= PD_STABLE_FRAMES
    print(f"\033[92m[NAV] PD Centering {'Complete' if success else 'TIMED OUT'} ({target.upper()})\033[0m")
    return success


# ─────────────────────────────────────────────────────────────
# SMOOTH PATH EXECUTOR
# ─────────────────────────────────────────────────────────────

async def execute_smooth_scan(
    path: list[tuple[float, float]],
    alt: float,
    speed: float,
    yaw_deg: float,
) -> bool:
    """
    Interpolate along path at speed m/s, watching for QR.

    Orange exit logging uses TRIPLE-METHOD fusion:
      A: ground_point_from_vision (trig projection)
      B: lateral_ground_point (bearing offset)
      C: accumulate_weighted_centroid (reliability-weighted running mean)
    fuse_orange_location() blends all three medians/centroids.

    Returns True → target QR found, False → path exhausted.
    """
    state.use_velocity_mode = False
    state.target_alt_m      = float(alt)

    if not path:
        print("\033[91m[SCAN] Empty path — skipping.\033[0m")
        return False

    state.offboard_setpoint = PositionNedYaw(path[0][0], path[0][1], -abs(alt), yaw_deg)
    await asyncio.sleep(4.0)

    for i in range(1, len(path)):
        p1, p2 = path[i - 1], path[i]
        dist    = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
        if dist == 0:
            continue

        steps = max(1, int((dist / speed) / 0.05))

        for step in range(steps):
            v    = state.read_vision()
            frac = step / float(steps)
            cx   = p1[0] + (p2[0] - p1[0]) * frac
            cy_  = p1[1] + (p2[1] - p1[1]) * frac

            # ── Triple-method orange location update ──────────
            if v["orange_detected"]:
                proj_alt = max(state.current_alt_m, 0.5) or alt
                ex_px    = v["orange_ex"]

                # Method A: trig projection (full ex + ey)
                pt_a = ground_point_from_vision(
                    ex_px=ex_px, ey_px=0, alt_m=proj_alt
                )
                # Method B: lateral bearing offset
                pt_b = lateral_ground_point(
                    ex_px=ex_px, alt_m=proj_alt
                )
                # Method C: weighted centroid (favours low-alt, centred observations)
                pt_c = accumulate_weighted_centroid(pt_a, proj_alt, ex_px)

                state.orange_observations_A.append(pt_a)
                state.orange_observations_B.append(pt_b)

                best = fuse_orange_location()
                if best is not None:
                    state.orange_exit_location = best
                    print(
                        f"\r\033[93m[MEMORY] Orange Exit (A+B+C fused) @ {best}  "
                        f"(A:{len(state.orange_observations_A)} "
                        f"B:{len(state.orange_observations_B)} obs)\033[0m   ",
                        end="", flush=True,
                    )

            # ── QR logic ──────────────────────────────────────
            if state.target_id is None:
                if v["qr_detected"] and v["qr_id"]:
                    state.target_id = v["qr_id"]
                    print(f"\n\033[92m[MEMORY] Target locked: '{state.target_id}'\033[0m\n")
                    return True
            else:
                if v["qr_detected"] and v["qr_id"] == state.target_id:
                    print(f"\n\033[92m[MATCH] Target '{state.target_id}' found!\033[0m\n")
                    return True

            state.offboard_setpoint = PositionNedYaw(cx, cy_, -abs(alt), yaw_deg)
            await asyncio.sleep(0.05)

    return False

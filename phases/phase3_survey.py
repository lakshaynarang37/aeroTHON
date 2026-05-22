"""
phases/phase3_survey.py
─────────────────────────────────────────────────────────────
PHASE 3 — SURVEY ZONE: TARGET QR SEARCH

Sub-phases:
  3A  Perimeter scout — slow speed, triple-method orange logging,
      QR detection with frame-debounce failsafe
  3B  Internal lawnmower fallback

CHANGES:
  • Orange location uses TRIPLE-METHOD fusion:
      A: trig projection via ground_point_from_vision
      B: lateral bearing offset via lateral_ground_point
      C: weighted centroid via accumulate_weighted_centroid
    All accumulated; fuse_orange_location() blends the estimates.
  • QR detection on perimeter requires PHASE_3A_QR_CONFIRM_FRAMES
    consecutive matching frames before breaking — prevents false
    positives from partial reads or noisy pyzbar decodes while
    the drone is in motion.
  • reset_weighted_centroid() called at entry so Method C starts
    fresh for each survey (not polluted by Phase 1 observations).
  • Perimeter yaw is computed segment-by-segment (direction of
    travel) so vision always looks forward along the lane.
  • target_alt_m set at entry for altitude hold.
─────────────────────────────────────────────────────────────
"""

import asyncio
import math
from mavsdk.offboard import PositionNedYaw

import core.state as state
from core.config import (
    SURVEY_CORNERS, NO_FLY_ZONES,
    SURVEY_STEP_M, SURVEY_SPEED_MPS, SURVEY_ALT_M, SURVEY_YAW_DEG,
    PERIMETER_SPEED_MPS, PERIMETER_ALT_M,
    PHASE_3A_QR_CONFIRM_FRAMES,
)
from utils.navigation import generate_lawnmower, execute_smooth_scan
from utils.projection import (
    ground_point_from_vision,
    lateral_ground_point,
    accumulate_weighted_centroid,
    reset_weighted_centroid,
    fuse_orange_location,
)


async def run(drone) -> bool:
    """
    Execute survey zone search.
    Returns True → target found, False → exhausted search.
    """

    if not state.target_id:
        print("\033[91m[PHASE 3] No target ID set — aborting.\033[0m")
        return False    

    # Set altitude target for heartbeat hold
    state.target_alt_m = PERIMETER_ALT_M

    # Reset ALL orange observation accumulators for a clean run
    state.orange_exit_location  = None
    state.orange_observations_A = []
    state.orange_observations_B = []
    reset_weighted_centroid()       # Method C internal accumulator

    # ─────────────────────────────────────────────────────────
    # 3A: PERIMETER SCOUT
    # ─────────────────────────────────────────────────────────
    state.mission_state = "PHASE 3A: PERIMETER SCOUT"
    print(f"[FSM] {state.mission_state}")
    state.use_velocity_mode = False

    perimeter_path = SURVEY_CORNERS + [SURVEY_CORNERS[0]]
    found_qr       = False
    qr_confirm_cnt = 0               # consecutive matching-QR frames

    for i in range(1, len(perimeter_path)):
        p1 = perimeter_path[i - 1]
        p2 = perimeter_path[i]

        # Compute facing yaw for this segment (direction of travel)
        seg_yaw = math.degrees(math.atan2(p2[1] - p1[1], p2[0] - p1[0]))

        dist  = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
        steps = max(1, int((dist / PERIMETER_SPEED_MPS) / 0.05))

        for step in range(steps):
            v    = state.read_vision()
            frac = step / float(steps)

            # ── Triple-method orange projection ───────────────
            if v["orange_detected"]:
                proj_alt = max(state.current_alt_m, 0.5) or PERIMETER_ALT_M
                ex_px    = v["orange_ex"]

                # Method A: pinhole trig projection (ex + ey)
                pt_a = ground_point_from_vision(
                    ex_px=ex_px, ey_px=0, alt_m=proj_alt
                )
                # Method B: lateral bearing offset
                pt_b = lateral_ground_point(
                    ex_px=ex_px, alt_m=proj_alt
                )
                # Method C: weighted centroid
                accumulate_weighted_centroid(pt_a, proj_alt, ex_px)

                state.orange_observations_A.append(pt_a)
                state.orange_observations_B.append(pt_b)

                best = fuse_orange_location()
                if best is not None:
                    state.orange_exit_location = best
                    print(
                        f"\r\033[93m[MEMORY] Orange Exit (A+B+C) @ {best}  "
                        f"(A:{len(state.orange_observations_A)} "
                        f"B:{len(state.orange_observations_B)})\033[0m   ",
                        end="", flush=True,
                    )

            # ── QR detection with debounce failsafe ───────────
            # A single noisy decode while moving should NOT break
            # the perimeter. Require PHASE_3A_QR_CONFIRM_FRAMES
            # consecutive frames of the exact target ID.
            if v["qr_detected"] and v["qr_id"] == state.target_id:
                qr_confirm_cnt += 1
                if qr_confirm_cnt >= PHASE_3A_QR_CONFIRM_FRAMES:
                    print(
                        f"\n\033[92m[PHASE 3A] Target confirmed after "
                        f"{qr_confirm_cnt} frames on perimeter!\033[0m"
                    )
                    found_qr = True
                    break
            else:
                # Reset on any non-matching frame
                if qr_confirm_cnt > 0:
                    print(
                        f"\r\033[93m[PHASE 3A] QR confirm reset "
                        f"({qr_confirm_cnt} → 0)\033[0m   ",
                        end="", flush=True,
                    )
                qr_confirm_cnt = 0

            # ── Advance setpoint ──────────────────────────────
            state.offboard_setpoint = PositionNedYaw(
                p1[0] + (p2[0] - p1[0]) * frac,
                p1[1] + (p2[1] - p1[1]) * frac,
                -abs(PERIMETER_ALT_M),
                seg_yaw,             # face direction of travel
            )
            await asyncio.sleep(0.05)

        if found_qr:
            break

    if found_qr:
        return True

    # ─────────────────────────────────────────────────────────
    # 3B: INTERNAL LAWNMOWER (fallback)
    # ─────────────────────────────────────────────────────────
    state.mission_state = "PHASE 3B: INTERNAL LAWNMOWER"
    print(f"\n[FSM] {state.mission_state}")
    state.target_alt_m = SURVEY_ALT_M

    survey_path = generate_lawnmower(
        SURVEY_CORNERS, obstacles_list=NO_FLY_ZONES, step=SURVEY_STEP_M
    )
    found_qr = await execute_smooth_scan(
        survey_path,
        alt=SURVEY_ALT_M,
        speed=SURVEY_SPEED_MPS,
        yaw_deg=SURVEY_YAW_DEG,
    )

    if not found_qr:
        print("\033[91m[PHASE 3] Target not found after full sweep.\033[0m")

    return found_qr

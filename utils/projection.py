"""
utils/projection.py
─────────────────────────────────────────────────────────────
Camera → World projection math + TRIPLE-METHOD orange location fusion.

ORANGE EXIT ESTIMATION — THREE INDEPENDENT METHODS
────────────────────────────────────────────────────
The fundamental problem: when the drone sees orange at pixel
offset (ex_px) from centre, where exactly IS that orange on
the ground?

Three independent methods answer this, then we blend medians:

  METHOD A — Trigonometric projection (pinhole model)
  ────────────────────────────────────────────────────
  Uses camera FOV and altitude to compute the exact ground
  location via tan(angle) = offset / altitude.
  Handles both lateral (ex) and forward (ey) offsets.
  Strength:  geometrically exact given accurate FOV + altitude.
  Weakness:  sensitive to altitude error, camera tilt from wind.

  METHOD B — Lateral bearing offset
  ──────────────────────────────────
  Treats the drone's current forward bearing and computes a
  perpendicular (lateral) ground offset from pixel error only.
  offset_m = (ex_px / IMAGE_W) * (alt * tan(HFOV/2)) * 2
  The orange ground point = drone position + lateral offset in NED.
  Strength:  independent of absolute altitude; uses bearing.
  Weakness:  only good for lateral (not forward) displacement.

  METHOD C — Running weighted centroid with distance penalty
  ───────────────────────────────────────────────────────────
  Keeps an exponential-decay weighted average of ALL observed
  ground points, giving higher weight to observations made
  when the drone is at lower altitude (more accurate projection)
  and closer to the detected feature (more reliable signal).
  Weight = 1 / (alt_m^0.5 * (1 + |ex_px|/IMAGE_W))
  This provides stability: outlier flashes don't shift the
  estimate, and the running mean converges as observations pile up.
  Strength:  robust against single bad observations; self-correcting.
  Weakness:  slow to update if the field of view was wildly off initially.

  FUSION
  ──────
  1. Take median of Method A observations  → med_a
  2. Take median of Method B observations  → med_b
  3. Compute Method C weighted centroid    → wc
  4. final = ALPHA_A*med_a + ALPHA_B*med_b + ALPHA_C*wc
     (alphas sum to 1.0, defined in config)

  Because all three methods have UNCORRELATED error sources,
  blending them reduces combined variance far beyond any single
  method. Method C additionally suppresses outliers that survive
  per-method medians.
─────────────────────────────────────────────────────────────
"""

import math
import core.state as state
from core.config import (
    CAM_HFOV_DEG,
    CAM_IMAGE_W,
    CAM_IMAGE_H,
    SURVEY_ALT_M,
    ORANGE_LOC_BLEND_ALPHA,
    ORANGE_LOC_BLEND_ALPHA_B,
    ORANGE_LOC_BLEND_ALPHA_C,
)


# ─────────────────────────────────────────────────────────────
# METHOD A: TRIGONOMETRIC PIXEL→GROUND PROJECTION
# ─────────────────────────────────────────────────────────────

def pixel_to_ned_offset(
    ex_px:   float,
    ey_px:   float,
    alt_m:   float,
    yaw_deg: float,
) -> tuple[float, float]:
    """
    Convert pixel error (ex_px, ey_px) at given altitude+yaw
    into a NED ground offset (ΔN, ΔE) using pinhole projection.

    ex_px positive → target right of centre → East in body frame
    ey_px positive → target below centre    → South in body frame
    """
    if alt_m <= 0.1:
        return (0.0, 0.0)

    hfov_rad = math.radians(CAM_HFOV_DEG)
    vfov_rad = hfov_rad * (CAM_IMAGE_H / CAM_IMAGE_W)

    nx = ex_px / CAM_IMAGE_W        # normalised [-0.5, +0.5]
    ny = ey_px / CAM_IMAGE_H

    angle_x = nx * hfov_rad         # azimuth in body (East +)
    angle_y = ny * vfov_rad         # elevation in body (South +)

    body_fwd   = -alt_m * math.tan(angle_y)   # ey>0 → South → neg forward
    body_right =  alt_m * math.tan(angle_x)   # ex>0 → East  → pos right

    yaw_rad = math.radians(yaw_deg)
    cos_y, sin_y = math.cos(yaw_rad), math.sin(yaw_rad)

    delta_n = body_fwd * cos_y - body_right * sin_y
    delta_e = body_fwd * sin_y + body_right * cos_y

    return (delta_n, delta_e)


def ground_point_from_vision(
    ex_px:  float,
    ey_px:  float,
    alt_m:  float | None = None,
) -> tuple[float, float]:
    """
    Method A: full pipeline → absolute NED of visible ground feature.
    Uses both lateral (ex) and forward (ey) pixel offsets.
    """
    drone_n, drone_e = state.current_pos
    drone_yaw        = state.current_yaw
    altitude         = alt_m if alt_m is not None else SURVEY_ALT_M

    dn, de = pixel_to_ned_offset(ex_px, ey_px, altitude, drone_yaw)
    return (round(drone_n + dn, 2), round(drone_e + de, 2))


# ─────────────────────────────────────────────────────────────
# METHOD B: LATERAL BEARING OFFSET
# ─────────────────────────────────────────────────────────────

def lateral_ground_point(
    ex_px: float,
    alt_m: float | None = None,
) -> tuple[float, float]:
    """
    Method B: compute ground point using lateral offset only.

    The full image width subtends 2 * alt * tan(HFOV/2) metres
    at altitude alt_m. A pixel error of ex_px therefore maps to:
        lateral_m = (ex_px / IMAGE_W) * 2 * alt * tan(HFOV/2)

    This lateral offset is then applied perpendicular to the
    drone's current heading (i.e. rotated 90° from forward in
    the NED frame).
    """
    altitude  = alt_m if alt_m is not None else SURVEY_ALT_M
    if altitude <= 0.1:
        return state.current_pos

    hfov_rad = math.radians(CAM_HFOV_DEG)
    ground_width_m = 2.0 * altitude * math.tan(hfov_rad / 2.0)

    # Lateral offset in metres (+ve = right of drone = East in body)
    lateral_m = (ex_px / CAM_IMAGE_W) * ground_width_m

    # Rotate to NED using current yaw
    # body right in NED: delta_n = -lat*sin(yaw), delta_e = lat*cos(yaw)
    yaw_rad = math.radians(state.current_yaw)
    delta_n = -lateral_m * math.sin(yaw_rad)
    delta_e =  lateral_m * math.cos(yaw_rad)

    drone_n, drone_e = state.current_pos
    return (round(drone_n + delta_n, 2), round(drone_e + delta_e, 2))


# ─────────────────────────────────────────────────────────────
# METHOD C: RUNNING WEIGHTED CENTROID
# ─────────────────────────────────────────────────────────────

# Internal accumulators for weighted centroid (not in state — projection owns this)
_wc_sum_n:  float = 0.0
_wc_sum_e:  float = 0.0
_wc_sum_w:  float = 0.0


def reset_weighted_centroid() -> None:
    """Call at phase 3 entry to clear the C-method accumulator."""
    global _wc_sum_n, _wc_sum_e, _wc_sum_w
    _wc_sum_n = 0.0
    _wc_sum_e = 0.0
    _wc_sum_w = 0.0


def accumulate_weighted_centroid(
    pt: tuple[float, float],
    alt_m: float,
    ex_px: float,
) -> tuple[float, float] | None:
    """
    Method C: add an observation with reliability weight.

    Weight formula:
        w = 1 / (sqrt(alt_m) * (1 + |ex_px| / IMAGE_W))

    Lower altitude → higher weight (projection is more accurate).
    Smaller pixel offset → higher weight (object is more centred,
    less perspective distortion).

    Returns current weighted centroid estimate, or None if
    fewer than 3 observations accumulated.
    """
    global _wc_sum_n, _wc_sum_e, _wc_sum_w

    if alt_m <= 0.1:
        return None

    weight = 1.0 / (math.sqrt(max(alt_m, 0.5)) * (1.0 + abs(ex_px) / CAM_IMAGE_W))
    _wc_sum_n += pt[0] * weight
    _wc_sum_e += pt[1] * weight
    _wc_sum_w += weight

    if _wc_sum_w < 0.01:
        return None

    return (round(_wc_sum_n / _wc_sum_w, 2), round(_wc_sum_e / _wc_sum_w, 2))


def get_weighted_centroid() -> tuple[float, float] | None:
    """Return current Method C estimate (None if insufficient weight)."""
    if _wc_sum_w < 0.5:  # require meaningful accumulated weight
        return None
    return (round(_wc_sum_n / _wc_sum_w, 2), round(_wc_sum_e / _wc_sum_w, 2))


# ─────────────────────────────────────────────────────────────
# MEDIAN FILTER UTILITY
# ─────────────────────────────────────────────────────────────

def _median_ned(
    observations: list[tuple[float, float]],
    max_samples: int = 50,
) -> tuple[float, float] | None:
    """Median-filter a list of (N, E) observations."""
    if len(observations) < 3:
        return None
    recent = observations[-max_samples:]
    ns = sorted(p[0] for p in recent)
    es = sorted(p[1] for p in recent)
    return (round(ns[len(ns) // 2], 2), round(es[len(es) // 2], 2))


def accumulate_orange_observations(
    observations: list[tuple[float, float]],
    max_samples:  int = 50,
) -> tuple[float, float] | None:
    """Legacy single-list median filter — kept for compatibility."""
    return _median_ned(observations, max_samples)


# ─────────────────────────────────────────────────────────────
# TRIPLE-METHOD FUSION
# ─────────────────────────────────────────────────────────────

def fuse_orange_location() -> tuple[float, float] | None:
    """
    Triple-method fusion: blend medians of A and B with
    weighted centroid C.

    α_A + α_B + α_C = 1.0 (defined in config).

    Falls back gracefully:
      - All three available   → full weighted blend
      - Two available         → renormalise weights between those two
      - Only one available    → use it directly
      - None available (< 3 samples) → return None
    """
    med_a  = _median_ned(state.orange_observations_A)
    med_b  = _median_ned(state.orange_observations_B)
    wc     = get_weighted_centroid()

    alpha_a = ORANGE_LOC_BLEND_ALPHA
    alpha_b = ORANGE_LOC_BLEND_ALPHA_B
    alpha_c = ORANGE_LOC_BLEND_ALPHA_C

    available = [(med_a, alpha_a), (med_b, alpha_b), (wc, alpha_c)]
    valid     = [(pt, w) for pt, w in available if pt is not None]

    if not valid:
        return None

    if len(valid) == 1:
        return valid[0][0]

    # Renormalise weights among available methods
    total_w = sum(w for _, w in valid)
    fused_n = sum(pt[0] * (w / total_w) for pt, w in valid)
    fused_e = sum(pt[1] * (w / total_w) for pt, w in valid)
    return (round(fused_n, 2), round(fused_e, 2))

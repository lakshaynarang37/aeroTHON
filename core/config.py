"""
core/config.py
─────────────────────────────────────────────────────────────
All tunable constants for the SkyScan mission.

REVISION CHANGES:
  • PERIMETER_SPEED_MPS      → 0.6  (phase 3A safe for real drone)
  • SURVEY_SPEED_MPS         → 0.7
  • GREEN speeds reduced for real-drone safety
  • ORANGE tracking speeds further reduced for real drone
  • ORANGE_YAW_RATE_MAX      → 8 deg/s  (was 12)
  • ORANGE_TRANSIT_SPEED_WAIT → 25 s  (was 20; safer)
  • ORANGE_LOC_BLEND_ALPHA_B  + ALPHA_C  → triple-method fusion
  • Phase 4→5 transit: position-mode only, speed is commented out
    because PX4 native speed governs; use wait time to cap it.
  • RED_SEEN_FRAMES_CONFIRM   → 15 frames (was 10)
  • PHASE_3A_QR_CONFIRM_FRAMES → debounce QR on perimeter
─────────────────────────────────────────────────────────────
"""

# ── Camera ────────────────────────────────────────────────────
CAMERA_TOPIC        = "/camera_down"
VISION_LATENCY_SEC  = 0.32
FRAME_INTERVAL_SEC  = 0.08

# ── Camera optics (for ground projection) ────────────────────
CAM_HFOV_DEG        = 80.0
CAM_IMAGE_W         = 640
CAM_IMAGE_H         = 480

# ── Takeoff ───────────────────────────────────────────────────
TAKEOFF_ALT_M       = 5.0
TAKEOFF_HOVER_SEC   = 4.0
TAKEOFF_STABLE_VEL  = 0.15

# ── Altitude hold ─────────────────────────────────────────────
ALTITUDE_HOLD_KP    = 0.3
ALTITUDE_HOLD_MAX   = 0.5

# ── Scan-zone (QR discovery near start pad) ───────────────────
SCAN_CORNERS = [
    (-7.0, -7.0), (-7.0,  2.5),
    ( 7.0,  2.5), ( 7.0, -2.5),
]
SCAN_STEP_M         = 2.0
SCAN_SPEED_MPS      = 0.8
SCAN_ALT_M          = 5.0

# ── Survey zone ───────────────────────────────────────────────
SURVEY_CORNERS = [
    (10.0, -15.0), (50.0, -15.0),
    (50.0,  15.0), (10.0,  15.0),
]
SURVEY_CENTER = (
    sum(p[0] for p in SURVEY_CORNERS) / len(SURVEY_CORNERS),
    sum(p[1] for p in SURVEY_CORNERS) / len(SURVEY_CORNERS),
)
NO_FLY_ZONES: list = []

# ── Lawnmower survey ─────────────────────────────────────────
SURVEY_STEP_M       = 3.0
SURVEY_SPEED_MPS    = 0.7           # was 1.0 → 0.7 real-drone safe
SURVEY_ALT_M        = 4.0
SURVEY_YAW_DEG      = 0.0

# ── Perimeter scout ───────────────────────────────────────────
# REAL DRONE: 0.6 m/s gives vision >80 ms per frame-width at 4 m alt.
# At 640 px / 80° FOV → ~5.7 m swath → 9.5 s to cross → safe.
PERIMETER_SPEED_MPS = 0.6           # was 2.5 → 1.0 → 0.6
PERIMETER_ALT_M     = 4.0
PERIMETER_YAW_DEG   = 90.0

# ── QR debounce on perimeter scout ───────────────────────────
# Require this many consecutive frames of correct-ID QR detection
# before breaking from the perimeter loop — avoids false breaks
# on QR fragments or partial reads while the drone is moving.
PHASE_3A_QR_CONFIRM_FRAMES = 5

# ── PD centering gains ────────────────────────────────────────
PD_KP               = 0.0010        # was 0.0015 → 0.0012 → 0.0010
PD_KD               = 0.0006        # was 0.001  → 0.0008 → 0.0006
PD_MAX_VEL_MPS      = 0.20          # was 0.4 → 0.25 → 0.20  m/s
PD_STABLE_PIX       = 30
PD_STABLE_FRAMES    = 25

# ── QR centering buffer (Phase 4) ────────────────────────────
QR_CENTER_BUFFER_PIX = 45

# ── Green-lane tracking ───────────────────────────────────────
GREEN_FWD_FAST      = 0.6           # was 1.2 → 0.7 → 0.6
GREEN_FWD_SLOW      = 0.30          # was 0.5 → 0.35 → 0.30
GREEN_YAW_FINE      = 0.007
GREEN_YAW_COARSE    = 0.012
GREEN_LAT_SCALE     = 0.0015
GREEN_EX_THRESHOLD  = 60
GREEN_LOST_LIMIT    = 25
GREEN_YAW_BIAS_K    = 0.003

# ── Orange-lane tracking — REAL DRONE SAFE ────────────────────
# Speeds deliberately very conservative.  The drone should never
# exceed ~0.3 m/s forward during return so it can react to the
# red pad before overshooting.
ORANGE_FWD_FAST     = 0.25          # was 0.30 → 0.25
ORANGE_FWD_SLOW     = 0.12          # was 0.15 → 0.12
ORANGE_YAW_FINE     = 0.04          # proportional yaw (deg/s per pixel)
ORANGE_YAW_COARSE   = 0.07
ORANGE_LAT_SCALE    = 0.0010        # was 0.0012 → 0.0010
ORANGE_EX_THRESHOLD = 55
ORANGE_YAW_BIAS_K   = 0.0025        # was 0.003 → 0.0025
ORANGE_BLIND_FWD    = 0.10          # was 0.15 → 0.10 (dead-reckoning creep)
ORANGE_YAW_RATE_MAX = 8.0           # deg/s hard cap — was 12 → 8

# ── Phase 4→5 slow transit ────────────────────────────────────
# Position-mode transit to the orange exit. PX4 controls speed
# internally — we do NOT set velocity here (line intentionally
# commented / removed to prevent accidental sprint):
#   # state.body_vel = ...  ← DO NOT use velocity mode here
# The effective max speed is therefore: distance / wait_time.
# Example: 12 m gap / 25 s wait ≈ 0.48 m/s max — safe for real drone.
# Increase ORANGE_TRANSIT_SPEED_WAIT if your delivery zone is farther
# from the orange lane entry.
ORANGE_TRANSIT_SPEED_WAIT = 25.0    # seconds — was 20 → 25

# ── Orange exit location — TRIPLE-METHOD FUSION ───────────────
# Three independent estimates are computed and blended:
#   A (α_A): Trigonometric pixel→ground projection (FOV + altitude)
#   B (α_B): Dead-reckoning from drone bearing + lateral pixel offset
#   C (α_C): Running weighted centroid (weight by 1/sqrt(alt) and centredness)
# α_A + α_B + α_C = 1.0 exactly.
# Higher α_A because trig method is most geometrically grounded.
ORANGE_LOC_BLEND_ALPHA   = 0.50     # weight for trig method A
ORANGE_LOC_BLEND_ALPHA_B = 0.25     # weight for bearing method B
ORANGE_LOC_BLEND_ALPHA_C = 0.25     # weight for weighted centroid C

# ── Red pad confirmation debounce ─────────────────────────────
RED_SEEN_FRAMES_CONFIRM = 15        # was 10 → 15

# ── Payload hold ─────────────────────────────────────────────
PAYLOAD_HOLD_SEC    = 5.0

# ── Heartbeat ────────────────────────────────────────────────
HEARTBEAT_INTERVAL  = 0.1

# ── Vision HSV thresholds ────────────────────────────────────
HSV_GREEN_LO  = (35,  50,  50)
HSV_GREEN_HI  = (85, 255, 255)
HSV_ORANGE_LO = (12, 100, 100)
HSV_ORANGE_HI = (25, 255, 255)
HSV_RED1_LO   = (  0, 120,  70)
HSV_RED1_HI   = (  8, 255, 255)
HSV_RED2_LO   = (172, 120,  70)
HSV_RED2_HI   = (180, 255, 255)
GREEN_MIN_AREA  = 5000
ORANGE_MIN_AREA = 5000
RED_MIN_AREA    = 12000

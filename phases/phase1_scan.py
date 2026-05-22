"""
phases/phase1_scan.py
─────────────────────────────────────────────────────────────
PHASE 1 — TRANSIT TO START PAD + QR TARGET ACQUISITION

Sub-phases:
  1A  Transit to start-pad hover position
  1B  Quick visual check (6 s) — grab QR if already in frame
  1C  Slow lawnmower scan of SCAN_CORNERS if 1B fails
  1D  PD precision centering on the found QR

Failsafes:
  • If QR is never found after full lawnmower → land + raise.
  • PD centering timeout → log warning and proceed anyway
    (we have the ID, centering just wasn't perfect — non-fatal).
─────────────────────────────────────────────────────────────
"""

import asyncio

import core.state as state
from core.config import (
    SCAN_CORNERS, SCAN_STEP_M, SCAN_SPEED_MPS, SCAN_ALT_M,
)
from utils.navigation import (
    generate_lawnmower,
    execute_smooth_scan,
    go_to,
    pd_center,
)

# Start-pad hover point (tweak in config if your world differs)
START_PAD_N = -5.0
START_PAD_E =  0.0


async def run(drone) -> None:
    """
    Run Phase 1 end-to-end.
    Raises RuntimeError if no QR target is found.
    """

    # ── 1A: Transit to start-pad hover ───────────────────────
    state.mission_state = "PHASE 1A: TRANSIT TO START PAD"
    print(f"[FSM] {state.mission_state}")
    await go_to(START_PAD_N, START_PAD_E, SCAN_ALT_M, yaw_deg=0.0, wait=8.0)

    # ── 1B: Quick visual check ───────────────────────────────
    state.mission_state = "PHASE 1B: SCANNING START PAD"
    print(f"[FSM] {state.mission_state}")

    for _ in range(60):                          # ~6 seconds
        v = state.read_vision()
        if v["qr_detected"] and v["qr_id"]:
            state.target_id = v["qr_id"]
            print(f"\n[MEMORY] Target locked instantly: '{state.target_id}'\n")
            break
        await asyncio.sleep(0.1)

    # ── 1C: Lawnmower scan if still no target ────────────────
    if not state.target_id:
        state.mission_state = "PHASE 1C: LAWNMOWER SCAN"
        print(f"[FSM] {state.mission_state}")

        base_path = generate_lawnmower(
            SCAN_CORNERS, obstacles_list=[], step=SCAN_STEP_M, start_right=True
        )
        found = await execute_smooth_scan(
            base_path, alt=SCAN_ALT_M, speed=SCAN_SPEED_MPS, yaw_deg=0.0
        )

        if not found or not state.target_id:
            print("\033[91m[PHASE 1] CRITICAL: No QR found. Landing.\033[0m")
            await drone.action.land()
            raise RuntimeError("[PHASE 1] QR target not found — mission aborted.")

    # ── 1D: PD centering on the start QR ─────────────────────
    state.mission_state = "PHASE 1D: PD CENTER ON START QR"
    print(f"[FSM] {state.mission_state}")

    locked = await pd_center(target="qr", timeout=15.0)
    if not locked:
        print(
            "\033[93m[PHASE 1] PD centering timed out — QR ID is locked, "
            "proceeding with best-effort position.\033[0m"
        )
        # Non-fatal: we have the ID; a perfect centre isn't mandatory
        # for subsequent phases. Log and continue.

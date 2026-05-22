"""
utils/hud.py
─────────────────────────────────────────────────────────────
Draws the on-screen HUD overlay onto a BGR frame.
Reads from core.state only — zero side-effects.
─────────────────────────────────────────────────────────────
"""

import cv2
import core.state as state


def draw_hud(frame):
    hud  = frame.copy()
    h, w = hud.shape[:2]
    cx, cy = w // 2, h // 2
    v    = state.read_vision()

    # ── Top dashboard bar ─────────────────────────────────────
    cv2.rectangle(hud, (0, 0), (w, 100), (15, 15, 15), -1)

    cv2.putText(hud, f"STATE: {state.mission_state}",
                (15, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

    lock_color = (0, 255, 0) if state.target_id else (0, 0, 255)
    cv2.putText(hud, f"TARGET ID: {state.target_id or 'SEARCHING...'}",
                (15, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, lock_color, 2)

    scout_color = (0, 165, 255) if state.orange_exit_location else (100, 100, 100)
    scout_text  = (
        f"ORANGE EXIT: {state.orange_exit_location}"
        if state.orange_exit_location
        else "ORANGE EXIT: NOT FOUND"
    )
    cv2.putText(hud, scout_text,
                (15, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.5, scout_color, 2)

    # ── Crosshair ─────────────────────────────────────────────
    cv2.line(hud, (cx - 15, cy), (cx + 15, cy), (255, 255, 255), 1)
    cv2.line(hud, (cx, cy - 15), (cx, cy + 15), (255, 255, 255), 1)

    # ── QR bounding box + error line ─────────────────────────
    if v["qr_detected"] and v["qr_rect"] is not None:
        qx, qy, qw, qh = v["qr_rect"]
        q_id  = v["qr_id"]
        color = (0, 255, 0) if q_id == state.target_id else (0, 255, 255)
        cv2.rectangle(hud, (qx, qy), (qx + qw, qy + qh), color, 2)
        cv2.putText(hud, q_id, (qx, qy - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        qcx, qcy = qx + qw // 2, qy + qh // 2
        cv2.line(hud, (cx, cy), (qcx, qcy), (0, 0, 255), 1)

    # ── Orange lane cue ───────────────────────────────────────
    if v["orange_detected"]:
        ox = cx + v["orange_ex"]
        cv2.circle(hud, (ox, cy + 50), 10, (0, 165, 255), -1)
        cv2.putText(hud, "ORANGE PATH", (ox - 40, cy + 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 165, 255), 1)

    # ── Red pad tracker ───────────────────────────────────────
    if v["red_detected"] and "RED" in state.mission_state:
        rx = cx + v["red_ex"]
        ry = cy + v["red_ey"]
        cv2.circle(hud, (rx, ry), 20, (0, 0, 255), 3)
        cv2.putText(hud, "RED PAD", (rx - 30, ry - 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

    # ── Green lane error readout ──────────────────────────────
    if v["green_detected"] and "CORRIDOR" in state.mission_state:
        cv2.putText(hud, f"LANE ERR: {v['green_ex']:+d}",
                    (15, h - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

    # ── Green vector line (if populated by vision) ────────────
    if v["green_detected"] and v.get("green_line"):
        x1, y1, x2, y2 = v["green_line"]
        cv2.line(hud, (x1, y1), (x2, y2), (0, 255, 0), 4)
        cv2.putText(hud, f"ANG: {v['green_angle']:.1f}",
                    (x1 + 10, y1), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    # ── Orange vector line (if populated by vision) ───────────
    if v["orange_detected"] and v.get("orange_line"):
        x1, y1, x2, y2 = v["orange_line"]
        cv2.line(hud, (x1, y1), (x2, y2), (0, 165, 255), 4)
        cv2.putText(hud, f"ANG: {v['orange_angle']:.1f}",
                    (x1 + 10, y1), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)

    return hud

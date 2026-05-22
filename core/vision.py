"""
core/vision.py
─────────────────────────────────────────────────────────────
Gazebo camera subscriber + CV pipeline.

Decoded results are written atomically into core.state so every
phase can call state.read_vision() without knowing anything
about OpenCV or pyzbar.

Call `start(node)` once from main to subscribe; the callback
runs in a Gazebo transport thread.
─────────────────────────────────────────────────────────────
"""

import time
import threading
import numpy as np
import cv2
from pyzbar import pyzbar
from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image

import core.state as state
from core.config import (
    CAMERA_TOPIC, FRAME_INTERVAL_SEC,
    HSV_GREEN_LO, HSV_GREEN_HI,
    HSV_ORANGE_LO, HSV_ORANGE_HI,
    HSV_RED1_LO, HSV_RED1_HI,
    HSV_RED2_LO, HSV_RED2_HI,
    GREEN_MIN_AREA, ORANGE_MIN_AREA, RED_MIN_AREA,
)

latest_frame = None          # raw BGR frame — read by HUD
_last_frame_time: float = 0.0
_kernel = np.ones((7, 7), np.uint8)


def _on_image_callback(msg: Image) -> None:
    global latest_frame, _last_frame_time

    # ── Rate-limit processing ─────────────────────────────────
    now = time.time()
    if now - _last_frame_time < FRAME_INTERVAL_SEC:
        return
    _last_frame_time = now

    if not state.camera_connected:
        print("\n\033[92m[CAMERA] CONNECTION ESTABLISHED! Receiving frames.\033[0m\n")
        state.camera_connected = True

    try:
        raw = np.frombuffer(msg.data, dtype=np.uint8)
        if msg.pixel_format_type == 1:          # MONO8
            img = cv2.cvtColor(raw.reshape((msg.height, msg.width)), cv2.COLOR_GRAY2BGR)
        elif msg.pixel_format_type == 3:        # RGB8
            img = cv2.cvtColor(raw.reshape((msg.height, msg.width, 3)), cv2.COLOR_RGB2BGR)
        else:
            img = raw.reshape((msg.height, msg.width, 3))

        img = cv2.resize(img, (640, 480))
        latest_frame = img.copy()

        h, w = img.shape[:2]
        cx, cy = w // 2, h // 2
        w3 = w // 3
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # ── QR decode ─────────────────────────────────────────
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        barcodes = pyzbar.decode(gray)
        qr_det, qr_id, qr_ex, qr_ey, qr_rect = False, None, 0, 0, None
        if barcodes:
            qr = barcodes[0]
            qx, qy, qw, qh = qr.rect
            qr_det  = True
            qr_id   = qr.data.decode("utf-8")
            qr_ex   = (qx + qw // 2) - cx
            qr_ey   = (qy + qh // 2) - cy
            qr_rect = (qx, qy, qw, qh)

        # ── Green detection ───────────────────────────────────
        mask_g = cv2.morphologyEx(
            cv2.inRange(hsv, HSV_GREEN_LO, HSV_GREEN_HI),
            cv2.MORPH_OPEN, _kernel
        )
        l_g = cv2.countNonZero(mask_g[:, :w3])
        c_g = cv2.countNonZero(mask_g[:, w3:2*w3])
        r_g = cv2.countNonZero(mask_g[:, 2*w3:])
        g_det = (l_g + c_g + r_g) > GREEN_MIN_AREA
        g_ex, g_sector = 0, 0
        if g_det:
            g_sector = 0 if c_g >= l_g and c_g >= r_g else (-1 if l_g > r_g else 1)
            cnts, _ = cv2.findContours(mask_g, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if cnts:
                M = cv2.moments(max(cnts, key=cv2.contourArea))
                if M["m00"] > 0:
                    g_ex = int(M["m10"] / M["m00"]) - cx

        # ── Orange detection ──────────────────────────────────
        mask_o = cv2.morphologyEx(
            cv2.inRange(hsv, HSV_ORANGE_LO, HSV_ORANGE_HI),
            cv2.MORPH_OPEN, _kernel
        )
        l_o = cv2.countNonZero(mask_o[:, :w3])
        c_o = cv2.countNonZero(mask_o[:, w3:2*w3])
        r_o = cv2.countNonZero(mask_o[:, 2*w3:])
        o_det = (l_o + c_o + r_o) > ORANGE_MIN_AREA
        o_ex, o_sector = 0, 0
        if o_det:
            o_sector = 0 if c_o >= l_o and c_o >= r_o else (-1 if l_o > r_o else 1)
            cnts, _ = cv2.findContours(mask_o, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if cnts:
                M = cv2.moments(max(cnts, key=cv2.contourArea))
                if M["m00"] > 0:
                    o_ex = int(M["m10"] / M["m00"]) - cx

        # ── Red detection ─────────────────────────────────────
        mask_r = cv2.morphologyEx(
            cv2.bitwise_or(
                cv2.inRange(hsv, HSV_RED1_LO, HSV_RED1_HI),
                cv2.inRange(hsv, HSV_RED2_LO, HSV_RED2_HI),
            ),
            cv2.MORPH_OPEN, _kernel
        )
        Mr = cv2.moments(mask_r)
        r_det, r_ex, r_ey = False, 0, 0
        if Mr["m00"] > RED_MIN_AREA:
            r_det = True
            r_ex  = int(Mr["m10"] / Mr["m00"]) - cx
            r_ey  = int(Mr["m01"] / Mr["m00"]) - cy

        # ── Atomic state write ────────────────────────────────
        state.write_vision({
            "qr_detected":     qr_det,  "qr_id":      qr_id,
            "qr_rect":         qr_rect, "qr_ex":      qr_ex,  "qr_ey": qr_ey,
            "green_detected":  g_det,   "green_ex":   g_ex,   "green_sector": g_sector,
            "orange_detected": o_det,   "orange_ex":  o_ex,   "orange_sector": o_sector,
            "red_detected":    r_det,   "red_ex":     r_ex,   "red_ey": r_ey,
        })

    except Exception as e:
        print(f"\033[91m[VISION ERROR] {e}\033[0m")


def start(node: Node) -> None:
    """Subscribe the camera callback on the given Gazebo Node."""
    node.subscribe(Image, CAMERA_TOPIC, _on_image_callback)
    print(f"[VISION] Subscribed to {CAMERA_TOPIC}")

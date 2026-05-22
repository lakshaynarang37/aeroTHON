"""
main.py — SkyScan Mission Orchestrator. Entry point — run with: python main.py

Thread architecture:
  Main thread  → OpenCV HUD window (GUI must be on main OS thread)
  Mission thread → asyncio event loop with all drone commands
  gz-transport thread → Gazebo camera callback (writes to state atomically)
"""

# Protobuf shim — must run before any gz import
import os, sys
import google.protobuf as _pb
if tuple(int(x) for x in _pb.__version__.split(".")[:2]) >= (3, 21):
    os.environ["PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION"] = "python"
    for _m in [k for k in sys.modules if "google.protobuf" in k or "gz.msgs" in k]:
        del sys.modules[_m]

import asyncio
import threading
import numpy as np
import cv2

from gz.transport13 import Node
from mavsdk import System

import core.state  as state
import core.vision as vision
from core.telemetry import update_position, update_attitude, heartbeat

from phases import (
    phase0_takeoff, phase1_scan, phase2_green_lane,
    phase3_survey, phase4_payload, phase5_return,
)
from utils.hud import draw_hud


async def run_mission() -> None:
    """
    Top-level mission coroutine — runs entirely inside the mission thread.
    Connects to drone, waits for EKF health, then runs phases in sequence.
    Any RuntimeError from a phase triggers a safe land and abort.
    """
    drone = System()
    await drone.connect(system_address="udpin://0.0.0.0:14540")

    print("[MAIN] Waiting for drone connection...")
    async for conn in drone.core.connection_state():
        if conn.is_connected:
            print("[MAIN] Drone connected.")
            break

    print("[MAIN] Waiting for EKF...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("[MAIN] EKF ready.")
            break

    # Telemetry tasks run in background for the whole mission
    asyncio.create_task(update_position(drone))
    asyncio.create_task(update_attitude(drone))
    asyncio.create_task(heartbeat(drone))

    try:
        await phase0_takeoff.run(drone)
        await phase1_scan.run(drone)
        await phase2_green_lane.run(drone)

        found = await phase3_survey.run(drone)
        if not found:
            print("\033[91m[MAIN] Target not found — landing.\033[0m")
            state.mission_state = "ABORT: TARGET NOT FOUND"
            await drone.action.land()
            return

        await phase4_payload.run(drone)
        await phase5_return.run(drone)

        state.mission_state = "MISSION COMPLETE"
        print("\n\033[92m[MAIN] ✓ MISSION COMPLETE\033[0m\n")

    except RuntimeError as e:
        print(f"\033[91m[MAIN] Aborted: {e}\033[0m")
        state.mission_state = f"ABORTED: {e}"
        try:
            await drone.action.land()
        except Exception:
            pass

    except Exception as e:
        import traceback
        print(f"\033[91m[MAIN] Unexpected error: {e}\033[0m")
        traceback.print_exc()
        state.mission_state = "EMERGENCY LAND"
        try:
            await drone.action.land()
        except Exception:
            pass


if __name__ == "__main__":
    # Start Gazebo camera subscriber (runs in gz-transport's own thread)
    node = Node()
    vision.start(node)

    # Mission runs in a daemon thread with its own asyncio event loop.
    # Daemon = killed automatically when the main thread (HUD) exits.
    threading.Thread(
        target=lambda: asyncio.run(run_mission()),
        name="MissionThread",
        daemon=True,
    ).start()

    # HUD window — MUST be created and driven from the main OS thread.
    # OpenCV HighGUI crashes or shows blank windows if called from threads.
    cv2.namedWindow("SkyScan HUD", cv2.WINDOW_AUTOSIZE)

    placeholder = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2.putText(placeholder, "WAITING FOR GAZEBO CAMERA...",
                (80, 240), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
    cv2.putText(placeholder, "(Check simulation is not paused)",
                (100, 280), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)

    first_frame = True
    print("\n[MAIN] HUD open. Press 'q' to close.")

    while True:
        if cv2.waitKey(30) & 0xFF == ord("q"):
            break
        frame = vision.latest_frame
        if frame is not None:
            cv2.imshow("SkyScan HUD", draw_hud(frame))
            if first_frame:
                print("\033[92m[MAIN] Camera live.\033[0m\n")
                first_frame = False
        else:
            cv2.imshow("SkyScan HUD", placeholder)

    cv2.destroyAllWindows()

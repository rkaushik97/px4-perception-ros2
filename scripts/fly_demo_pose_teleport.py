#!/usr/bin/env python3
"""
Smooth flyover demo for YOLO perception showcase.

The drone follows one continuous path:
  - takeoff (climb from ground to cruise altitude)
  - cruise (constant altitude, constant velocity, straight line forward)
  - descend (smooth landing past the scene)

The whole trajectory is parameterized as t in [0, 1] across the full duration.
No per-segment easing means no oscillation during cruise.
"""

import subprocess
import time

# ---------- Trajectory parameters ----------
TOTAL_DURATION = 30.0   # seconds for the whole flight
UPDATE_HZ = 30
UPDATE_DT = 1.0 / UPDATE_HZ

# Phase boundaries as fractions of TOTAL_DURATION:
#   0.00 - 0.20  takeoff (climb to altitude)
#   0.20 - 0.85  cruise  (constant z, x advances linearly)
#   0.85 - 1.00  descent
TAKEOFF_END  = 0.20
CRUISE_END   = 0.85

# Spatial parameters
START_X      = -8.0
END_X        = 38.0
GROUND_Z     = 1.0
CRUISE_Z     = 8.0
END_Z        = 2.0

# Camera pitch (Y-axis quaternion)
LEVEL_PITCH  = (0.000, 1.000)   # (qy, w)
CRUISE_PITCH = (0.215, 0.977)   # ~25° down


def set_pose(x, y, z, qy, w):
    req = (
        f"name: 'x500_mono_cam_0', "
        f"position: {{x: {x:.3f}, y: {y:.3f}, z: {z:.3f}}}, "
        f"orientation: {{x: 0, y: {qy:.4f}, z: 0, w: {w:.4f}}}"
    )
    subprocess.run(
        ["gz", "service",
         "-s", "/world/default/set_pose",
         "--reqtype", "gz.msgs.Pose",
         "--reptype", "gz.msgs.Boolean",
         "--timeout", "100",
         "--req", req],
        capture_output=True,
    )


def smoothstep(t):
    """Smooth ease in/out, t in [0, 1]."""
    return t * t * (3 - 2 * t)


def lerp(a, b, t):
    return a + (b - a) * t


def trajectory(t):
    """
    t in [0, 1] over the full flight. Returns (x, y, z, qy, w).

    Cruise leg uses LINEAR x and CONSTANT z, so no oscillation.
    Takeoff and descent use smoothstep for graceful transitions.
    """
    if t < TAKEOFF_END:
        # Takeoff: x stays at START_X, z climbs ground → cruise
        u = t / TAKEOFF_END
        s = smoothstep(u)
        x = START_X
        z = lerp(GROUND_Z, CRUISE_Z, s)
        qy = lerp(LEVEL_PITCH[0], CRUISE_PITCH[0], s)
        w  = lerp(LEVEL_PITCH[1], CRUISE_PITCH[1], s)

    elif t < CRUISE_END:
        # Cruise: linear x from START_X → END_X (approx), constant z, constant pitch
        u = (t - TAKEOFF_END) / (CRUISE_END - TAKEOFF_END)
        x = lerp(START_X, END_X, u)
        z = CRUISE_Z
        qy = CRUISE_PITCH[0]
        w  = CRUISE_PITCH[1]

    else:
        # Descent: x stays at END_X, z drops to landing
        u = (t - CRUISE_END) / (1.0 - CRUISE_END)
        s = smoothstep(u)
        x = END_X
        z = lerp(CRUISE_Z, END_Z, s)
        qy = lerp(CRUISE_PITCH[0], LEVEL_PITCH[0], s)
        w  = lerp(CRUISE_PITCH[1], LEVEL_PITCH[1], s)

    return x, 0.0, z, qy, w


def main():
    print("Starting smooth flyover demo...")
    start = time.time()
    next_tick = start

    while True:
        elapsed = time.time() - start
        t = elapsed / TOTAL_DURATION
        if t >= 1.0:
            break

        x, y, z, qy, w = trajectory(t)
        set_pose(x, y, z, qy, w)

        # Pace at UPDATE_HZ but skip ticks if we fell behind
        next_tick += UPDATE_DT
        sleep_for = next_tick - time.time()
        if sleep_for > 0:
            time.sleep(sleep_for)

    # Final landing pose
    x, y, z, qy, w = trajectory(1.0)
    set_pose(x, y, z, qy, w)
    print("Done.")


if __name__ == "__main__":
    main()
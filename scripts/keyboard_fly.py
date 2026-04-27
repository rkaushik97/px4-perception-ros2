#!/usr/bin/env python3
"""
Keyboard teleop for the simulated drone.

Run AFTER PX4 + Gazebo are up and the drone is spawned.

Controls:
   W / S   forward / backward
   A / D   left / right (strafe)
   R / F   up / down
   Q / E   yaw left / yaw right
   T / G   pitch camera up / down
   space   reset to neutral pitch
   x       quit

Hold keys for continuous motion. Releases stop motion immediately.
"""

import math
import select
import subprocess
import sys
import termios
import time
import tty

# ---------- Tunables ----------
UPDATE_HZ        = 30
UPDATE_DT        = 1.0 / UPDATE_HZ
LINEAR_SPEED     = 4.0   # m/s
VERTICAL_SPEED   = 2.5   # m/s
YAW_SPEED        = 0.8   # rad/s
PITCH_SPEED      = 0.6   # rad/s

# Initial state
state = {
    "x": -8.0,
    "y":  0.0,
    "z":  3.0,
    "yaw":   0.0,   # radians, 0 = facing +X
    "pitch": 0.215, # camera tilt down (radians)
}


def euler_to_quat(yaw, pitch):
    """Convert yaw (around Z) + pitch (around Y) to quaternion."""
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    qx = -sp * sy
    qy =  sp * cy
    qz =  cp * sy
    qw =  cp * cy
    return qx, qy, qz, qw


def set_pose(x, y, z, yaw, pitch):
    qx, qy, qz, qw = euler_to_quat(yaw, pitch)
    req = (
        f"name: 'x500_mono_cam_0', "
        f"position: {{x: {x:.3f}, y: {y:.3f}, z: {z:.3f}}}, "
        f"orientation: {{x: {qx:.4f}, y: {qy:.4f}, z: {qz:.4f}, w: {qw:.4f}}}"
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


def get_key_nonblocking(timeout):
    """Read one key if available within `timeout` seconds. Returns '' if none."""
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    if rlist:
        return sys.stdin.read(1)
    return ""


HELP = """
Keyboard drone control. Hold keys for continuous motion.

   W / S   forward / backward (in drone heading)
   A / D   strafe left / right
   R / F   up / down
   Q / E   yaw left / right
   T / G   pitch camera up / down
  space    reset camera pitch to neutral
   x       quit
"""


def main():
    print(HELP)
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        # Send initial pose
        set_pose(**state)
        last_tick = time.time()

        while True:
            now = time.time()
            dt = now - last_tick
            last_tick = now

            # Drain all available keypresses since last tick (low latency)
            keys = ""
            while True:
                k = get_key_nonblocking(0)
                if not k:
                    break
                keys += k

            if "x" in keys:
                print("\nQuitting.")
                break

            # Compute movement deltas (heading-relative)
            forward = 0.0
            strafe = 0.0
            vertical = 0.0
            dyaw = 0.0
            dpitch = 0.0

            if "w" in keys: forward  += LINEAR_SPEED * dt
            if "s" in keys: forward  -= LINEAR_SPEED * dt
            if "d" in keys: strafe   += LINEAR_SPEED * dt
            if "a" in keys: strafe   -= LINEAR_SPEED * dt
            if "r" in keys: vertical += VERTICAL_SPEED * dt
            if "f" in keys: vertical -= VERTICAL_SPEED * dt
            if "q" in keys: dyaw     += YAW_SPEED * dt
            if "e" in keys: dyaw     -= YAW_SPEED * dt
            if "t" in keys: dpitch   -= PITCH_SPEED * dt   # tilt camera up
            if "g" in keys: dpitch   += PITCH_SPEED * dt   # tilt camera down
            if " " in keys: state["pitch"] = 0.215         # reset pitch

            # Convert heading-relative motion to world frame
            cy, sy = math.cos(state["yaw"]), math.sin(state["yaw"])
            state["x"] += forward * cy - strafe * sy
            state["y"] += forward * sy + strafe * cy
            state["z"] = max(0.5, state["z"] + vertical)
            state["yaw"]   += dyaw
            state["pitch"] = max(-0.5, min(1.0, state["pitch"] + dpitch))

            set_pose(**state)

            # Pace
            elapsed = time.time() - now
            sleep = UPDATE_DT - elapsed
            if sleep > 0:
                time.sleep(sleep)

    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


if __name__ == "__main__":
    main()
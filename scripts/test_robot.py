#!/usr/bin/env python3
"""
Connect to the Franka, read joint state, disconnect (no motion).
Set robot IP: export ROBOT_IP=192.168.2.10   (default below)
"""
import os
import subprocess
import sys

ROBOT_IP = os.environ.get("ROBOT_IP", "192.168.2.10")
SKIP_PING = os.environ.get("SKIP_PING", "").lower() in ("1", "true", "yes")


def ping_robot(ip: str) -> bool:
    try:
        r = subprocess.run(
            ["ping", "-c", "1", "-W", "2", ip],
            capture_output=True,
            timeout=5,
        )
        return r.returncode == 0
    except (subprocess.TimeoutExpired, FileNotFoundError):
        return False


def main() -> int:
    print("=" * 60)
    print("TEST 2 — robot connection (read-only)")
    print("=" * 60)
    print(f"Robot IP: {ROBOT_IP}")
    print("(override with: export ROBOT_IP=<your-ip>)")
    print()

    if not SKIP_PING and not ping_robot(ROBOT_IP):
        print("FAIL — robot not reachable (ping)")
        print()
        print("This PC must be on the Franka subnet, e.g.:")
        print("  PC and robot on same subnet, e.g. PC 192.168.2.x | Robot 192.168.2.10")
        print("  See docs/ROBOT_CONNECTION.md")
        print()
        print("To skip ping and try libfranka anyway: export SKIP_PING=1")
        return 1

    try:
        import pyfranka_interface as franka
        import numpy as np
    except ImportError as e:
        print("FAIL — run scripts/test_import.py first:", e)
        return 1

    print("Connecting...")
    try:
        robot = franka.Robot_(
            ROBOT_IP,
            False,
            hand_franka=False,
            auto_init=True,
            speed_factor=0.1,
        )
        state = robot.getState()
    except Exception as e:
        print("FAIL — could not connect or read state:")
        print(" ", e)
        print()
        print("Checklist:")
        print("  • Ethernet to control box, same subnet as robot (192.168.2.0/24)")
        print("  • ping", ROBOT_IP)
        print("  • Desk: brakes unlocked, FCI activated")
        print("  • docs/ROBOT_CONNECTION.md")
        return 1

    q = np.array(state.q)
    print("OK — connected")
    print()
    print("Joint positions (rad):")
    print(" ", q.tolist())
    print()
    for i, val in enumerate(q, 1):
        print(f"  Joint {i}: {val:.6f} rad ({np.degrees(val):.2f} deg)")
    print()
    print("End-effector pose (4x4):")
    print(state.T)
    return 0


if __name__ == "__main__":
    sys.exit(main())

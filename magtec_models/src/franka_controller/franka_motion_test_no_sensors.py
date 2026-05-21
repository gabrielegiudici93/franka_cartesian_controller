#!/usr/bin/env python3
"""
Robot-only motion test using MagTec workspace poses.

- Uses the same 10-point grid as example 04 (franka_10_random_points.py)
- No StretchMagTec (magnetic skin) serial port
- No FT sensor — small fixed Z dip instead of force-controlled press

Prerequisites: FCI active, brakes unlocked (see docs/ROBOT_CONNECTION.md).

Usage:
  python3 src/franka_controller/franka_motion_test_no_sensors.py
  python3 src/franka_controller/franka_motion_test_no_sensors.py --points 1 2 3
  python3 src/franka_controller/franka_motion_test_no_sensors.py --all --indent-mm 0.5
"""

from __future__ import annotations

import argparse
import sys
import os
import time

import numpy as np
import pyfranka_interface as franka
from scipy.spatial.transform import Rotation as R

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from franka_controller.config import CONFIG_DIR, ROBOT_IP, ROBOT_SPEED_FACTOR

# Reuse proven motion helpers from the FT example
from franka_controller.franka_10_random_points import (
    MULTI_POINT_OFFSETS,
    TARGET_POSITION_COORDS,
    move_joints_safe,
    move_to_position,
    move_relative,
    safe_robot_move,
    ABSOLUTE_MOVEMENT_DURATION,
)

DEFAULT_INDENT_M = 0.001  # 1 mm Z press without FT


def _load_initial_joints():
    path = CONFIG_DIR / "hardware.yaml"
    if path.exists():
        try:
            import yaml

            with open(path, "r", encoding="utf-8") as f:
                data = yaml.safe_load(f) or {}
            joints = data.get("initial_joints")
            if joints and len(joints) == 7:
                return list(joints)
        except Exception as exc:
            print(f"[warn] Could not read initial_joints from {path}: {exc}")
    return [
        -1.460883997177473,
        -1.4397968588005559,
        1.8498105422813298,
        -1.680352194797862,
        1.4646542101436189,
        1.8593807739681665,
        0.8594902150722012,
    ]


def fixed_z_press(r, indent_m: float) -> None:
    """Small downward move and return (no force sensor)."""
    start_z = r.getState().T[2, 3]
    print(f"  Z dip: {indent_m * 1000:.2f} mm down, then return")
    move_relative(r, 0, 0, -indent_m, duration=0.3)
    time.sleep(0.3)
    move_relative(r, 0, 0, indent_m, duration=0.3)
    end_z = r.getState().T[2, 3]
    print(f"  Z: {start_z:.6f} -> {end_z:.6f} m")


def connect_robot():
    print(f"Connecting to robot at {ROBOT_IP}...")
    for attempt in range(5):
        try:
            r = franka.Robot_(
                ROBOT_IP,
                False,
                hand_franka=False,
                auto_init=True,
                speed_factor=ROBOT_SPEED_FACTOR,
            )
            print("Robot connected")
            return r
        except Exception as e:
            print(f"  attempt {attempt + 1}/5: {e}")
            if "Reflex" in str(e) or "reflex" in str(e).lower():
                input("Unlock robot / reset reflex, then press Enter...")
            elif attempt == 4:
                raise
            time.sleep(2)
    return None


def main() -> int:
    parser = argparse.ArgumentParser(
        description="MagTec grid motion test — robot only, no skin or FT sensor"
    )
    parser.add_argument(
        "--points",
        nargs="+",
        default=["1"],
        help="Point IDs 1-10 to visit (default: 1 only)",
    )
    parser.add_argument(
        "--all",
        action="store_true",
        help="Visit all 10 grid points",
    )
    parser.add_argument(
        "--indent-mm",
        type=float,
        default=DEFAULT_INDENT_M * 1000,
        help="Fixed Z press depth in mm (default: 1.0)",
    )
    parser.add_argument(
        "--no-press",
        action="store_true",
        help="Only move to poses, skip Z dip",
    )
    parser.add_argument(
        "--no-prompt",
        action="store_true",
        help="Do not wait for Enter between points",
    )
    args = parser.parse_args()

    if args.all:
        point_names = list(MULTI_POINT_OFFSETS.keys())
    else:
        point_names = [str(p) for p in args.points]

    indent_m = max(0.0, args.indent_mm / 1000.0)
    initial_joints = _load_initial_joints()

    print("=" * 70)
    print(" MAGTEC ROBOT MOTION TEST (no skin, no FT)")
    print("=" * 70)
    print(f"Robot IP: {ROBOT_IP}")
    print(f"Points: {', '.join(point_names)}")
    print(f"Base XYZ: {TARGET_POSITION_COORDS}")
    print(f"Z dip: {indent_m * 1000:.2f} mm" if not args.no_press else "Z dip: disabled")
    print("=" * 70)

    r = connect_robot()

    print("\nMoving to initial joint pose...")
    move_joints_safe(r, initial_joints)
    print("Initial pose reached\n")

    try:
        for i, name in enumerate(point_names):
            if name not in MULTI_POINT_OFFSETS:
                print(f"Unknown point '{name}', skipping (valid: 1-10)")
                continue

            offset = MULTI_POINT_OFFSETS[name]
            target_pos = np.array(TARGET_POSITION_COORDS) + np.array(offset)
            print(f"--- Point {name} ({i + 1}/{len(point_names)}) ---")
            print(
                f"  Target: x={target_pos[0]:.6f} y={target_pos[1]:.6f} z={target_pos[2]:.6f}"
            )

            if not args.no_prompt:
                try:
                    input("  Press Enter to move (Ctrl+C to stop)... ")
                except KeyboardInterrupt:
                    print("\nStopped by user")
                    break

            move_to_position(r, target_pos)
            time.sleep(0.5)
            print("  Pose reached")

            if not args.no_press and indent_m > 0:
                fixed_z_press(r, indent_m)

            print("  Done\n")

    except KeyboardInterrupt:
        print("\nInterrupted")
    finally:
        print("Returning to initial joint pose...")
        try:
            move_joints_safe(r, initial_joints)
        except Exception as e:
            print(f"  warn: could not return home: {e}")
        try:
            r.stop()
        except Exception:
            pass
        print("Finished")
    return 0


if __name__ == "__main__":
    sys.exit(main())

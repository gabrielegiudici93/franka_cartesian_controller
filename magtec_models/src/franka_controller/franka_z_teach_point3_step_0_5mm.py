#!/usr/bin/env python3
"""
Z teaching utility derived from the working quick shear script structure.

Behavior:
- Same robot setup flow (connect -> initial joints -> move to point)
- Move directly above point 3
- Wait for Enter, then descend 0.5mm per Enter
- Print absolute Z and cumulative descent each step
- Type 'q' + Enter to stop and print final values
"""

import os
import sys
import time

import numpy as np
import pyfranka_interface as franka
from scipy.spatial.transform import Rotation as R

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from franka_controller.config import *  # noqa: F403,F401


TARGET_POSITION_COORDS = np.array([0.500781, 0.419620, 0.034311], dtype=float)
MULTI_POINT_OFFSETS = {
    "1": [0.0, 0.0, 0.0],
    "2": [-0.01, 0.0, 0.0],
    "3": [-0.01, 0.01, 0.0],
    "4": [0.0, 0.01, 0.0],
    "5": [0.0, 0.02, 0.0],
    "6": [-0.01, 0.02, 0.0],
    "7": [-0.01, 0.03, 0.0],
    "8": [0.0, 0.03, 0.0],
    "9": [0.0, 0.04, 0.0],
    "10": [-0.01, 0.04, 0.0],
}
INITIAL_JOINT_POSITIONS = [
    -1.460883997177473,
    -1.4397968588005559,
    1.8498105422813298,
    -1.680352194797862,
    1.4646542101436189,
    1.8593807739681665,
    0.8594902150722012,
]

STEP_DZ = 0.0005  # 0.5mm


def safe_robot_move(robot, move_type, target, duration=None, max_retries=3):
    for attempt in range(max_retries):
        try:
            if duration is None:
                robot.move(move_type, target)
            else:
                robot.move(move_type, target, duration)
            return True
        except Exception as e:
            error_str = str(e)
            print(f"   ❌ Movement attempt {attempt + 1}/{max_retries} failed: {error_str}")
            if "Reflex" in error_str or "reflex" in error_str.lower():
                print("   🛑 Robot in reflex mode. Unlock/reset and press Enter to retry...")
                input()
                time.sleep(3)
            elif attempt < max_retries - 1:
                time.sleep(2)
            else:
                raise
    return False


def move_joints_safe(robot, target_joints, speed_factor=0.05):
    current_joints = robot.getState().q
    joint_diffs = np.array(target_joints) - np.array(current_joints)
    num_steps = max(10, int(np.max(np.abs(joint_diffs)) / 0.1))
    for step in range(num_steps):
        alpha = (step + 1) / num_steps
        intermediate_joints = current_joints + alpha * joint_diffs
        robot.move_joints(intermediate_joints, speed_factor)
        time.sleep(0.2)


def move_to_position(robot, target_pos):
    orientation = R.from_euler("xyz", [180, 0, 0], degrees=True)
    target_pose = np.eye(4)
    target_pose[:3, :3] = orientation.as_matrix()
    target_pose[:3, 3] = target_pos
    safe_robot_move(robot, "absolute", target_pose, duration=2.0)


def move_relative(robot, dx, dy, dz, duration=0.4):
    delta = np.eye(4)
    delta[:3, 3] = [dx, dy, dz]
    safe_robot_move(robot, "relative", delta, duration=duration)


def main():
    print("=== Z TEACH TOOL (POINT 3, FROM WORKING SETUP FLOW) ===")
    point3 = TARGET_POSITION_COORDS + np.array(MULTI_POINT_OFFSETS["3"], dtype=float)
    print(f"Target point 3: [{point3[0]:.6f}, {point3[1]:.6f}, {point3[2]:.6f}]")
    print(f"Step per Enter: {STEP_DZ*1000:.1f} mm down")

    robot = franka.Robot_(ROBOT_IP, False, hand_franka=False, auto_init=True, speed_factor=ROBOT_SPEED_FACTOR)
    try:
        print("Moving to initial joints...")
        move_joints_safe(robot, INITIAL_JOINT_POSITIONS, speed_factor=0.05)

        print("Moving to point 3 reference position...")
        move_to_position(robot, point3)
        time.sleep(0.4)

        start_z = float(robot.getState().T[2, 3])
        print(f"\nstart_z = {start_z:.6f} m")
        print("Press Enter to descend by 0.5mm each step. Type 'q' and Enter to finish.")

        step_idx = 0
        while True:
            cmd = input("> ").strip().lower()
            if cmd == "q":
                break
            move_relative(robot, 0.0, 0.0, -STEP_DZ, duration=0.4)
            time.sleep(0.1)
            step_idx += 1
            current_z = float(robot.getState().T[2, 3])
            descent = start_z - current_z
            print(
                f"step={step_idx:02d} | current_z={current_z:.6f} m | "
                f"descent={descent:.6f} m ({descent*1000:.2f} mm)"
            )

        final_z = float(robot.getState().T[2, 3])
        total_descent = start_z - final_z
        print("\n=== FINAL VALUES ===")
        print(f"start_z       = {start_z:.6f} m")
        print(f"final_z       = {final_z:.6f} m")
        print(f"total_descent = {total_descent:.6f} m ({total_descent*1000:.2f} mm)")
        print("Send me these values and I will set the Z offset in the shear experiment.")
    finally:
        if hasattr(robot, "stop"):
            try:
                robot.stop()
            except Exception:
                pass


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
Shear test on points 3..8 with fixed displacements (no data save).

Requirements implemented:
- Use only points: 3,4,5,6,7,8 (starts directly at 3)
- 4 directions per point: x+, x-, y+, y-
- One sequence per direction
- Fixed displacement: XY = 1mm, Z approach = 1mm
- Manual confirmations before point 3 and before point 7 (i.e. after point 6)
- No magnetic sensor serial connection (to avoid port conflicts)
"""

from pathlib import Path

import numpy as np

import franka_skin_test_shear_forces as base
from franka_controller import config


POINT_SEQUENCE = ["3", "4", "5", "6", "7", "8"]
XY_DISPLACEMENT_M = 0.001  # 1mm
Z_APPROACH_M = 0.00375  # 2.75mm to touch + 1.00mm press
CONFIRM_BEFORE_POINTS = {"3", "5", "7"}


def _save_disabled(output_file, sequences_by_point, stretch_value, stretch_label):
    total_points = len(sequences_by_point)
    total_sequences = sum(len(v) for v in sequences_by_point.values())
    print("\n[NO SAVE MODE] Data recording disabled.")
    print(f"[NO SAVE MODE] Points processed: {total_points}")
    print(f"[NO SAVE MODE] Sequences executed: {total_sequences}")
    print(f"[NO SAVE MODE] Skipped writing: {output_file}")
    config.LAST_OUTPUT_FILE = None


class _NoMagReader:
    """Drop-in replacement that never opens the magnetic serial port."""

    def __init__(self, *args, **kwargs):
        self.running = False
        self.daemon = True

    def start(self):
        self.running = True
        try:
            base.stretchmagtec_ready_event.set()
        except Exception:
            pass

    def join(self, timeout=None):
        self.running = False

    def get_latest_data(self):
        return np.full((config.STRETCHMAGTEC_SENSORS, config.STRETCHMAGTEC_CHANNELS), 2.0, dtype=float)


def _dummy_stretch_data():
    return np.full((config.STRETCHMAGTEC_SENSORS, config.STRETCHMAGTEC_CHANNELS), 2.0, dtype=float)


def _press_fixed_z(r, ft_thread, target_fz, initial_z=None, max_iterations=500):
    """
    Replace force-based press with fixed Z approach.
    Moves exactly Z_APPROACH_M down from the provided initial_z.
    """
    current_state = r.getState()
    if initial_z is None:
        initial_z = float(current_state.T[2, 3])
    target_pose = current_state.T.copy()
    target_pose[2, 3] = initial_z - Z_APPROACH_M
    base.safe_robot_move(r, "absolute", target_pose, duration=0.8)
    return True


_ORIG_COLLECT = base.collect_shear_data_for_point


def _collect_with_manual_point_gates(
    r,
    ft_thread,
    stretchmagtec_reader,
    ft_calibration,
    stretchmagtec_calibration,
    target_pos,
    point_name,
    stretch_value,
    stretch_label,
):
    if str(point_name) in CONFIRM_BEFORE_POINTS:
        input(f"\nPress Enter to start point {point_name}...")
    return _ORIG_COLLECT(
        r,
        ft_thread,
        stretchmagtec_reader,
        ft_calibration,
        stretchmagtec_calibration,
        target_pos,
        point_name,
        stretch_value,
        stretch_label,
    )


def main():
    # Core behavior overrides.
    base.TARGET_OFFSETS = POINT_SEQUENCE
    config.SELECTED_OFFSETS = POINT_SEQUENCE
    base.MOVEMENTS_PER_DIRECTION = 1
    base.SHEAR_DISPLACEMENT = XY_DISPLACEMENT_M
    base.press_to_fz = _press_fixed_z
    base.collect_shear_data_for_point = _collect_with_manual_point_gates

    # Runtime mode.
    base.ENABLE_GUI = False
    base.PROMPT_FOR_STRETCH = False
    base.STRETCH_LEVELS = [0.0]
    base.save_shear_data_to_h5 = _save_disabled

    # Disable magnet serial usage in this script.
    base.StretchMagTecSerialReader = _NoMagReader
    base.read_stretchmagtec_data = _dummy_stretch_data
    if hasattr(base, "skin_test_module"):
        base.skin_test_module.read_stretchmagtec_data = _dummy_stretch_data

    # Disable point delays/confirm logic from prior wrappers (we handle it above).
    config.CONFIRM_BETWEEN_POINTS = False
    config.CONFIRM_BEFORE_FIRST_POINT = False
    config.AUTO_START_FIRST_N_POINTS = 0
    config.DELAY_BEFORE_POINT_START_SEC = {}
    config.DELAY_EVERY_N_POINTS = 0
    config.DELAY_EVERY_N_SECONDS = 0.0

    # Labels.
    config.CURRENT_RUN_LABEL = "shear_3_8_fixed_1mm_no_save"
    config.CURRENT_STRETCH_LABEL = "000pct"
    config.CURRENT_STRETCH_VALUE = 0.0
    config.DATA_DIR = str(Path(config.DATA_DIR))

    print("=== SHEAR TEST POINTS 3..8 (XY=1mm, Z=1mm, NO SAVE) ===")
    print(f"Point sequence: {POINT_SEQUENCE}")
    print("Manual gates before points: 3, 5, 7")
    base.main()


if __name__ == "__main__":
    main()

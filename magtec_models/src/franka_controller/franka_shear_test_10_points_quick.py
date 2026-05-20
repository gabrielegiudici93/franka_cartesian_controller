#!/usr/bin/env python3
"""
Quick shear test derived directly from franka_skin_test_shear_forces.py.

Changes vs working baseline:
1) One movement per direction (x+, x-, y+, y-) on each point.
2) No data recording to disk.
"""

from pathlib import Path

import numpy as np
import franka_skin_test_shear_forces as base
from franka_controller import config


def _save_disabled(output_file, sequences_by_point, stretch_value, stretch_label):
    total_points = len(sequences_by_point)
    total_sequences = sum(len(v) for v in sequences_by_point.values())
    print("\n[NO SAVE MODE] Data recording disabled by quick script.")
    print(f"[NO SAVE MODE] Points processed: {total_points}")
    print(f"[NO SAVE MODE] Sequences executed: {total_sequences}")
    print(f"[NO SAVE MODE] Skipped writing: {output_file}")
    config.LAST_OUTPUT_FILE = None


class _NoMagReader:
    """
    Drop-in replacement for StretchMagTecSerialReader that never opens serial.
    Provides deterministic dummy data so base checks pass without sensor lock.
    """

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


def main():
    # Keep behavior aligned with the proven script, only overriding requested knobs.
    base.MOVEMENTS_PER_DIRECTION = 1
    base.ENABLE_GUI = False
    base.PROMPT_FOR_STRETCH = False
    base.STRETCH_LEVELS = [0.0]
    base.save_shear_data_to_h5 = _save_disabled
    base.StretchMagTecSerialReader = _NoMagReader
    base.read_stretchmagtec_data = _dummy_stretch_data
    if hasattr(base, "skin_test_module"):
        base.skin_test_module.read_stretchmagtec_data = _dummy_stretch_data

    # Keep explicit run labels so logs are clear this is no-save execution.
    config.CURRENT_RUN_LABEL = "quick_shear_no_save"
    config.CURRENT_STRETCH_LABEL = "000pct"
    config.CURRENT_STRETCH_VALUE = 0.0
    config.DATA_DIR = str(Path(config.DATA_DIR))
    config.CONFIRM_BETWEEN_POINTS = False
    config.CONFIRM_BEFORE_FIRST_POINT = True
    config.AUTO_START_FIRST_N_POINTS = 0
    config.DELAY_BEFORE_POINT_START_SEC = {}
    config.DELAY_EVERY_N_POINTS = 2
    config.DELAY_EVERY_N_SECONDS = 20.0

    print("=== QUICK SHEAR (BASE LOGIC, NO SAVE, 1x DIRECTION) ===")
    base.main()


if __name__ == "__main__":
    main()

# MagTec Models

Data collection, real-time visualization, and training for the **15-taxel magnetic skin**, using the Franka controller from the parent repo (`pyfranka_interface`).

**Prerequisites:** complete the Franka setup in the [top-level README](../README.md).

---

## Setup (every new terminal)

```bash
conda activate franka_interface
export LD_LIBRARY_PATH=$HOME/franka_cartesian_controller/pyfranka_interface/third_party/libfranka/lib:$LD_LIBRARY_PATH
cd ~/franka_cartesian_controller/magtec_models
```

One-time:

```bash
cp config/hardware.example.yaml config/hardware.yaml
# edit ROBOT_IP, ports, initial_joints
pip install -r ../requirements-magtec.txt
python3 src/franka_controller/config.py
```

---

## Python commands (all examples)

Run from `magtec_models/` after the setup block above.

| # | Task | Command |
|---|------|---------|
| 01 | 15-taxel live GUI | `python3 src/validation_tests/15_taxels_visualization.py` |
| 02 | Liquid-magnet GUI | `python3 src/validation_tests/liquid_magnet_15_taxels_sensorreader.py` |
| 03 | No-touch baseline | `python3 src/franka_controller/collect_no_touch_data.py --stretch 0 10 20 --data-dir data/Multiple_Points --run-label no_touch_$(date +%Y%m%d_%H%M%S)` |
| 04 | Press 10 points (FT, no skin) | `python3 src/franka_controller/franka_10_random_points.py` |
| 05 | Multi-point collection | `python3 src/franka_controller/franka_skin_test_multiple_points.py` |
| 06 | Shear quick (no save) | `python3 src/franka_controller/franka_shear_test_10_points_quick.py` |
| 07 | Shear points 3–8 | `python3 src/franka_controller/franka_shear_test_points_3_8_fixed_1mm.py` |
| 08 | Train models | `python3 src/training/train_best_models.py --normal-dir data/Multiple_Points/YOUR_RUN --run-label collab_demo_models --remove-outliers` |
| **09** | **Robot only (no skin, no FT)** | `python3 src/franka_controller/franka_motion_test_no_sensors.py` |

### Example 09 options (no sensors)

```bash
# Point 1 only (safest)
python3 src/franka_controller/franka_motion_test_no_sensors.py

# Points 1–3, 0.5 mm dip, no Enter prompts
python3 src/franka_controller/franka_motion_test_no_sensors.py --points 1 2 3 --indent-mm 0.5 --no-prompt

# All 10 grid points
python3 src/franka_controller/franka_motion_test_no_sensors.py --all
```

### Robot utilities

```bash
python3 src/franka_controller/get_current_joints.py
python3 ~/franka_cartesian_controller/scripts/test_robot.py
```

---

## Hardware matrix

| # | Robot | FT | Skin |
|---|:-----:|:--:|:----:|
| 01–03 | — | — | ✔ |
| 04, 06, 07 | ✔ | ✔ | — |
| 05 | ✔ | ✔ | ✔ |
| 08 | — | — | — |
| **09** | ✔ | — | — |

**No magnetic skin?** Use **09**. **FT but no skin?** Use **04 / 06 / 07**.

Only **one** process at a time may use the skin serial port. Robot examples need FCI active — [../docs/ROBOT_CONNECTION.md](../docs/ROBOT_CONNECTION.md).

Per-example notes: `examples/<NN_*>/README.md`.

---

## Data collection: `franka_skin_test.py` (core engine)

**Yes — this file is in the repo:** `src/franka_controller/franka_skin_test.py`

It is the **shared engine** for all robot-based skin data collection. You normally do **not** run it directly; higher-level scripts configure the grid, points, and stretch levels, then call it (via `runpy`) with the right settings from `config.py` / `hardware.yaml`.

### What it does

- Connects to the **Franka** (`pyfranka_interface`), **FT sensor**, and **StretchMagTec** magnetic skin
- Moves the robot through a grid of press positions (9 offsets per location: center, N/S/E/W, corners)
- Logs FT forces, magnetic channels, and end-effector pose at ~100 Hz into **HDF5** under `data/Multiple_Points/`
- Handles calibration, stream stabilization, and press sequencing (normal force on Fz)

### How other scripts use it

| Script / example | Role relative to `franka_skin_test.py` |
|------------------|----------------------------------------|
| **`franka_skin_test_multiple_points.py`** (ex. **05**) | Multi-point normal-force collection; sets point grid and stretch levels, then **runs the engine** |
| **`franka_skin_test_single_point.py`** | Single-location variant; same engine, one target pose |
| **`franka_skin_test_shear_forces.py`** (ex. **06** full save) | Shear protocol (Fx/Fy holds); **reuses sensor classes and HDF5 layout** from the engine |
| **`franka_shear_test_10_points_quick.py`** (ex. **06** quick) | Thin wrapper over shear script — no HDF5 save |
| **`franka_shear_test_points_3_8_fixed_1mm.py`** (ex. **07**) | Shear on taxels 3–8; imports shear module |
| **`collect_no_touch_data.py`** (ex. **03**) | **Independent** — magnetic skin only, no robot |
| **`franka_10_random_points.py`** (ex. **04**) | **Independent** — robot + FT only (no skin serial) |
| **`franka_motion_test_no_sensors.py`** (ex. **09**) | **Independent** — robot motion only, no FT/skin |

Direct launch (advanced):

```bash
python3 src/franka_controller/franka_skin_test.py
```

Recommended for full datasets: **example 05** → `franka_skin_test_multiple_points.py` → `franka_skin_test.py`.

---

## Core scripts (reference)

### Data collection
- `src/franka_controller/franka_skin_test.py` — **core engine** (see above)
- `src/franka_controller/franka_skin_test_multiple_points.py` — multi-point wrapper → engine
- `src/franka_controller/franka_skin_test_single_point.py` — single-point wrapper → engine
- `src/franka_controller/franka_skin_test_shear_forces.py` — shear collection
- `src/franka_controller/collect_no_touch_data.py` — baseline without robot
- `src/franka_controller/franka_10_random_points.py` — 10-point press, FT only
- `src/franka_controller/franka_motion_test_no_sensors.py` — robot grid, no sensors

### Visualization
- `src/validation_tests/15_taxels_visualization.py` — live taxel GUI
- `src/validation_tests/15_taxels_visualization_plus.py` — + XYZ plots + MP4
- `src/validation_tests/liquid_magnet_15_taxels_sensorreader.py` — hybrid GUI
- `src/validation_tests/10_points_real_time_predictor.py` — 10-point grid + ML

### Training
- `src/training/train_best_models.py` — main trainer
- `src/training/clean_sequences.py` — preprocess HDF5
- `src/training/inspect_h5_files.py` — debug HDF5 structure

Further reading: [docs/guides/OVERVIEW.md](docs/guides/OVERVIEW.md)

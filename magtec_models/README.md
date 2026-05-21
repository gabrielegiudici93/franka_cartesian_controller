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

## Core scripts

- `src/franka_controller/franka_motion_test_no_sensors.py` — robot grid, no sensors
- `src/franka_controller/franka_10_random_points.py` — 10-point press, FT only
- `src/franka_controller/franka_skin_test_multiple_points.py` — full collection
- `src/validation_tests/15_taxels_visualization.py` — live taxel GUI
- `src/training/train_best_models.py` — training

Further reading: [docs/guides/OVERVIEW.md](docs/guides/OVERVIEW.md)

# Data Collection Guide

Step-by-step instructions for starting a **new** multi-point dataset with the Franka robot, FT sensor, and 15-taxel magnetic skin.

---

## Which script should I run?

| Goal | Script to run | Example # |
|------|---------------|-----------|
| **Full multi-point dataset (normal forces)** — *most common* | `franka_skin_test_multiple_points.py` | 05 |
| Core engine (advanced; usually called by the wrapper above) | `franka_skin_test.py` | — |
| Single target location only | `franka_skin_test_single_point.py` | — |
| Shear forces (Fx/Fy protocol) | `franka_skin_test_shear_forces.py` | 06 (full) |
| Magnetic baseline only (no robot) | `collect_no_touch_data.py` | 03 |
| Robot + FT grid check (no skin serial) | `franka_10_random_points.py` | 04 |
| Manual keyboard teleop (pose finding) | `teleop_franka_keyboard.py` | 10 |

**For a new standard dataset, use example 05:**

```bash
python3 src/franka_controller/franka_skin_test_multiple_points.py
```

### How the scripts connect

```
franka_skin_test_multiple_points.py   ← you run this (configures grid, stretch, run name)
        │
        └── calls franka_skin_test.py  ← core engine (robot moves, logs FT + skin → HDF5)
```

The wrapper sets `config.py` values (output path, offsets, press profile), then runs the engine once per stretch level. A live GUI may open during collection (`ENABLE_GUI = True` in the wrapper).

---

## Before you start (checklist)

- [ ] Franka controller installed — [top-level INSTALL.md](../../../docs/INSTALL.md)
- [ ] Robot reachable, FCI active, robot unlocked — [ROBOT_CONNECTION.md](../../../docs/ROBOT_CONNECTION.md)
- [ ] `config/hardware.yaml` created from `config/hardware.example.yaml` (`ROBOT_IP`, `FT_PORT`, `STRETCHMAGTEC_PORT`, reference pose, initial joints)
- [ ] Config sanity check: `python3 src/franka_controller/config.py`
- [ ] Magnetic skin responds in visualization (example 01) — **close visualization before collection**
- [ ] No other process using the skin serial port (`jobs -l`, kill stale Python if needed)
- [ ] Workspace clear; emergency stop reachable

---

## New data collection — step by step

### 1. Open a terminal and activate the environment

```bash
conda activate franka_interface
export LD_LIBRARY_PATH=$HOME/franka_cartesian_controller/pyfranka_interface/third_party/libfranka/lib:$LD_LIBRARY_PATH
cd ~/franka_cartesian_controller/magtec_models
```

Pull the latest repo (bug fixes and docs):

```bash
cd ~/franka_cartesian_controller
git pull --rebase origin main
cd magtec_models
```

### 2. Configure hardware (once per bench)

```bash
cp config/hardware.example.yaml config/hardware.yaml
# Edit: ROBOT_IP, FT_PORT, STRETCHMAGTEC_PORT, REFERENCE_POSITION, INITIAL_JOINT_POSITIONS
python3 src/franka_controller/config.py
```

### 3. Configure the collection run (edit the wrapper script)

Open:

`src/franka_controller/franka_skin_test_multiple_points.py`

Key settings at the top of the file:

| Variable | Meaning | Example |
|----------|---------|---------|
| `TARGET_POSITION_COORDS` | Center XY(Z) of the grid in meters | `[0.500781, 0.419620, 0.032311]` |
| `INITIAL_JOINT_POSITIONS` | Home joints between stretch runs | from `get_current_joints.py` |
| `STRETCH_LEVELS` | Skin stretch fractions (0.10 = 10%) | `[0.0, 0.10, 0.20]` for three levels |
| `PROMPT_FOR_STRETCH` | Wait for Enter before each stretch | `True` |
| `PRESS_DEPTH_MM` | Normal press depth | `2.5` |
| `PRESSES_PER_POINT` | Press cycles per grid offset | `33` (reduce for quick tests) |
| `ENABLE_GUI` | Live GUI during collection | `True` (set `False` if GUI causes issues) |
| `ENABLE_EXPLORATION` | Dry-run point exploration first | `False` for normal runs |

To log current joint positions into the script:

```bash
python3 src/franka_controller/get_current_joints.py
```

### 4. Start collection

```bash
python3 src/franka_controller/franka_skin_test_multiple_points.py
```

**Interactive prompts:**

1. **Run label** — unique name for this session (e.g. `2.5mm_test_may26`). Data goes under `data/Multiple_Points/<run_label>/`.
2. **Stretch level** — for each value in `STRETCH_LEVELS`, set the skin stretch manually, then press **Enter**.
3. Robot runs the full press sequence (no-touch baseline, then points 1–10 with 9 offsets each, depending on config).

Collection can take a long time. Do not close the terminal until you see completion messages.

### 5. Verify that data was actually saved

**Important:** a message like “stretch levels processed” does **not** guarantee HDF5 files were written. Always check:

```bash
ls -lh data/Multiple_Points/<your_run_label>/
```

You should see one or more `.h5` files (names depend on stretch labels), for example:

```
<run_label>_stretch_010pct.h5
```

If the folder is **empty**:

- Scroll the terminal for errors during `franka_skin_test` (robot, FT, or serial failures).
- Try again with `ENABLE_GUI = False` in the wrapper (avoids post-collection GUI issues).
- Confirm `hardware.yaml` ports and robot IP.
- Re-run example 01 to confirm the skin serial works.

Inspect file structure:

```bash
python3 src/training/inspect_h5_files.py data/Multiple_Points/<your_run_label>/
```

### 6. Optional — train models

After you have valid `.h5` files:

```bash
python3 src/training/train_best_models.py \
  --normal-dir data/Multiple_Points/<your_run_label> \
  --run-label <model_output_name> \
  --remove-outliers
```

See example 08 README and [OVERVIEW.md](OVERVIEW.md).

---

## Output layout

```
magtec_models/data/Multiple_Points/
└── <run_label>/                    ← you choose this name at startup
    ├── <prefix>_stretch_000pct.h5  ← one file per stretch level (names vary)
    ├── <prefix>_stretch_010pct.h5
    └── ...
```

Large `.h5` files are usually gitignored; back them up on your lab storage or Git LFS if you version them.

---

## Troubleshooting

| Symptom | What to do |
|---------|------------|
| `ModuleNotFoundError: validation_tests.real_time_predictor` | `git pull` — repo includes a compatibility shim |
| `AttributeError: contact_label` on GUI after collection | `git pull` — fixed in recent commits; or set `ENABLE_GUI = False` |
| Empty output folder | Engine exited early; read full terminal log; check robot/FT/skin |
| `SerialException` / stuck calibration | Only one process on skin port; kill other Python jobs |
| `Combined model files not found` | **Warning only** during collection — ML models in `models/` are optional for saving HDF5 |
| Robot does not move | FCI, unlock, `ROBOT_IP` in `hardware.yaml` |

---

## Quick reference — copy/paste

```bash
conda activate franka_interface
export LD_LIBRARY_PATH=$HOME/franka_cartesian_controller/pyfranka_interface/third_party/libfranka/lib:$LD_LIBRARY_PATH
cd ~/franka_cartesian_controller/magtec_models

# optional: edit STRETCH_LEVELS and coords in:
#   src/franka_controller/franka_skin_test_multiple_points.py

python3 src/franka_controller/franka_skin_test_multiple_points.py

# after run:
ls -lh data/Multiple_Points/YOUR_RUN_LABEL/
```

---

## Related docs

- [MagTec README](../../README.md)
- [Teleoperation guide](TELEOPERATION.md) — manual jogging before collection
- [Workflow overview](OVERVIEW.md)
- [Example 05 README](../../examples/05_collect_multipoint_data/README.md)
- [Franka robot connection](../../../docs/ROBOT_CONNECTION.md)

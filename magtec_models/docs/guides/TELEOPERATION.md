# Keyboard Teleoperation Guide

Manual control of the Franka end-effector from the keyboard, with optional real-time visualization of the FT sensor and 15-taxel magnetic skin.

**Script:** `src/franka_controller/teleop_franka_keyboard.py`  
**Example folder:** [examples/10_teleop_keyboard](../../examples/10_teleop_keyboard/README.md)

---

## What it does

1. Starts FT and StretchMagTec reader threads (same stack as data collection).
2. Runs initial sensor calibration (unless disabled in `franka_skin_test` / config).
3. Moves the robot to a configured grid pose (`SELECTED_POSITION_ID` + offset).
4. Lets you jog the tool in Cartesian space with the keyboard.
5. Optionally shows a live GUI (`ENABLE_GUI = True`) with FT and taxel readings while you move.

This is useful for **finding press poses**, checking contact, and debugging before a full `franka_skin_test_multiple_points.py` run.

---

## Prerequisites

- Franka controller built and `franka_interface` conda env active
- [hardware.yaml](../../config/hardware.example.yaml) → `config/hardware.yaml` with correct `ROBOT_IP`, ports, and grid reference
- Robot unlocked, FCI enabled — [ROBOT_CONNECTION.md](../../../docs/ROBOT_CONNECTION.md)
- Run in an **interactive terminal** (keyboard raw mode does not work in background jobs)
- Close other programs using the skin serial port (visualization, collection, etc.)

---

## Run

```bash
conda activate franka_interface
export LD_LIBRARY_PATH=$HOME/franka_cartesian_controller/pyfranka_interface/third_party/libfranka/lib:$LD_LIBRARY_PATH
cd ~/franka_cartesian_controller/magtec_models

python3 src/franka_controller/teleop_franka_keyboard.py
```

After calibration, the script prints the key map. Press any key to start jogging.

---

## Configuration

Edit constants at the top of `teleop_franka_keyboard.py`:

| Variable | Description |
|----------|-------------|
| `SELECTED_POSITION_ID` | Key in `MAIN_GRID_POSITIONS` from `config.py` |
| `SELECTED_OFFSET` | Offset name: `center`, `n`, `s`, `e`, `w`, `ne`, … |
| `ENABLE_GUI` | `True` = sensor GUI during teleop; `False` = keyboard only |
| `STEP_SIZE` | Initial jog step in metres (default `0.001`) |

To update the grid center for position 32, either edit `config.py` / `hardware.yaml` reference pose, or the override block in the teleop script.

Get current joint positions for collection scripts:

```bash
python3 src/franka_controller/get_current_joints.py
```

---

## Keyboard controls

| Key | Action |
|-----|--------|
| `8` | Move −X |
| `2` | Move +X |
| `4` | Move −Y |
| `6` | Move +Y |
| `-` | Move −Z |
| `+` | Move +Z |
| `*` | Increase step size |
| `/` | Decrease step size (minimum ~0.5 mm) |
| `p` | Print current position and accumulated displacement |
| `0` | Clear accumulated displacement counter |
| `q` or `c` | Quit teleoperation |

Step size changes apply to the next move.

---

## Relation to data collection

| Task | Script |
|------|--------|
| Manual jogging / pose finding | `teleop_franka_keyboard.py` (this guide) |
| Automated multi-point HDF5 collection | `franka_skin_test_multiple_points.py` — [DATA_COLLECTION.md](DATA_COLLECTION.md) |
| Core collection engine | `franka_skin_test.py` |

Teleoperation does **not** save HDF5 datasets. Use it to validate poses, then copy coordinates into `franka_skin_test_multiple_points.py` (`TARGET_POSITION_COORDS`, joints, etc.) before running a collection.

---

## Troubleshooting

| Issue | Fix |
|-------|-----|
| Keys not detected | Run in a local terminal, not SSH without TTY or embedded output panel |
| Calibration timeout | Check `STRETCHMAGTEC_PORT` / `FT_PORT`; close other serial users |
| GUI error on `contact_label` | Update repo (`git pull`); recent commits use optional GUI widgets |
| Robot does not move | FCI, unlock, correct `ROBOT_IP` |

---

## Related

- [MagTec README](../../README.md)
- [Data collection guide](DATA_COLLECTION.md)
- [Workflow overview](OVERVIEW.md)

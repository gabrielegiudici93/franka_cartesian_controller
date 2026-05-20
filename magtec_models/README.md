# MagTec Models (Essential Handoff)

Subset of the full research codebase: data collection, visualization, and training for the 15-taxel magnetic skin.

## Configure hardware (required)

From the repo root, after activating the conda env (`conda activate franka_interface`):

```bash
cd magtec_models
cp config/hardware.example.yaml config/hardware.yaml
# edit ROBOT_IP, FT_PORT, STRETCHMAGTEC_PORT, reference position, initial joints
```

Or use environment variables:

```bash
export robot_ip=192.168.2.10
export ft_port=/dev/ttyUSB0
export stretchmagtec_port=/dev/ttyACM0
```

Check config:

```bash
python3 src/franka_controller/config.py
```

## Examples (recommended entry points)

Each subfolder under `examples/` is self-contained: it holds a `README.md` (what it does, what it needs, expected output) and a `run.sh` (one-line launcher). The numbering reflects the recommended order to follow.

| # | Task | Robot | FT | Skin | Guide |
|---|------|:-----:|:--:|:----:|-------|
| 01 | 15-taxel live GUI                 | — | — | ✔ | [examples/01_visualize_15_taxels/README.md](examples/01_visualize_15_taxels/README.md) |
| 02 | Liquid-magnet style GUI           | — | — | ✔ | [examples/02_visualize_liquid_magnet/README.md](examples/02_visualize_liquid_magnet/README.md) |
| 03 | No-touch baseline (no robot)      | — | — | ✔ | [examples/03_collect_no_touch/README.md](examples/03_collect_no_touch/README.md) |
| 04 | Press 10 points (FT only)         | ✔ | ✔ | — | [examples/04_press_10_points/README.md](examples/04_press_10_points/README.md) |
| 05 | Multi-point data collection       | ✔ | ✔ | ✔ | [examples/05_collect_multipoint_data/README.md](examples/05_collect_multipoint_data/README.md) |
| 06 | Shear test (quick, no save)       | ✔ | ✔ | — | [examples/06_shear_test_quick/README.md](examples/06_shear_test_quick/README.md) |
| 07 | Shear points 3–8 (fixed Z)        | ✔ | ✔ | — | [examples/07_shear_points_3_8/README.md](examples/07_shear_points_3_8/README.md) |
| 08 | Train models                      | — | — | — | [examples/08_train_models/README.md](examples/08_train_models/README.md) |

### How to run any example

Always launch from this directory (`magtec_models/`). `run.sh` does the `cd` for you:

```bash
conda activate franka_interface
export LD_LIBRARY_PATH=$PWD/../pyfranka_interface/third_party/libfranka/lib:$LD_LIBRARY_PATH

cat examples/01_visualize_15_taxels/README.md     # read first
./examples/01_visualize_15_taxels/run.sh          # then launch
```

Only **one** process at a time may open the magnetic serial port — stop the visualization before launching a collection script (or vice versa).

## Core scripts (working code)

### Data collection
- `src/franka_controller/franka_skin_test.py` — engine
- `src/franka_controller/franka_skin_test_multiple_points.py` — normal forces, multi-point
- `src/franka_controller/franka_skin_test_shear_forces.py` — shear collection
- `src/franka_controller/collect_no_touch_data.py` — baseline without robot

### Visualization
- `src/validation_tests/15_taxels_visualization.py`
- `src/validation_tests/15_taxels_visualization_plus.py` — + XYZ plots + MP4 record
- `src/validation_tests/liquid_magnet_15_taxels_sensorreader.py` — hybrid GUI
- `src/validation_tests/10_points_real_time_predictor.py` — 10-point grid + ML

### Training
- `src/training/train_best_models.py` — main trainer
- `src/training/clean_sequences.py` — preprocess HDF5
- `src/training/inspect_h5_files.py` — debug HDF5 structure

## Data layout

```
data/Multiple_Points/<run_label>/
  ├── <run>_000pct.h5
  ├── <run>_010pct.h5
  └── ...
models/<run_label>/...
plots/...
```

## Notes for collaborators

- Only **one** process should open the magnetic serial port at a time
- Press scripts that use the robot but not the magnet: `franka_shear_test_10_points_quick.py`, `franka_shear_test_points_3_8_fixed_1mm.py`
- Full guides: [docs/guides/OVERVIEW.md](docs/guides/OVERVIEW.md)

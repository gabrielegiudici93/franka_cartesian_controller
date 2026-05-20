# MagTec Models

Data collection, real-time visualization, and training for the **15-taxel magnetic skin**, using the Franka controller from the parent repo (`pyfranka_interface`).

**Prerequisites:** complete the Franka setup in the [top-level README](../README.md) (`conda activate franka_interface`, build `pyfranka_interface`).

## Repository layout (this folder)

```
magtec_models/
├── README.md                 # this file
├── config/
│   ├── hardware.example.yaml
│   └── hardware.yaml         # you create this (gitignored)
├── examples/                 # 8 runnable demos — each has README.md + run.sh
│   ├── 01_visualize_15_taxels/
│   ├── 02_visualize_liquid_magnet/
│   ├── …
│   └── 08_train_models/
├── docs/guides/OVERVIEW.md   # recommended pipeline
├── src/                      # core scripts
└── data/, models/, plots/, logs/   # outputs (empty in the repo)
```

## Configure hardware (required)

From the repo root, with the conda env active:

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

Install MagTec Python deps (once):

```bash
pip install -r ../requirements-magtec.txt
```

## Examples

Each subfolder under `examples/` is self-contained:

```
examples/<NN_name>/
├── README.md   # what it does, prerequisites, expected output
└── run.sh      # launcher (cd's to magtec_models/ and runs the script)
```

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

Recommended order: **01 → 03 → 05 → 08** (visualize → baseline → collect → train). Use **04 / 06 / 07** to validate the robot without the magnetic port.

## How to run an example

Always work from `magtec_models/` (or use `run.sh`, which sets the directory for you):

```bash
conda activate franka_interface
export LD_LIBRARY_PATH=$PWD/../pyfranka_interface/third_party/libfranka/lib:$LD_LIBRARY_PATH
cd magtec_models

cat examples/01_visualize_15_taxels/README.md
./examples/01_visualize_15_taxels/run.sh
```

**Rules**

- Only **one** process at a time may open the magnetic serial port — stop visualization before collection (see [docs/guides/OVERVIEW.md](docs/guides/OVERVIEW.md)).
- Robot examples (04–07): robot unlocked, FCI enabled — [../docs/ROBOT_CONNECTION.md](../docs/ROBOT_CONNECTION.md).

## First-time walk-through

1. **Visualize** — confirm sensors and calibration:
   ```bash
   ./examples/01_visualize_15_taxels/run.sh
   ```
2. **No-touch baseline** (no robot):
   ```bash
   ./examples/03_collect_no_touch/run.sh
   ```
3. **Robot dry-run** (FT only, no skin serial):
   ```bash
   ./examples/04_press_10_points/run.sh
   ```
4. **Full collection** (robot + FT + skin):
   ```bash
   ./examples/05_collect_multipoint_data/run.sh
   ```
5. **Train** — edit `examples/08_train_models/run.sh` (`NORMAL_DIR`, `RUN_LABEL`), then:
   ```bash
   ./examples/08_train_models/run.sh
   ```

## Core scripts

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

## Further reading

- Pipeline and hardware matrix: [docs/guides/OVERVIEW.md](docs/guides/OVERVIEW.md)
- Franka install and robot connection: [../README.md](../README.md), [../docs/ROBOT_CONNECTION.md](../docs/ROBOT_CONNECTION.md)

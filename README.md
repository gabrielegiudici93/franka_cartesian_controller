# Franka Cartesian Controller

A self-contained workspace for controlling the **Franka Research 3 / Panda** in Cartesian (and joint) space from **Python and C++**, plus a tactile-sensing project (**MagTec skin**) that builds on top of it.

The Python library `pyfranka_interface` is vendored here so collaborators can install everything from this single repository, even if the upstream source is unavailable.

## Repository layout

```
franka_cartesian_controller/
├── README.md                 # this file
├── LICENSE
├── AUTHORS.md
├── environment.yml           # conda env (Python + build deps)
├── requirements-magtec.txt   # MagTec stack (sklearn, h5py, ...)
├── docs/
│   ├── INSTALL.md            # full setup: conda + libfranka + build
│   ├── ROBOT_CONNECTION.md   # network + FCI
│   ├── USAGE_PYTHON.md       # Python API examples
│   ├── USAGE_CPP.md          # C++ API examples
│   └── SETUP_GITHUB.md
├── pyfranka_interface/       # vendored controller library (source + waf + setup.py)
│   ├── README.md             # original upstream docs
│   ├── setup.py
│   ├── waf, run_build.sh
│   ├── environment.yml
│   ├── src/                  # python.cpp + cartesian_franka + examples
│   └── third_party/          # vendored libfranka 0.9.2 + pybind11
└── magtec_models/            # tactile skin project (subfolder)
    ├── README.md             # MagTec entry point — read this first
    ├── config/               # hardware.example.yaml (copy to hardware.yaml)
    ├── src/                  # core working code
    ├── examples/             # 8 runnable examples, each with its own README
    ├── docs/guides/          # OVERVIEW.md = recommended pipeline
    └── data/, models/, plots/, logs/   # outputs (kept empty in the repo)
```

## Quick start (install)

```bash
git clone <YOUR_REPO_URL> franka_cartesian_controller
cd franka_cartesian_controller

# 1. Conda env (Python 3.9 + build tools)
conda env create -f environment.yml
conda activate franka_interface

# 2. Build & install pyfranka_interface (see docs/INSTALL.md for prerequisites)
cd pyfranka_interface
./run_build.sh
export LD_LIBRARY_PATH=$PWD/third_party/libfranka/lib:$LD_LIBRARY_PATH
cd ..

# 3. Verify
python3 -c "import pyfranka_interface as franka; print('OK')"

# 4. (Optional) MagTec stack
cd magtec_models
pip install -r ../requirements-magtec.txt
cp config/hardware.example.yaml config/hardware.yaml
# edit ROBOT_IP, ports, etc.
python3 src/franka_controller/config.py   # sanity check
```

Full instructions, including libfranka and real-time kernel notes: [docs/INSTALL.md](docs/INSTALL.md).

## How the example folders are organized

All runnable demos live under [`magtec_models/examples/`](magtec_models/examples/). Each example is a self-contained folder with:

```
magtec_models/examples/<NN_name>/
├── README.md   # what it does, prerequisites, how to run, expected output
└── run.sh      # one-liner launcher (sets cwd and forwards args)
```

The numbering reflects the **recommended order** to follow when learning the stack:

| # | Folder | What it does | Needs robot | Needs FT | Needs skin |
|---|--------|--------------|:-----------:|:--------:|:----------:|
| 01 | `01_visualize_15_taxels`       | Live 3×5 taxel GUI (Fx/Fy/Fz)            |  —  |  —  | ✔ |
| 02 | `02_visualize_liquid_magnet`   | Alternative GUI for the liquid-magnet skin |  —  |  —  | ✔ |
| 03 | `03_collect_no_touch`          | Baseline HDF5 without the robot          |  —  |  —  | ✔ |
| 04 | `04_press_10_points`           | Robot presses 10 points to ~3 N (FT only) | ✔  | ✔  |  —  |
| 05 | `05_collect_multipoint_data`   | Full normal-force dataset (robot+FT+skin) | ✔  | ✔  | ✔ |
| 06 | `06_shear_test_quick`          | Quick shear motion (no save)             | ✔  | ✔  |  —  |
| 07 | `07_shear_points_3_8`          | Shear on taxels 3–8 with fixed Z         | ✔  | ✔  |  —  |
| 08 | `08_train_models`              | Train sklearn models from collected HDF5 |  —  |  —  |  —  |

A typical workflow is: **01 → 03 → 05 → 08** (visualize → baseline → collect normals → train), with **04 / 06 / 07** as robot-only validators when the skin is not connected.

See the pipeline matrix in [`magtec_models/docs/guides/OVERVIEW.md`](magtec_models/docs/guides/OVERVIEW.md) for the full overview.

## How to run an example

The pattern is always the same. From the repo root:

```bash
# 1) Activate the env (every new shell)
conda activate franka_interface
export LD_LIBRARY_PATH=$PWD/pyfranka_interface/third_party/libfranka/lib:$LD_LIBRARY_PATH

# 2) Move into the MagTec subproject
cd magtec_models

# 3) Read the example's README to learn what it needs
cat examples/01_visualize_15_taxels/README.md

# 4) Launch via the provided run.sh (recommended)
./examples/01_visualize_15_taxels/run.sh

# … or call the underlying Python script directly (see each README)
python3 src/validation_tests/15_taxels_visualization.py
```

**Important rules of the road**

- Always launch from the `magtec_models/` directory (or via `run.sh`, which `cd`s for you). Relative paths in the code assume that working directory.
- Only **one** process at a time may open the magnetic serial port. If a visualization is running, stop it before launching a collection script (or vice versa). See "Port conflicts" in [OVERVIEW.md](magtec_models/docs/guides/OVERVIEW.md).
- Edit `magtec_models/config/hardware.yaml` (copied from `hardware.example.yaml`) so `ROBOT_IP`, `FT_PORT`, `STRETCHMAGTEC_PORT` and the reference pose match your bench. No defaults are baked in.
- Before running anything that moves the robot (examples 04–07), make sure the FCI is enabled and the robot is unlocked — see [docs/ROBOT_CONNECTION.md](docs/ROBOT_CONNECTION.md).

## A complete walk-through (first-time user)

1. **Visualize the skin** to confirm hardware:
   ```bash
   cd magtec_models
   ./examples/01_visualize_15_taxels/run.sh
   ```
   Calibrate when prompted; check that taxels respond to a gentle touch.
2. **Collect a no-touch baseline** (no robot needed):
   ```bash
   ./examples/03_collect_no_touch/run.sh
   ```
   Produces an HDF5 under `data/`.
3. **Connect the robot** (see [docs/ROBOT_CONNECTION.md](docs/ROBOT_CONNECTION.md)) and dry-run the press grid without the skin:
   ```bash
   ./examples/04_press_10_points/run.sh
   ```
4. **Collect a full dataset** (robot + FT + skin):
   ```bash
   ./examples/05_collect_multipoint_data/run.sh
   ```
   Follow the on-screen prompts for stretch levels (0% / 10% / 20%).
5. **Train models** on the collected HDF5:
   ```bash
   # edit examples/08_train_models/run.sh to set NORMAL_DIR and RUN_LABEL
   ./examples/08_train_models/run.sh
   ```
   Trained `.joblib` artifacts and plots land under `models/` and `plots/`.

## What you get

- `pyfranka_interface`: Python bindings + C++ headers for joint / Cartesian / torque control of the Franka. Includes `Robot_.move`, `move_joints`, `extMove`, custom control callbacks.
- `magtec_models`: data collection, real-time 15-taxel visualization, ML training pipelines for the magnetic skin, plus 8 ready-to-run examples.

## Documentation

| Topic | Where |
|-------|-------|
| Install everything | [docs/INSTALL.md](docs/INSTALL.md) |
| Connect to the robot | [docs/ROBOT_CONNECTION.md](docs/ROBOT_CONNECTION.md) |
| Use the Python API | [docs/USAGE_PYTHON.md](docs/USAGE_PYTHON.md) |
| Use the C++ API | [docs/USAGE_CPP.md](docs/USAGE_CPP.md) |
| Publish to GitHub | [docs/SETUP_GITHUB.md](docs/SETUP_GITHUB.md) |
| MagTec skin project | [magtec_models/README.md](magtec_models/README.md) |
| Recommended MagTec pipeline | [magtec_models/docs/guides/OVERVIEW.md](magtec_models/docs/guides/OVERVIEW.md) |
| Original upstream notes | [pyfranka_interface/README.md](pyfranka_interface/README.md) |

## Authors

- **Gabriele Giudici** — main author / maintainer (workspace, docs, `magtec_models`, current upkeep of vendored `pyfranka_interface`)
- **Valerio Modugno** — co-author (original developer of `pyfranka_interface`)

Full list: [AUTHORS.md](AUTHORS.md).

This repository is intended to work in any lab with a Franka Research 3 / Panda robot — no lab-specific credentials or paths are baked in.

# Franka Cartesian Controller

A self-contained workspace for controlling the **Franka Research 3 / Panda** in Cartesian (and joint) space from **Python and C++**, plus a tactile-sensing project (**MagTec skin**) that builds on top of it.

The Python library `pyfranka_interface` is vendored here so collaborators can install everything from this single repository, even if the upstream source is unavailable.

## Repository layout

```
franka_cartesian_controller/
├── README.md                 # this file
├── LICENSE
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
    ├── README.md
    ├── config/, src/, examples/, docs/
    └── data/, models/, plots/, logs/
```

## Quick start

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

# 3. Verify
python3 -c "import pyfranka_interface as franka; print('OK')"

# 4. (Optional) MagTec stack
cd ../magtec_models
pip install -r ../requirements-magtec.txt
cp config/hardware.example.yaml config/hardware.yaml
# edit ROBOT_IP, ports, etc., then read examples/
```

## What you get

- `pyfranka_interface`: Python bindings + C++ headers for joint / Cartesian / torque control of the Franka. Includes `Robot_.move`, `move_joints`, `extMove`, custom control callbacks.
- `magtec_models`: data collection, real-time 15-taxel visualization, ML training pipelines for the magnetic skin.

## Documentation

| Topic | Where |
|-------|-------|
| Install everything | [docs/INSTALL.md](docs/INSTALL.md) |
| Connect to the robot | [docs/ROBOT_CONNECTION.md](docs/ROBOT_CONNECTION.md) |
| Use the Python API | [docs/USAGE_PYTHON.md](docs/USAGE_PYTHON.md) |
| Use the C++ API | [docs/USAGE_CPP.md](docs/USAGE_CPP.md) |
| Publish to GitHub | [docs/SETUP_GITHUB.md](docs/SETUP_GITHUB.md) |
| MagTec skin project | [magtec_models/README.md](magtec_models/README.md) |
| Original upstream notes | [pyfranka_interface/README.md](pyfranka_interface/README.md) |

## Authors

- **Gabriele Giudici** — main author / maintainer (workspace, docs, `magtec_models`, current upkeep of vendored `pyfranka_interface`)
- **Valerio Modugno** — co-author (original developer of `pyfranka_interface`)

Full list: [AUTHORS.md](AUTHORS.md).

This repository is intended to work in any lab with a Franka Research 3 / Panda robot — no lab-specific credentials or paths are baked in.

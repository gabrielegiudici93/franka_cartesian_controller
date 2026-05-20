# Franka Cartesian Controller

Self-contained workspace for controlling the **Franka Research 3 / Panda** in Cartesian and joint space from **Python and C++**.

The library `pyfranka_interface` is vendored here so you can clone, build, and use the controller from a single repository, even if the original upstream source is unavailable.

## Repository layout

```
franka_cartesian_controller/
├── README.md                 # this file (Franka controller only)
├── LICENSE
├── AUTHORS.md
├── environment.yml           # conda env (Python 3.9 + build deps)
├── docs/
│   ├── INSTALL.md            # full setup: conda + libfranka + build
│   ├── ROBOT_CONNECTION.md   # network + FCI + Desk
│   ├── USAGE_PYTHON.md       # Python API examples
│   ├── USAGE_CPP.md          # C++ API examples
│   └── SETUP_GITHUB.md
├── pyfranka_interface/       # vendored controller (source + waf + setup.py)
│   ├── README.md             # upstream notes
│   ├── run_build.sh
│   ├── src/                  # bindings + cartesian_franka + examples
│   └── third_party/          # libfranka 0.9.2 + pybind11
└── magtec_models/            # optional: tactile skin project (separate README)
```

## Quick start

```bash
git clone <YOUR_REPO_URL> franka_cartesian_controller
cd franka_cartesian_controller

# 1. Conda environment
conda env create -f environment.yml
conda activate franka_interface

# 2. Build pyfranka_interface (system prerequisites: docs/INSTALL.md)
cd pyfranka_interface
./run_build.sh
export LD_LIBRARY_PATH=$PWD/third_party/libfranka/lib:$LD_LIBRARY_PATH
cd ..

# 3. Verify import
python3 -c "import pyfranka_interface as franka; print('OK')"
```

Set your robot IP in code or via environment; see [docs/ROBOT_CONNECTION.md](docs/ROBOT_CONNECTION.md) for network setup and FCI.

## What you get

- **Python** (`pyfranka_interface`): `Robot_` with `move`, `move_joints`, `extMove`, state queries, and custom control callbacks.
- **C++** (`cartesian_franka`): headers and examples under `pyfranka_interface/src/`.
- **Build**: `waf` + vendored `libfranka` 0.9.2 and `pybind11` — no separate clone of upstream repos required.

## Minimal Python example

```python
import pyfranka_interface as franka
import numpy as np

robot = franka.Robot_("172.16.0.2", False, hand_franka=False, auto_init=True, speed_factor=0.1)
state = robot.getState()
print("joints:", state.q)

target = np.eye(4)
target[:3, 3] = [0.500, 0.420, 0.034]
robot.move("absolute", target, 2.0)
```

More examples (relative moves, joint space, callbacks): [docs/USAGE_PYTHON.md](docs/USAGE_PYTHON.md).

## Documentation

| Topic | Guide |
|-------|--------|
| Install (conda, libfranka, build) | [docs/INSTALL.md](docs/INSTALL.md) |
| Connect to the robot | [docs/ROBOT_CONNECTION.md](docs/ROBOT_CONNECTION.md) |
| Python API | [docs/USAGE_PYTHON.md](docs/USAGE_PYTHON.md) |
| C++ API | [docs/USAGE_CPP.md](docs/USAGE_CPP.md) |
| Upstream / build details | [pyfranka_interface/README.md](pyfranka_interface/README.md) |

## MagTec tactile skin (optional subproject)

This repo also ships **`magtec_models/`** — data collection, 15-taxel visualization, and ML training for the magnetic skin. It uses `pyfranka_interface` but is documented separately.

**→ Start here:** [magtec_models/README.md](magtec_models/README.md)  
**→ Workflow overview:** [magtec_models/docs/guides/OVERVIEW.md](magtec_models/docs/guides/OVERVIEW.md)

Install extra Python deps only if you need MagTec: `pip install -r requirements-magtec.txt` (after the Franka env above).

## Authors

- **Gabriele Giudici** — main author / maintainer
- **Valerio Modugno** — co-author (original `pyfranka_interface`)

Full list: [AUTHORS.md](AUTHORS.md).

Intended for any lab with a Franka Research 3 / Panda — no lab-specific credentials or paths are baked in.

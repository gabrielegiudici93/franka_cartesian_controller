# Franka Cartesian Controller

Self-contained workspace for controlling the **Franka Research 3 / Panda** in Cartesian and joint space from **Python and C++**.

The library `pyfranka_interface` is vendored here so you can clone, build, and use the controller from a single repository.

**Repository:** https://github.com/gabrielegiudici93/franka_cartesian_controller

---

## Repository layout

```
franka_cartesian_controller/
├── README.md                 # this tutorial
├── environment.yml           # conda env (Python 3.9)
├── requirements-magtec.txt   # optional MagTec deps only
├── scripts/
│   ├── test_import.py        # software test (no robot)
│   └── test_robot.py         # connect + read joints (robot required)
├── docs/
│   ├── INSTALL.md
│   ├── ROBOT_CONNECTION.md
│   ├── USAGE_PYTHON.md
│   └── USAGE_CPP.md
├── pyfranka_interface/       # Python/C++ bindings + vendored libfranka 0.9.2
└── magtec_models/            # optional tactile skin (separate README)
```

---

## Full install tutorial (Ubuntu 20.04 / 22.04 / 24.04)

### Step 1 — Clone

```bash
cd ~
git clone https://github.com/gabrielegiudici93/franka_cartesian_controller.git
cd franka_cartesian_controller
```

### Step 2 — System packages

```bash
sudo apt update
sudo apt install -y \
    build-essential cmake git \
    libpoco-dev libeigen3-dev \
    python3-dev
```

For USB serial devices later (force sensor, MagTec skin):

```bash
sudo usermod -aG dialout $USER
# log out and back in for group membership
```

### Step 3 — Real-time kernel (recommended for live control)

Follow the official Franka guide for a low-latency kernel:

https://frankaemika.github.io/docs/installation_linux.html#setting-up-the-real-time-kernel

Skipping this step is OK for a first install check; control may drop frames under load.

### Step 4 — Conda / Mamba

If you do not have conda yet, install [Miniforge](https://github.com/conda-forge/miniforge) (recommended):

```bash
curl -fsSL -o Miniforge3.sh \
  https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-Linux-x86_64.sh
bash Miniforge3.sh -b -p ~/miniforge3
~/miniforge3/bin/conda init bash
source ~/.bashrc
```

Create the project environment:

```bash
cd ~/franka_cartesian_controller
mamba env create -f environment.yml    # or: conda env create -f environment.yml
conda activate franka_interface
```

### Step 5 — libfranka runtime path

Vendored **libfranka 0.9.2** lives under `pyfranka_interface/third_party/libfranka/`.

**Fix linker symlinks** (required on a fresh clone — the repo may only ship `libfranka.so.0.9.2`):

```bash
cd ~/franka_cartesian_controller/pyfranka_interface/third_party/libfranka/lib
ln -sf libfranka.so.0.9.2 libfranka.so
ln -sf libfranka.so.0.9.2 libfranka.so.0.9
```

Add to `~/.bashrc` so every new shell finds the library:

```bash
echo 'export LD_LIBRARY_PATH=$HOME/franka_cartesian_controller/pyfranka_interface/third_party/libfranka/lib:${LD_LIBRARY_PATH:-}' >> ~/.bashrc
source ~/.bashrc
```

Or per session:

```bash
export LD_LIBRARY_PATH=$HOME/franka_cartesian_controller/pyfranka_interface/third_party/libfranka/lib:$LD_LIBRARY_PATH
```

If your robot control box needs a **different libfranka version**, build the matching release from [libfranka](https://github.com/frankaemika/libfranka) and replace `pyfranka_interface/third_party/libfranka/`.

### Step 6 — Build `pyfranka_interface`

Activate the env, then build with **setup.py** (recommended — works out of the box):

```bash
conda activate franka_interface
export LD_LIBRARY_PATH=$HOME/franka_cartesian_controller/pyfranka_interface/third_party/libfranka/lib:$LD_LIBRARY_PATH

cd ~/franka_cartesian_controller/pyfranka_interface
python setup.py build_ext --inplace
python setup.py install
```

If `waf` is available and `wscript` exists in `pyfranka_interface/`, you can use `python3 ./waf configure --python && python3 ./waf` instead of `setup.py`.

### Step 7 — Software test (no robot)

```bash
conda activate franka_interface
cd ~/franka_cartesian_controller
python scripts/test_import.py
```

Expected output includes `OK — module loaded`.

One-liner equivalent:

```bash
python3 -c "import pyfranka_interface as franka; print('pyfranka_interface OK')"
```

---

## Robot connection & hardware test

See [docs/ROBOT_CONNECTION.md](docs/ROBOT_CONNECTION.md) for full network and Desk/FCI steps.

### Network (summary)

| Field | Example |
|-------|---------|
| PC address | `192.168.2.x` (same subnet as robot, e.g. DHCP or static) |
| Netmask | `255.255.255.0` |
| Robot IP | `192.168.2.10` |
| Desk URL | http://192.168.2.10 |

```bash
ping -c 3 192.168.2.10
```

On **Desk**: unlock brakes, activate **FCI**, clear any reflexes.

### Step 8 — Robot test (read joints, no motion)

```bash
conda activate franka_interface
export LD_LIBRARY_PATH=$HOME/franka_cartesian_controller/pyfranka_interface/third_party/libfranka/lib:$LD_LIBRARY_PATH

# optional if your robot IP differs
export ROBOT_IP=192.168.2.10

cd ~/franka_cartesian_controller
python scripts/test_robot.py
```

This connects, prints joint angles and end-effector pose, and exits without commanding motion.

### Step 9 — Minimal motion example (optional)

Only when the workspace is clear and FCI is active:

```python
import pyfranka_interface as franka
import numpy as np

robot = franka.Robot_("192.168.2.10", False, hand_franka=False, auto_init=True, speed_factor=0.1)
print("joints:", robot.getState().q)

target = np.eye(4)
target[:3, 3] = [0.500, 0.420, 0.034]
robot.move("absolute", target, 2.0)
```

More API examples: [docs/USAGE_PYTHON.md](docs/USAGE_PYTHON.md) and `pyfranka_interface/src/examples/`.

---

## Every new terminal session

```bash
conda activate franka_interface
# LD_LIBRARY_PATH should already be in ~/.bashrc from Step 5
```

---

## What you get

- **Python** (`pyfranka_interface`): `Robot_` with `move`, `move_joints`, `extMove`, state queries, and custom control callbacks.
- **C++** (`cartesian_franka`): headers and examples under `pyfranka_interface/src/`.
- **Build**: vendored `libfranka` 0.9.2 + `pybind11` via conda — no separate upstream clones required.

---

## Troubleshooting

| Symptom | Fix |
|---------|-----|
| `ImportError: libfranka.so.0.9` | Create `libfranka.so` symlinks (Step 5) and set `LD_LIBRARY_PATH` |
| `cannot find -lfranka` at link time | Same symlink script |
| `waf configure` / missing `wscript` | Build with `python setup.py build_ext --inplace && python setup.py install` |
| `conda: command not found` | Install Miniforge (Step 4) |
| Robot connection fails | [docs/ROBOT_CONNECTION.md](docs/ROBOT_CONNECTION.md) — ping, FCI, brakes |
| `reflex_aborted` during motion | Lower `speed_factor`, reset reflexes on Desk |
| Permission denied on `/dev/ttyUSB*` | `sudo usermod -aG dialout $USER`, re-login |

---

## MagTec tactile skin (optional)

Not required for the Franka controller alone.

```bash
pip install -r requirements-magtec.txt
```

**→** [magtec_models/README.md](magtec_models/README.md)  
**→** [New data collection guide](magtec_models/docs/guides/DATA_COLLECTION.md)

---

## Documentation index

| Topic | Guide |
|-------|--------|
| Install details | [docs/INSTALL.md](docs/INSTALL.md) |
| Connect to the robot | [docs/ROBOT_CONNECTION.md](docs/ROBOT_CONNECTION.md) |
| Python API | [docs/USAGE_PYTHON.md](docs/USAGE_PYTHON.md) |
| C++ API | [docs/USAGE_CPP.md](docs/USAGE_CPP.md) |
| Upstream build notes | [pyfranka_interface/README.md](pyfranka_interface/README.md) |
| MagTec data collection | [magtec_models/docs/guides/DATA_COLLECTION.md](magtec_models/docs/guides/DATA_COLLECTION.md) |

---

## Authors

- **Gabriele Giudici** — main author / maintainer
- **Valerio Modugno** — co-author (original `pyfranka_interface`)

Full list: [AUTHORS.md](AUTHORS.md).

Intended for any lab with a Franka Research 3 / Panda — no lab-specific credentials or paths are baked in.

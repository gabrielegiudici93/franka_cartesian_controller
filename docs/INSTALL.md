# Installation — Franka Cartesian Controller

Step-by-step build & install. Tested on Ubuntu 20.04 / 22.04.

---

## 1. System prerequisites

```bash
sudo apt update
sudo apt install -y \
    build-essential cmake git \
    libpoco-dev libeigen3-dev \
    python3-dev
```

Make sure you are a member of the `dialout` group (for FT and skin USB-serial later):

```bash
sudo usermod -aG dialout $USER
# log out and back in
```

---

## 2. Real-time kernel (recommended)

The Franka requires a low-latency / real-time kernel for stable control. Follow Franka Emika's official guide:

https://frankaemika.github.io/docs/installation_linux.html#setting-up-the-real-time-kernel

If you skip this step, control will still work but may drop frames.

---

## 3. Conda / Mamba environment

```bash
cd ~/franka_cartesian_controller
conda env create -f environment.yml
conda activate franka_interface
```

(`mamba env create -f environment.yml` also works.)

---

## 4. libfranka

The Franka C++ library v0.9.2 is **already vendored** under:

```
pyfranka_interface/third_party/libfranka/
  ├── include/
  └── lib/  (libfranka.so, libfranka.so.0.9, libfranka.so.0.9.2)
```

You must expose it to the linker each shell session you build/run in:

```bash
export LD_LIBRARY_PATH=$HOME/franka_cartesian_controller/pyfranka_interface/third_party/libfranka/lib:$LD_LIBRARY_PATH
```

Add the export line to your `~/.bashrc` for convenience.

**If your robot uses a Franka Control version that requires a different libfranka**, follow Franka's docs to build the matching libfranka from source, then replace the contents of `pyfranka_interface/third_party/libfranka/` with your build.

---

## 5. Build & install `pyfranka_interface`

```bash
cd ~/franka_cartesian_controller/pyfranka_interface
./run_build.sh
```

`run_build.sh` runs `waf configure --python && waf`, copies the resulting `.so` into `src/`, and runs `python setup.py install` into the active conda env.

Verify:

```bash
python3 -c "import pyfranka_interface as franka; print('pyfranka_interface OK')"
```

---

## 6. MagTec skin stack (optional)

If you also want the tactile-skin examples (visualization, data collection, training):

```bash
cd ~/franka_cartesian_controller
pip install -r requirements-magtec.txt
```

Then configure hardware:

```bash
cd magtec_models
cp config/hardware.example.yaml config/hardware.yaml
# edit ROBOT_IP, FT_PORT, STRETCHMAGTEC_PORT, reference position, initial joints
python3 src/franka_controller/config.py     # prints loaded config
```

---

## 7. First robot test

→ [ROBOT_CONNECTION.md](ROBOT_CONNECTION.md) for network + FCI activation.

→ [USAGE_PYTHON.md](USAGE_PYTHON.md) for a minimal motion example.

---

## Troubleshooting

| Symptom | Fix |
|---------|-----|
| `ImportError: libfranka.so.0.9: cannot open shared object file` | `export LD_LIBRARY_PATH=$HOME/franka_cartesian_controller/pyfranka_interface/third_party/libfranka/lib:$LD_LIBRARY_PATH` |
| `waf configure` fails on pybind11 | check `conda activate franka_interface` and `which python3` |
| Robot not reachable | see [ROBOT_CONNECTION.md](ROBOT_CONNECTION.md) |
| `reflex_aborted` mid-motion | reduce `ROBOT_SPEED_FACTOR`, unlock brakes on Desk, reset reflexes |
| Permission denied on `/dev/ttyUSB*` | `sudo usermod -aG dialout $USER`, then log out/in |
